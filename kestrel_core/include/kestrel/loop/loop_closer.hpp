// SPDX-License-Identifier: Apache-2.0
//
// kestrel/loop/loop_closer.hpp — loop detection, geometric verification and the pose graph.
//
// TUTORIAL NOTE
// -------------
// Theory: docs/theory/08_place_recognition_and_loop_closure.md.  References: [GalvezLopez12],
// [Lepetit09] (EPnP), [Fischler81] (RANSAC), [Kaess12] (iSAM2), [Qin18] (VINS pose graph).
//
// Odometry drifts: every keyframe's error builds on the previous one. When the robot returns to a
// place it has seen, recognizing it gives a constraint that ties "now" to "then" and lets the
// accumulated drift be distributed back over the whole loop. For each keyframe:
//
//   1. Describe it: ORB features on the left image (OpenCV); stereo matches along rectified
//      rows give many of them a 3D position in the keyframe's camera frame.
//   2. Propose (PlaceRecognizer, bag of words): the most similar keyframe that is not in the
//      recent past. The score is NORMALIZED by the query's similarity to its own previous
//      keyframe, so one threshold works in both rich and bland scenes.
//   3. Require temporal consistency: the same place must be proposed for a few consecutive
//      keyframes. A false loop is catastrophic (it bends the whole map); a missed loop only
//      costs a little accuracy — so we err on the side of caution.
//   4. Verify geometrically (OpenCV): match the query's descriptors to the candidate's 3D points
//      and solve PnP with RANSAC. Only many inliers with a consistent pose make a loop. This
//      also measures the relative pose between the two keyframes.
//   5. Optimize the pose graph (GTSAM iSAM2): keyframe poses linked by odometry edges (from the
//      VIO) and loop edges (from step 4; Gaussian — see addLoopConstraint()). The result is the drift
//      correction T_map_odom that is applied to every odometry pose.
//
// Runs in its own thread (live) or inline (offline, deterministic), set by
// `loop_closure.synchronous`. Odometry never waits for it.

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <condition_variable>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <thread>
#include <vector>

#include "kestrel/config.hpp"
#include "kestrel/loop/place_recognizer.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace gtsam {
class ISAM2;
}

namespace kestrel {

/// What the odometry hands over for each keyframe.
struct KeyframeInput {
  int id = -1;                 ///< increasing keyframe id
  double t = 0.0;
  gtsam::Pose3 T_odom_body;    ///< odometry estimate of the body pose
  cv::Mat left, right;         ///< raw 8-bit images
};

/// A verified loop closure.
struct LoopEvent {
  int query_id = -1, match_id = -1;
  double t_query = 0.0, t_match = 0.0;
  gtsam::Pose3 T_match_query;  ///< body pose of the query keyframe in the match's body frame
  int inliers = 0;
  double score = 0.0;          ///< normalized BoW score
};

/// The geometric description of a keyframe used for verification.
struct KeyframeFeatures {
  std::vector<cv::Point2f> rectified;  ///< left keypoints, rectified pixels
  cv::Mat descriptors;                 ///< ORB, one row per keypoint
  std::vector<cv::Point3f> points;     ///< 3D in the rectified left camera (NaN if no stereo)
  int with_depth = 0;
};

class LoopCloser {
 public:
  /// `recognizer` may be null: then only odometry edges are kept (loop detection disabled).
  LoopCloser(const LoopClosureConfig& config, std::shared_ptr<const StereoRig> rig,
             std::unique_ptr<PlaceRecognizer> recognizer);
  ~LoopCloser();

  LoopCloser(const LoopCloser&) = delete;
  LoopCloser& operator=(const LoopCloser&) = delete;

  /// Queues (threaded) or processes (synchronous) one keyframe.
  void addKeyframe(KeyframeInput keyframe);

  /// Blocks until all queued keyframes are processed.
  void waitUntilIdle();

  /// Drift correction: map ← odometry. Identity until the first loop closure.
  gtsam::Pose3 mapFromOdom() const;
  std::vector<LoopEvent> loops() const;
  /// Optimized keyframe poses in the map frame: (keyframe id, time, pose).
  std::vector<std::tuple<int, double, gtsam::Pose3>> keyframePoses() const;

  // --- building blocks, public for tests and tools ------------------------------------------------

  /// ORB features with stereo depth for one stereo pair.
  KeyframeFeatures describe(const cv::Mat& left, const cv::Mat& right) const;

  /// Relative pose of `query` w.r.t. `match` (rectified-camera frames: T_match_query) by matching
  /// query descriptors to match's 3D points and PnP-RANSAC. Returns the number of inliers (0 on
  /// failure).
  int verify(const KeyframeFeatures& match, const KeyframeFeatures& query,
             gtsam::Pose3* T_matchcam_querycam) const;

  /// Adds a loop constraint between two known keyframes and re-optimizes (also used by tests).
  void addLoopConstraint(int match_id, int query_id, const gtsam::Pose3& T_match_query);

 private:
  struct Record {
    int id;
    double t;
    gtsam::Pose3 T_odom_body;
    KeyframeFeatures features;
  };

  void process(KeyframeInput keyframe);
  void addOdometryNode(const Record& record);
  void optimize(int iterations);
  void workerLoop();

  LoopClosureConfig config_;
  std::shared_ptr<const StereoRig> rig_;
  std::unique_ptr<PlaceRecognizer> recognizer_;
  std::unique_ptr<gtsam::ISAM2> isam_;

  // Only the processing thread touches these (no lock needed).
  std::map<int, Record> records_;
  int last_id_ = -1;
  int consistent_count_ = 0;
  int last_candidate_ = -1;

  // Results read by other threads, guarded by state_mutex_ (held only briefly, so the odometry
  // thread calling mapFromOdom() never waits for an optimization).
  mutable std::mutex state_mutex_;
  std::map<int, gtsam::Pose3> optimized_;  // keyframe id → map pose
  std::map<int, double> times_;            // keyframe id → time
  std::vector<LoopEvent> loops_;
  gtsam::Pose3 T_map_odom_;

  // Worker thread and its queue, guarded by queue_mutex_.
  std::mutex queue_mutex_;
  std::deque<KeyframeInput> queue_;
  std::condition_variable queue_cv_, idle_cv_;
  bool busy_ = false, stop_ = false;
  std::thread worker_;
};

}  // namespace kestrel
