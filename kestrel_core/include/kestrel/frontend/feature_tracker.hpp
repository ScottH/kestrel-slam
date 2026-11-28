// SPDX-License-Identifier: Apache-2.0
//
// kestrel/frontend/feature_tracker.hpp — the visual front-end: stereo feature tracking.
//
// TUTORIAL NOTE
// -------------
// Theory: docs/theory/04_feature_tracking.md.  References: [Shi94], [Lucas81], [Bouguet00],
// [Fischler81], [Qin18] (VINS front-end).
//
// Every frame, the tracker turns a raw stereo image pair into a list of *tracked features*: the
// same physical point, identified by a persistent id, observed in the left (and usually right)
// image. The estimator (M3) turns these into GTSAM stereo factors.
//
// All the image processing is OpenCV:
//   cv::goodFeaturesToTrack   Shi–Tomasi corners
//   cv::calcOpticalFlowPyrLK  pyramidal Lucas–Kanade tracking (frame-to-frame and left→right)
//   cv::findFundamentalMat    RANSAC rejection of tracks inconsistent with rigid motion
//   cv::createCLAHE           contrast equalization (IR images are often flat)
//
// What Kestrel adds is the *policy* around them — each a short, explained step in track():
//   1. Gyro-aided prediction. A pure rotation R moves every image point by the homography
//      K·Rᵀ·K⁻¹ (theory ch. 04). Feeding that prediction to KLT as the initial guess keeps
//      tracks alive through fast turns that would otherwise exceed the pyramid's capture range.
//   2. Forward–backward check: track forward, then back; keep a track only if it returns to
//      where it started (catches drift on repetitive texture and occlusion boundaries).
//   3. RANSAC on *rectified* coordinates, so the pinhole epipolar model holds exactly.
//   4. Uniform coverage: new corners are only detected away from existing ones, and existing
//      tracks are thinned longest-first, so long (valuable) tracks win.
//   5. Stereo: KLT from left to right, accepted only if the rectified rows agree and the
//      disparity is positive (the point is in front of the rig).
//   6. Dynamic objects (M4, theory ch. 07): an external mask (e.g. semantic segmentation of
//      people) removes tracks and blocks detection; a detection-only mask (the dynamic heat
//      grid, kestrel/frontend/dynamic_heat_grid.hpp) only blocks NEW corners; and the estimator
//      drops tracks it has flagged as outliers.

#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/StereoPoint2.h>

#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>  // cv::CLAHE
#include <optional>
#include <unordered_set>
#include <vector>

#include "kestrel/config.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace kestrel {

/// One feature in the current frame.
struct TrackedFeature {
  int id = -1;                ///< persistent identity across frames (never reused)
  int track_length = 1;       ///< number of consecutive frames observed (1 = detected this frame)
  cv::Point2f left_raw;       ///< pixel in the raw (distorted) left image
  cv::Point2f left_rect;      ///< pixel in the rectified left image
  cv::Point2f prev_left_raw;  ///< position in the previous frame (valid if track_length > 1)
  bool has_right = false;     ///< stereo match found
  cv::Point2f right_raw;      ///< pixel in the raw right image (valid if has_right)
  cv::Point2f right_rect;     ///< pixel in the rectified right image (valid if has_right)

  /// Rectified stereo observation in GTSAM's convention (uL, uR, v). Requires has_right.
  gtsam::StereoPoint2 stereo() const {
    return gtsam::StereoPoint2(left_rect.x, right_rect.x, left_rect.y);
  }
  float disparity() const { return left_rect.x - right_rect.x; }
};

/// Per-frame bookkeeping: where features came from and where they were lost, and timings.
struct FrontendStats {
  int tracked = 0;      ///< features carried over from the previous frame
  int lost_klt = 0;     ///< KLT failed or left the image
  int lost_fb = 0;      ///< failed the forward–backward check
  int lost_ransac = 0;  ///< rejected by fundamental-matrix RANSAC
  int lost_mask = 0;    ///< inside the external mask
  int dropped_by_backend = 0;
  int thinned = 0;   ///< removed for being too close to a longer track
  int detected = 0;  ///< new corners
  int stereo_matched = 0;
  int stereo_rejected = 0;  ///< KLT found a match but the row/disparity check failed
  double ms_total = 0, ms_preprocess = 0, ms_track = 0, ms_detect = 0, ms_stereo = 0;
};

/// The tracker's output for one stereo frame.
struct FeatureFrame {
  double t = 0.0;
  std::vector<TrackedFeature> features;
  FrontendStats stats;
};

class FeatureTracker {
 public:
  FeatureTracker(const FrontendConfig& config, std::shared_ptr<const StereoRig> rig);

  /// Processes one stereo pair (8-bit grayscale, raw/unrectified).
  ///
  /// @param R_prev_cur  optional orientation of the current body frame expressed in the previous
  ///                    one (e.g. `pim.deltaRij()` from the IMU preintegration between the two
  ///                    frames). Enables gyro-aided prediction.
  /// @param mask        optional CV_8U mask, same size as the images; non-zero pixels are
  ///                    excluded (no features kept or detected there).
  /// @param detection_mask  optional CV_8U mask; non-zero pixels get no NEW features, but
  ///                    existing tracks there are kept (dynamic heat grid).
  FeatureFrame track(double t, const cv::Mat& left, const cv::Mat& right,
                     const std::optional<gtsam::Rot3>& R_prev_cur = std::nullopt,
                     const cv::Mat& mask = cv::Mat(), const cv::Mat& detection_mask = cv::Mat());

  /// Removes tracks by id before the next frame (estimator feedback, e.g. dynamic outliers).
  void dropTracks(const std::vector<int>& ids);

  /// Forgets all tracks (e.g. after a tracking failure).
  void reset();

  /// Draws the frame's features on the raw left image: colour encodes track length
  /// (red = new → green = long), lines show the motion since the previous frame, and a ring
  /// marks features with a stereo match.
  static cv::Mat drawDebug(const cv::Mat& left, const FeatureFrame& frame);

 private:
  struct Track {
    int id;
    int length;
    cv::Point2f raw;
  };

  cv::Mat preprocess(const cv::Mat& image) const;
  std::vector<cv::Point2f> predictWithGyro(const std::vector<cv::Point2f>& prev_raw,
                                           const gtsam::Rot3& R_prev_cur) const;
  void matchStereo(const std::vector<cv::Mat>& left_pyr, const cv::Mat& right,
                   std::vector<TrackedFeature>* features, FrontendStats* stats) const;

  FrontendConfig config_;
  std::shared_ptr<const StereoRig> rig_;
  cv::Ptr<cv::CLAHE> clahe_;
  std::vector<cv::Mat> prev_pyramid_;
  std::vector<Track> tracks_;
  std::unordered_set<int> to_drop_;
  int next_id_ = 0;
};

}  // namespace kestrel
