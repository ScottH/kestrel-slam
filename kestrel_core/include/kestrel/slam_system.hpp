// SPDX-License-Identifier: Apache-2.0
//
// kestrel/slam_system.hpp — the full system: visual-inertial odometry + loop closure.
//
// TUTORIAL NOTE
// -------------
// Two frames of reference, as in ROS's REP-105:
//   * odom: the odometry's world frame — smooth and continuous, but drifts;
//   * map:  the loop-corrected frame — globally consistent, but jumps when a loop closes.
// The loop closer estimates T_map_odom; the SLAM pose is T_map_body = T_map_odom · T_odom_body.
// Controllers that need smoothness use odom, planners that need consistency use map (Nav2 does
// exactly this with the map → odom → base_link TF chain).

#pragma once

#include <memory>
#include <optional>

#include "kestrel/config.hpp"
#include "kestrel/loop/loop_closer.hpp"
#include "kestrel/odometry.hpp"

namespace kestrel {

class SlamSystem {
 public:
  struct Output {
    VioEstimator::Output odometry;  ///< odometry result (odom frame)
    gtsam::Pose3 T_map_odom;        ///< current drift correction
    gtsam::Pose3 T_map_body;        ///< corrected pose (map frame)
    int keyframe_id = -1;           ///< SLAM keyframe id if this frame was a keyframe
  };

  explicit SlamSystem(const Config& config);

  void addImu(const ImuSample& sample) { odometry_.addImu(sample); }

  /// As VisualInertialOdometry::addStereo, plus loop closure on keyframes.
  std::optional<Output> addStereo(double t, const cv::Mat& left, const cv::Mat& right,
                                  const cv::Mat& mask = cv::Mat());

  /// Waits until queued keyframes have gone through loop closure (offline runs, shutdown).
  void finish();

  VisualInertialOdometry& odometry() { return odometry_; }
  const VisualInertialOdometry& odometry() const { return odometry_; }
  /// Null when loop closure is disabled in the configuration.
  const LoopCloser* loopCloser() const { return loop_closer_.get(); }

 private:
  Config config_;
  VisualInertialOdometry odometry_;
  std::unique_ptr<LoopCloser> loop_closer_;
  int next_keyframe_id_ = 0;
};

}  // namespace kestrel
