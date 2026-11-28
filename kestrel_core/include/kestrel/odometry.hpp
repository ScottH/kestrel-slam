// SPDX-License-Identifier: Apache-2.0
//
// kestrel/odometry.hpp — the visual-inertial odometry pipeline: IMU buffer → front-end →
// estimator, with the feedback between them.
//
// TUTORIAL NOTE
// -------------
// This class is the "wiring diagram" of the odometry, kept separate from the algorithms so that
// each piece stays testable on its own:
//
//   addImu() ──► IMU buffer ───────────────┬──────────────────────────────┐
//                                          │ gyro rotation between frames │ preintegration
//   addStereo() ──► FeatureTracker ◄───────┘                              ▼
//                       │  FeatureFrame                              VioEstimator
//                       └──────────────────────────────────────────────►  │
//                       ▲            outlier track ids (feedback)          │
//                       ├──────────────────────────────────────────────────┤
//                       │   detection mask ◄── DynamicHeatGrid ◄───────────┘
//                       │                      (where outliers cluster, M4)
//
// It is synchronous and single-threaded on purpose: the ROS node (M9) and the offline runners
// call it from one thread, which makes runs deterministic and easy to debug. The caller must
// deliver IMU samples up to a frame's timestamp before calling addStereo() for that frame.

#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <vector>

#include "kestrel/config.hpp"
#include "kestrel/estimator/vio_estimator.hpp"
#include "kestrel/frontend/dynamic_heat_grid.hpp"
#include "kestrel/frontend/feature_tracker.hpp"
#include "kestrel/imu/imu.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace kestrel {

class VisualInertialOdometry {
 public:
  explicit VisualInertialOdometry(const Config& config);

  /// Buffers one IMU sample (timestamps must increase).
  void addImu(const ImuSample& sample);

  /// Processes a stereo pair (raw, 8-bit grayscale). Returns nullopt — without consuming the
  /// frame — if the IMU has not yet reached time t; call again after more addImu() calls.
  /// `mask` (optional, CV_8U, raw left image size): non-zero pixels are excluded, e.g. people
  /// from a segmentation network (theory ch. 07 §5).
  std::optional<VioEstimator::Output> addStereo(double t, const cv::Mat& left, const cv::Mat& right,
                                                const cv::Mat& mask = cv::Mat());

  /// IMU-rate prediction: latest estimate propagated to time t using buffered IMU samples.
  /// Returns nullopt before initialization or if t is outside the buffered IMU data.
  std::optional<gtsam::NavState> predict(double t) const;

  /// The front-end output of the last processed frame (for visualization).
  const FeatureFrame& lastFrame() const { return last_frame_; }
  const StereoRig& rig() const { return *rig_; }
  const VioEstimator& estimator() const { return estimator_; }
  /// The dynamic heat grid, or nullptr when dynamic handling is disabled.
  const DynamicHeatGrid* heatGrid() const { return heat_ ? &*heat_ : nullptr; }

 private:
  std::vector<ImuSample> imuSince(double t) const;

  Config config_;
  std::shared_ptr<StereoRig> rig_;
  FeatureTracker tracker_;
  VioEstimator estimator_;
  std::optional<DynamicHeatGrid> heat_;
  ImuParamsPtr imu_params_;
  std::deque<ImuSample> imu_;
  std::optional<double> last_frame_t_;
  FeatureFrame last_frame_;
};

}  // namespace kestrel
