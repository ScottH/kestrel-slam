// SPDX-License-Identifier: Apache-2.0
//
// The VIO pipeline wiring. See kestrel/odometry.hpp.

#include "kestrel/odometry.hpp"

#include <algorithm>

#include "kestrel/common/logging.hpp"

namespace kestrel {

VisualInertialOdometry::VisualInertialOdometry(const Config& config)
    : config_(config),
      rig_(std::make_shared<StereoRig>(config.camera)),
      tracker_(config.frontend, rig_),
      estimator_(config, rig_),
      imu_params_(makeImuParams(config.imu)) {}

void VisualInertialOdometry::addImu(const ImuSample& sample) {
  if (!imu_.empty() && sample.t <= imu_.back().t) {
    KLOG_WARN("IMU sample at " << sample.t << " is not newer than the last one; dropped");
    return;
  }
  imu_.push_back(sample);
}

std::vector<ImuSample> VisualInertialOdometry::imuSince(double t) const {
  // One sample before t is needed so the interval start can be interpolated.
  auto it = std::upper_bound(imu_.begin(), imu_.end(), t,
                             [](double v, const ImuSample& s) { return v < s.t; });
  if (it != imu_.begin()) --it;
  return std::vector<ImuSample>(it, imu_.end());
}

std::optional<VioEstimator::Output> VisualInertialOdometry::addStereo(double t, const cv::Mat& left,
                                                                      const cv::Mat& right,
                                                                      const cv::Mat& mask) {
  const double t_cam = t + config_.camera.time_offset_s;           // camera clock → IMU clock
  if (imu_.empty() || imu_.back().t < t_cam) return std::nullopt;  // wait for the IMU

  // Gyro-aided tracking: rotation between the previous and this frame, bias-corrected with
  // the estimator's current gyro bias.
  std::optional<gtsam::Rot3> R_prev_cur;
  if (last_frame_t_) {
    gtsam::PreintegratedCombinedMeasurements pim(imu_params_, estimator_.bias());
    const auto samples = imuSince(*last_frame_t_);
    if (integrateImu(samples, *last_frame_t_, t_cam, &pim) > 0) R_prev_cur = pim.deltaRij();
  }
  last_frame_ = tracker_.track(t_cam, left, right, R_prev_cur, mask);

  // The estimator needs IMU from its last keyframe (or the static-init window) up to now.
  const double from = estimator_.hasKeyframe()
                          ? std::min(estimator_.keyframeTime(), t_cam)
                          : t_cam - config_.estimator.init_static_window_s - 0.05;
  auto output = estimator_.process(last_frame_, imuSince(from));
  if (!output.outlier_ids.empty()) tracker_.dropTracks(output.outlier_ids);
  if (output.reset) tracker_.reset();
  last_frame_t_ = t_cam;

  // Keep the buffer bounded: nothing older than the last keyframe (minus a margin) is needed.
  const double keep_from =
      (estimator_.hasKeyframe() ? std::min(estimator_.keyframeTime(), t_cam) : t_cam) -
      std::max(1.0, config_.estimator.init_static_window_s + 0.5);
  while (imu_.size() > 2 && imu_[1].t < keep_from) imu_.pop_front();
  return output;
}

std::optional<gtsam::NavState> VisualInertialOdometry::predict(double t) const {
  if (!estimator_.hasKeyframe() || imu_.empty()) return std::nullopt;
  const double t0 = estimator_.keyframeTime();
  if (t < t0 || t > imu_.back().t) return std::nullopt;
  gtsam::PreintegratedCombinedMeasurements pim(imu_params_, estimator_.bias());
  if (t > t0 && integrateImu(imuSince(t0), t0, t, &pim) <= 0) return std::nullopt;
  return pim.predict(estimator_.keyframeState(), estimator_.bias());
}

}  // namespace kestrel
