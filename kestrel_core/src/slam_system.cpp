// SPDX-License-Identifier: Apache-2.0
//
// Odometry + loop closure. See kestrel/slam_system.hpp.

#include "kestrel/slam_system.hpp"

#include <filesystem>

#include "kestrel/common/logging.hpp"

namespace kestrel {

SlamSystem::SlamSystem(const Config& config) : config_(config), odometry_(config) {
  if (!config.loop_closure.enable) return;
  std::unique_ptr<PlaceRecognizer> recognizer;
  const std::string& voc = config.loop_closure.vocabulary_path;
  if (voc.empty() || !std::filesystem::exists(voc)) {
    KLOG_WARN("loop closure: vocabulary '" << voc
                                           << "' not found (train one with train_vocabulary)");
  } else {
    recognizer = makeFbowPlaceRecognizer(voc);
    KLOG_INFO("loop closure: vocabulary " << voc);
  }
  // The rig is shared with the odometry (same calibration and rectification).
  loop_closer_ = std::make_unique<LoopCloser>(
      config.loop_closure,
      std::shared_ptr<const StereoRig>(&odometry_.rig(), [](const StereoRig*) {}),
      std::move(recognizer));
}

std::optional<SlamSystem::Output> SlamSystem::addStereo(double t, const cv::Mat& left,
                                                        const cv::Mat& right, const cv::Mat& mask) {
  const auto odo = odometry_.addStereo(t, left, right, mask);
  if (!odo) return std::nullopt;
  Output out;
  out.odometry = *odo;
  if (loop_closer_ && odo->initialized && odo->is_keyframe) {
    out.keyframe_id = next_keyframe_id_++;
    loop_closer_->addKeyframe({out.keyframe_id, odo->t, odo->state.pose(), left, right});
  }
  out.T_map_odom = loop_closer_ ? loop_closer_->mapFromOdom() : gtsam::Pose3();
  out.T_map_body = out.T_map_odom * odo->state.pose();
  return out;
}

void SlamSystem::finish() {
  if (loop_closer_) loop_closer_->waitUntilIdle();
}

}  // namespace kestrel
