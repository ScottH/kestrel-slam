// SPDX-License-Identifier: Apache-2.0
//
// kestrel/io/euroc.hpp — reader for the EuRoC MAV dataset in "ASL" folder format.
//
// Layout (Burri et al., IJRR 2016):
//   <root>/mav0/cam0/data.csv           #timestamp [ns],filename
//   <root>/mav0/cam0/data/<ns>.png
//   <root>/mav0/cam1/...                 (same timestamps as cam0: hardware-synchronized)
//   <root>/mav0/imu0/data.csv           #timestamp [ns],w_x,w_y,w_z [rad/s],a_x,a_y,a_z [m/s^2]
//   <root>/mav0/state_groundtruth_estimate0/data.csv
//                                        #timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v...,b_w...,b_a...
//
// The same "ASL" layout is produced by several converters, so the reader also works for
// TUM-VI-style exports and for our own recordings once converted (planned with the bag runner).

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

#include "kestrel/imu/imu.hpp"

namespace kestrel {

struct StereoImageEntry {
  double t = 0.0;  ///< seconds
  std::string left_path, right_path;
};

struct GroundTruthPose {
  double t = 0.0;
  gtsam::Pose3 T_world_body;  ///< body = IMU frame
};

class EurocDataset {
 public:
  /// Opens `root` (the folder containing `mav0/`). Throws std::runtime_error with a readable
  /// message if the layout is wrong.
  explicit EurocDataset(const std::string& root);

  const std::vector<StereoImageEntry>& images() const { return images_; }
  const std::vector<ImuSample>& imu() const { return imu_; }
  /// Empty if the sequence has no ground truth.
  const std::vector<GroundTruthPose>& groundTruth() const { return ground_truth_; }

  /// Loads a stereo pair as 8-bit grayscale. Throws if an image cannot be read.
  std::pair<cv::Mat, cv::Mat> loadStereo(std::size_t index) const;

 private:
  std::vector<StereoImageEntry> images_;
  std::vector<ImuSample> imu_;
  std::vector<GroundTruthPose> ground_truth_;
};

}  // namespace kestrel
