// SPDX-License-Identifier: Apache-2.0
//
// EuRoC (ASL format) dataset reader. See kestrel/io/euroc.hpp.

#include "kestrel/io/euroc.hpp"

#include <filesystem>
#include <fstream>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>

namespace kestrel {
namespace {

namespace fs = std::filesystem;

/// Reads a CSV file, skipping '#' comment lines, into rows of fields.
std::vector<std::vector<std::string>> readCsv(const fs::path& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("EuRoC: cannot open " + path.string());
  std::vector<std::vector<std::string>> rows;
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();  // CRLF files
    if (line.empty() || line[0] == '#') continue;
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) fields.push_back(field);
    rows.push_back(std::move(fields));
  }
  return rows;
}

/// Camera index: timestamp [ns] → image path.
std::map<long long, std::string> readCamera(const fs::path& cam_dir) {
  std::map<long long, std::string> out;
  for (const auto& row : readCsv(cam_dir / "data.csv")) {
    if (row.size() < 2) continue;
    out[std::stoll(row[0])] = (cam_dir / "data" / row[1]).string();
  }
  return out;
}

}  // namespace

EurocDataset::EurocDataset(const std::string& root_str) {
  const fs::path root = fs::path(root_str) / "mav0";
  if (!fs::is_directory(root)) {
    throw std::runtime_error("EuRoC: '" + root_str + "' does not contain a mav0/ folder");
  }

  // Stereo pairs: EuRoC cameras are hardware-triggered, so cam0 and cam1 share timestamps.
  const auto cam0 = readCamera(root / "cam0");
  const auto cam1 = readCamera(root / "cam1");
  for (const auto& [ns, left] : cam0) {
    const auto it = cam1.find(ns);
    if (it == cam1.end()) continue;
    images_.push_back({ns * 1e-9, left, it->second});
  }
  if (images_.empty()) throw std::runtime_error("EuRoC: no synchronized stereo pairs found");

  for (const auto& row : readCsv(root / "imu0" / "data.csv")) {
    if (row.size() < 7) continue;
    ImuSample s;
    s.t = std::stoll(row[0]) * 1e-9;
    s.gyr = Eigen::Vector3d(std::stod(row[1]), std::stod(row[2]), std::stod(row[3]));
    s.acc = Eigen::Vector3d(std::stod(row[4]), std::stod(row[5]), std::stod(row[6]));
    imu_.push_back(s);
  }

  // Ground truth: EuRoC ships state_groundtruth_estimate0; TUM-VI ships mocap0 (motion-capture
  // poses). Both start with: timestamp, p_x, p_y, p_z, q_w, q_x, q_y, q_z.
  fs::path gt = root / "state_groundtruth_estimate0" / "data.csv";
  if (!fs::exists(gt)) gt = root / "mocap0" / "data.csv";
  if (fs::exists(gt)) {
    for (const auto& row : readCsv(gt)) {
      if (row.size() < 8) continue;
      const gtsam::Rot3 R = gtsam::Rot3::Quaternion(std::stod(row[4]), std::stod(row[5]),
                                                    std::stod(row[6]), std::stod(row[7]));
      const gtsam::Point3 p(std::stod(row[1]), std::stod(row[2]), std::stod(row[3]));
      ground_truth_.push_back({std::stoll(row[0]) * 1e-9, gtsam::Pose3(R, p)});
    }
  }
}

std::pair<cv::Mat, cv::Mat> EurocDataset::loadStereo(std::size_t index) const {
  const auto& e = images_.at(index);
  cv::Mat left = cv::imread(e.left_path, cv::IMREAD_GRAYSCALE);
  cv::Mat right = cv::imread(e.right_path, cv::IMREAD_GRAYSCALE);
  if (left.empty() || right.empty()) {
    throw std::runtime_error("EuRoC: cannot read image pair " + e.left_path);
  }
  return {left, right};
}

}  // namespace kestrel
