// SPDX-License-Identifier: Apache-2.0
//
// The dynamic heat grid. See kestrel/frontend/dynamic_heat_grid.hpp.

#include "kestrel/frontend/dynamic_heat_grid.hpp"

#include <algorithm>

namespace kestrel {

DynamicHeatGrid::DynamicHeatGrid(const DynamicConfig& config, int image_width, int image_height)
    : config_(config),
      width_(image_width),
      height_(image_height),
      heat_(config.grid_rows, config.grid_cols, CV_64F, cv::Scalar(0.0)) {}

cv::Rect DynamicHeatGrid::cellRect(int row, int col) const {
  // Integer cell borders that tile the image exactly, also when it does not divide evenly.
  const int x0 = col * width_ / config_.grid_cols, x1 = (col + 1) * width_ / config_.grid_cols;
  const int y0 = row * height_ / config_.grid_rows, y1 = (row + 1) * height_ / config_.grid_rows;
  return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

void DynamicHeatGrid::update(const std::vector<TrackedFeature>& features,
                             const std::unordered_set<int>& flagged) {
  cv::Mat n_features(heat_.size(), CV_64F, cv::Scalar(0.0));
  cv::Mat n_flagged(heat_.size(), CV_64F, cv::Scalar(0.0));
  for (const auto& f : features) {
    const int col = std::clamp(static_cast<int>(f.left_raw.x * config_.grid_cols / width_), 0,
                               config_.grid_cols - 1);
    const int row = std::clamp(static_cast<int>(f.left_raw.y * config_.grid_rows / height_), 0,
                               config_.grid_rows - 1);
    n_features.at<double>(row, col) += 1.0;
    if (flagged.count(f.id)) n_flagged.at<double>(row, col) += 1.0;
  }
  const double a = config_.heat_decay;
  heat_ = a * heat_ + (1.0 - a) * n_flagged / (n_features + 1.0);
}

int DynamicHeatGrid::hotCells() const {
  int n = 0;
  for (int r = 0; r < heat_.rows; ++r) {
    for (int c = 0; c < heat_.cols; ++c) n += heat_.at<double>(r, c) > config_.heat_mask_threshold;
  }
  return n;
}

double DynamicHeatGrid::blockedFraction() const {
  double area = 0.0;
  for (int r = 0; r < heat_.rows; ++r) {
    for (int c = 0; c < heat_.cols; ++c) {
      if (heat_.at<double>(r, c) > config_.heat_mask_threshold) area += cellRect(r, c).area();
    }
  }
  return area / (static_cast<double>(width_) * height_);
}

cv::Mat DynamicHeatGrid::detectionMask() const {
  if (hotCells() == 0) return cv::Mat();
  cv::Mat mask(height_, width_, CV_8UC1, cv::Scalar(0));
  for (int r = 0; r < heat_.rows; ++r) {
    for (int c = 0; c < heat_.cols; ++c) {
      if (heat_.at<double>(r, c) > config_.heat_mask_threshold) mask(cellRect(r, c)).setTo(255);
    }
  }
  return mask;
}

}  // namespace kestrel
