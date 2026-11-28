// SPDX-License-Identifier: Apache-2.0
//
// Synthetic moving object for real images. See kestrel/eval/dynamic_overlay.hpp.

#include "kestrel/eval/dynamic_overlay.hpp"

#include <cmath>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace kestrel {
namespace {

cv::Mat rectifiedPositions(const StereoRig& rig, int width, int height, bool left) {
  std::vector<cv::Point2f> raw;
  raw.reserve(static_cast<std::size_t>(width) * height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) raw.emplace_back(static_cast<float>(x), static_cast<float>(y));
  }
  const auto rect = left ? rig.rectifyLeft(raw) : rig.rectifyRight(raw);
  cv::Mat map(height, width, CV_32FC2);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) map.at<cv::Point2f>(y, x) = rect[y * width + x];
  }
  return map;
}

}  // namespace

DynamicOverlay::DynamicOverlay(const StereoRig& rig, int width, int height,
                               const DynamicOverlayOptions& options)
    : options_(options),
      width_(width),
      height_(height),
      disparity_(rig.focalLength() * rig.baseline() / options.depth_m) {
  // A box-and-disc texture with plenty of corners, like a stack of labelled cartons.
  const int tw = static_cast<int>(options.width_fraction * width);
  const int th = static_cast<int>(options.height_fraction * height);
  texture_ = cv::Mat(th, tw, CV_8UC1, cv::Scalar(90));
  cv::RNG rng(options.seed);
  for (int i = 0; i < tw * th / 400; ++i) {
    const cv::Point c(rng.uniform(0, tw), rng.uniform(0, th));
    const int s = rng.uniform(4, 24);
    const cv::Scalar col(rng.uniform(0, 255));
    if (i % 2) {
      cv::rectangle(texture_, c, c + cv::Point(s, rng.uniform(4, 24)), col, cv::FILLED);
    } else {
      cv::circle(texture_, c, s / 2, col, cv::FILLED);
    }
  }
  cv::GaussianBlur(texture_, texture_, cv::Size(3, 3), 0);
  map_left_ = rectifiedPositions(rig, width, height, true);
  map_right_ = rectifiedPositions(rig, width, height, false);
}

bool DynamicOverlay::position(double t, double* x0) const {
  if (t < options_.start_s) return false;
  const double phase = std::fmod(t - options_.start_s, options_.period_s) / options_.crossing_s;
  if (phase > 1.0) return false;
  // From just outside the left border to just outside the right border.
  *x0 = -texture_.cols + phase * (width_ + texture_.cols);
  return true;
}

cv::Mat DynamicOverlay::apply(double t, cv::Mat* left, cv::Mat* right) const {
  double x0 = 0.0;
  if (!position(t, &x0)) return cv::Mat();
  const double y0 = 0.5 * (height_ - texture_.rows);
  cv::Mat mask(height_, width_, CV_8UC1, cv::Scalar(0));
  // Right camera: the same object, shifted left by its disparity in rectified coordinates.
  const struct {
    cv::Mat* image;
    const cv::Mat* map;
    double x0;
    bool left;
  } views[] = {{left, &map_left_, x0, true}, {right, &map_right_, x0 - disparity_, false}};
  for (const auto& view : views) {
    for (int y = 0; y < height_; ++y) {
      auto* row = view.image->ptr<uint8_t>(y);
      const auto* map = view.map->ptr<cv::Point2f>(y);
      for (int x = 0; x < width_; ++x) {
        const int u = static_cast<int>(std::floor(map[x].x - view.x0));
        const int v = static_cast<int>(std::floor(map[x].y - y0));
        if (u < 0 || v < 0 || u >= texture_.cols || v >= texture_.rows) continue;
        row[x] = texture_.at<uint8_t>(v, u);
        if (view.left) mask.at<uint8_t>(y, x) = 255;
      }
    }
  }
  return mask;
}

}  // namespace kestrel
