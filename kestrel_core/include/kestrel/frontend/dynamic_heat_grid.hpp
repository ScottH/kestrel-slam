// SPDX-License-Identifier: Apache-2.0
//
// kestrel/frontend/dynamic_heat_grid.hpp — spatial memory of where moving things are.
//
// TUTORIAL NOTE
// -------------
// Theory: docs/theory/07_dynamic_objects.md §4.
//
// A moving object rarely spoils one feature: it spoils a cluster, and it keeps doing so in the
// next frames. The estimator finds these features one by one, after they have been tracked and
// often already used. The heat grid remembers WHERE in the image this has been happening, so the
// front-end stops planting new features there for a while:
//
//     at each keyframe, per cell c:   h_c ← α·h_c + (1 − α) · n_flagged,c / (n_features,c + 1)
//     cells with h_c > threshold      → no new detections (existing tracks are left alone)
//
// * A cell full of flagged features heats up within a few keyframes. Isolated mismatches spread
//   over the image barely register, because the ratio, not the count, drives the heat.
// * The "+1" keeps a cell with a single, unlucky feature from jumping to full heat.
// * Heat decays geometrically, so a region is usable again soon after the object has left.
// * It only blocks DETECTION. Tracks already there keep being judged by the estimator, which
//   has the evidence. Blocking them too would throw away good static points that happen to
//   share a cell with a mover.
//
// The grid is in raw (distorted) left-image pixels, where the tracker detects. It does not
// compensate camera rotation. That is acceptable because heat lasts about a second and an object
// that sweeps fast across the image is handled by the estimator's per-landmark gates anyway.

#pragma once

#include <opencv2/core.hpp>
#include <unordered_set>
#include <vector>

#include "kestrel/config.hpp"
#include "kestrel/frontend/feature_tracker.hpp"

namespace kestrel {

class DynamicHeatGrid {
 public:
  DynamicHeatGrid(const DynamicConfig& config, int image_width, int image_height);

  /// One update, at a keyframe: the frame's features and the ids the estimator flagged.
  void update(const std::vector<TrackedFeature>& features, const std::unordered_set<int>& flagged);

  /// CV_8U mask of the image, 255 inside hot cells (empty Mat if no cell is hot).
  cv::Mat detectionMask() const;

  double heat(int row, int col) const { return heat_.at<double>(row, col); }
  int hotCells() const;
  /// Fraction of the image currently blocked for detection (for monitoring: too much = starving).
  double blockedFraction() const;
  int rows() const { return config_.grid_rows; }
  int cols() const { return config_.grid_cols; }
  void reset() { heat_.setTo(0.0); }

 private:
  cv::Rect cellRect(int row, int col) const;

  DynamicConfig config_;
  int width_, height_;
  cv::Mat heat_;  ///< CV_64F, grid_rows × grid_cols
};

}  // namespace kestrel
