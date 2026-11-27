// SPDX-License-Identifier: Apache-2.0
//
// Stereo feature tracking. See kestrel/frontend/feature_tracker.hpp for the overview.

#include "kestrel/frontend/feature_tracker.hpp"

#include <algorithm>
#include <chrono>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace kestrel {
namespace {

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

/// Pixels this close to the border are unreliable for KLT (the window would leave the image).
constexpr float kBorderPx = 8.0f;

bool inside(const cv::Point2f& p, const cv::Size& size) {
  return p.x >= kBorderPx && p.y >= kBorderPx && p.x < size.width - kBorderPx &&
         p.y < size.height - kBorderPx;
}

bool masked(const cv::Mat& mask, const cv::Point2f& p) {
  if (mask.empty()) return false;
  const int x = std::clamp(static_cast<int>(std::lround(p.x)), 0, mask.cols - 1);
  const int y = std::clamp(static_cast<int>(std::lround(p.y)), 0, mask.rows - 1);
  return mask.at<uint8_t>(y, x) != 0;
}

}  // namespace

FeatureTracker::FeatureTracker(const FrontendConfig& config, std::shared_ptr<const StereoRig> rig)
    : config_(config), rig_(std::move(rig)) {
  if (config_.use_clahe) clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
}

void FeatureTracker::reset() {
  prev_pyramid_.clear();
  tracks_.clear();
  to_drop_.clear();
}

void FeatureTracker::dropTracks(const std::vector<int>& ids) {
  to_drop_.insert(ids.begin(), ids.end());
}

cv::Mat FeatureTracker::preprocess(const cv::Mat& image) const {
  CV_Assert(image.type() == CV_8UC1);
  if (!clahe_) return image;
  cv::Mat out;
  clahe_->apply(image, out);
  return out;
}

std::vector<cv::Point2f> FeatureTracker::predictWithGyro(const std::vector<cv::Point2f>& prev_raw,
                                                         const gtsam::Rot3& R_prev_cur) const {
  // Rotation of the rectified-left camera between frames:  R_L'prev_L'cur = R_L'B · R_BprevBcur ·
  // R_BL'.
  const gtsam::Rot3 R_BL = rig_->bodyToRectifiedLeft().rotation();
  const gtsam::Matrix3 R_cur_prev = (R_BL.inverse() * R_prev_cur * R_BL).matrix().transpose();
  const auto& K = *rig_->calibration();

  // A pure rotation maps a viewing ray d_prev to d_cur = R_cur_prev · d_prev — independent of
  // depth, which is why the prediction works without knowing where the points are.
  const auto rect_prev = rig_->rectifyLeft(prev_raw);
  std::vector<cv::Point2f> rect_pred(rect_prev.size());
  for (std::size_t i = 0; i < rect_prev.size(); ++i) {
    const Eigen::Vector3d d_prev((rect_prev[i].x - K.px()) / K.fx(),
                                 (rect_prev[i].y - K.py()) / K.fy(), 1.0);
    const Eigen::Vector3d d_cur = R_cur_prev * d_prev;
    if (d_cur.z() < 0.1) {  // rotated (almost) out of view: no useful prediction
      rect_pred[i] = rect_prev[i];
      continue;
    }
    rect_pred[i] = cv::Point2f(static_cast<float>(K.fx() * d_cur.x() / d_cur.z() + K.px()),
                               static_cast<float>(K.fy() * d_cur.y() / d_cur.z() + K.py()));
  }
  // Back to raw pixels, where KLT runs.
  auto raw_pred = rig_->unrectifyLeft(rect_pred);
  for (std::size_t i = 0; i < raw_pred.size(); ++i) {
    if (!std::isfinite(raw_pred[i].x) || !std::isfinite(raw_pred[i].y)) raw_pred[i] = prev_raw[i];
  }
  return raw_pred;
}

FeatureFrame FeatureTracker::track(double t, const cv::Mat& left_in, const cv::Mat& right_in,
                                   const std::optional<gtsam::Rot3>& R_prev_cur,
                                   const cv::Mat& mask, const cv::Mat& detection_mask) {
  const auto t_start = Clock::now();
  FeatureFrame frame;
  frame.t = t;
  FrontendStats& stats = frame.stats;

  // --- 0. Pre-processing ------------------------------------------------------------------------
  auto t_step = Clock::now();
  const cv::Mat left = preprocess(left_in);
  const cv::Mat right = preprocess(right_in);
  const cv::Size win(config_.klt_window_px, config_.klt_window_px);
  const int levels = config_.klt_pyramid_levels;
  std::vector<cv::Mat> pyramid;
  cv::buildOpticalFlowPyramid(left, pyramid, win, levels);
  stats.ms_preprocess = msSince(t_step);

  // --- 1–3. Track existing features --------------------------------------------------------------
  t_step = Clock::now();
  std::vector<Track> survivors;
  std::vector<cv::Point2f> survivor_prev;
  if (!tracks_.empty() && !prev_pyramid_.empty()) {
    // Backend feedback first: forget tracks the estimator rejected.
    std::vector<Track> candidates;
    candidates.reserve(tracks_.size());
    for (const auto& tr : tracks_) {
      if (to_drop_.count(tr.id)) {
        ++stats.dropped_by_backend;
      } else {
        candidates.push_back(tr);
      }
    }
    std::vector<cv::Point2f> prev_pts(candidates.size());
    for (std::size_t i = 0; i < candidates.size(); ++i) prev_pts[i] = candidates[i].raw;

    // 1. Gyro-aided initial guess (or "no motion").
    const bool predict = R_prev_cur.has_value() && config_.gyro_aided_prediction;
    std::vector<cv::Point2f> cur_pts = predict ? predictWithGyro(prev_pts, *R_prev_cur) : prev_pts;

    const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
    std::vector<uchar> status, back_status;
    std::vector<float> err;
    if (!prev_pts.empty()) {
      cv::calcOpticalFlowPyrLK(prev_pyramid_, pyramid, prev_pts, cur_pts, status, err, win, levels,
                               criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
      // 2. Forward–backward check: track back from the new positions to the old frame.
      std::vector<cv::Point2f> back_pts = prev_pts;
      cv::calcOpticalFlowPyrLK(pyramid, prev_pyramid_, cur_pts, back_pts, back_status, err, win,
                               levels, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
      for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (!status[i] || !inside(cur_pts[i], left.size())) {
          ++stats.lost_klt;
          continue;
        }
        if (!back_status[i] || cv::norm(back_pts[i] - prev_pts[i]) > config_.klt_fb_threshold_px) {
          ++stats.lost_fb;
          continue;
        }
        if (masked(mask, cur_pts[i])) {
          ++stats.lost_mask;
          continue;
        }
        survivors.push_back({candidates[i].id, candidates[i].length + 1, cur_pts[i]});
        survivor_prev.push_back(prev_pts[i]);
      }
    }

    // 3. RANSAC on rectified coordinates: rigid-scene motion must satisfy one epipolar geometry.
    if (survivors.size() >= 8) {
      std::vector<cv::Point2f> cur_raw(survivors.size());
      for (std::size_t i = 0; i < survivors.size(); ++i) cur_raw[i] = survivors[i].raw;
      const auto prev_rect = rig_->rectifyLeft(survivor_prev);
      const auto cur_rect = rig_->rectifyLeft(cur_raw);
      std::vector<uchar> inliers;
      cv::findFundamentalMat(prev_rect, cur_rect, cv::FM_RANSAC, config_.ransac_threshold_px, 0.99,
                             inliers);
      if (inliers.size() == survivors.size()) {
        std::vector<Track> kept;
        std::vector<cv::Point2f> kept_prev;
        for (std::size_t i = 0; i < survivors.size(); ++i) {
          if (inliers[i]) {
            kept.push_back(survivors[i]);
            kept_prev.push_back(survivor_prev[i]);
          } else {
            ++stats.lost_ransac;
          }
        }
        survivors.swap(kept);
        survivor_prev.swap(kept_prev);
      }
    }
  }
  to_drop_.clear();
  stats.ms_track = msSince(t_step);

  // --- 4. Uniform coverage: thin existing tracks (longest first), then detect new corners --------
  t_step = Clock::now();
  std::vector<std::size_t> order(survivors.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return survivors[a].length > survivors[b].length;
  });

  // Detection mask: 255 where new corners may go.
  cv::Mat allowed(left.size(), CV_8UC1, cv::Scalar(0));
  cv::rectangle(allowed,
                cv::Rect(static_cast<int>(kBorderPx), static_cast<int>(kBorderPx),
                         left.cols - 2 * static_cast<int>(kBorderPx),
                         left.rows - 2 * static_cast<int>(kBorderPx)),
                cv::Scalar(255), cv::FILLED);
  if (!mask.empty()) allowed.setTo(0, mask);

  std::vector<Track> next_tracks;
  std::vector<cv::Point2f> next_prev;
  const int radius = config_.min_feature_distance_px;
  for (const std::size_t i : order) {
    const cv::Point2f& p = survivors[i].raw;
    const cv::Point pi(static_cast<int>(std::lround(p.x)), static_cast<int>(std::lround(p.y)));
    if (allowed.at<uint8_t>(pi) == 0) {  // too close to a longer track (or masked): thinned
      ++stats.thinned;
      continue;
    }
    next_tracks.push_back(survivors[i]);
    next_prev.push_back(survivor_prev[i]);
    cv::circle(allowed, pi, radius, cv::Scalar(0), cv::FILLED);
  }
  stats.tracked = static_cast<int>(next_tracks.size());

  const int wanted = config_.max_features - static_cast<int>(next_tracks.size());
  if (wanted > 0) {
    std::vector<cv::Point2f> corners;
    // Hot cells of the dynamic heat grid are closed for new corners only; the thinning above
    // already ran, so existing tracks there survive.
    if (!detection_mask.empty()) allowed.setTo(0, detection_mask);
    cv::goodFeaturesToTrack(left, corners, wanted, 0.01, radius, allowed);
    if (!corners.empty()) {
      cv::cornerSubPix(left, corners, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.01));
    }
    for (const auto& c : corners) {
      next_tracks.push_back({next_id_++, 1, c});
      next_prev.push_back(c);
    }
    stats.detected = static_cast<int>(corners.size());
  }
  stats.ms_detect = msSince(t_step);

  // --- Assemble the output and rectify
  // ------------------------------------------------------------
  std::vector<cv::Point2f> raw(next_tracks.size());
  for (std::size_t i = 0; i < next_tracks.size(); ++i) raw[i] = next_tracks[i].raw;
  const auto rect = rig_->rectifyLeft(raw);
  frame.features.resize(next_tracks.size());
  for (std::size_t i = 0; i < next_tracks.size(); ++i) {
    TrackedFeature& f = frame.features[i];
    f.id = next_tracks[i].id;
    f.track_length = next_tracks[i].length;
    f.left_raw = raw[i];
    f.left_rect = rect[i];
    f.prev_left_raw = next_prev[i];
  }

  // --- 5. Stereo matching
  // --------------------------------------------------------------------------
  t_step = Clock::now();
  matchStereo(pyramid, right, &frame.features, &stats);
  stats.ms_stereo = msSince(t_step);

  tracks_.swap(next_tracks);
  prev_pyramid_ = std::move(pyramid);
  stats.ms_total = msSince(t_start);
  return frame;
}

void FeatureTracker::matchStereo(const std::vector<cv::Mat>& left_pyr, const cv::Mat& right,
                                 std::vector<TrackedFeature>* features,
                                 FrontendStats* stats) const {
  if (features->empty()) return;
  const cv::Size win(config_.klt_window_px, config_.klt_window_px);
  const int levels = config_.klt_pyramid_levels;
  std::vector<cv::Mat> right_pyr;
  cv::buildOpticalFlowPyramid(right, right_pyr, win, levels);

  std::vector<cv::Point2f> left_pts(features->size());
  for (std::size_t i = 0; i < features->size(); ++i) left_pts[i] = (*features)[i].left_raw;

  // Initial guess: the left position. Disparities of tens of pixels are well within the capture
  // range of a 3-level pyramid (≈ window · 2^levels).
  std::vector<cv::Point2f> right_pts = left_pts, back_pts = left_pts;
  std::vector<uchar> status, back_status;
  std::vector<float> err;
  const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 30, 0.01);
  cv::calcOpticalFlowPyrLK(left_pyr, right_pyr, left_pts, right_pts, status, err, win, levels,
                           criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
  cv::calcOpticalFlowPyrLK(right_pyr, left_pyr, right_pts, back_pts, back_status, err, win, levels,
                           criteria, cv::OPTFLOW_USE_INITIAL_FLOW);

  const auto right_rect = rig_->rectifyRight(right_pts);
  // Largest plausible disparity: a point at the minimum usable range (~0.2 m).
  const double max_disparity = rig_->focalLength() * rig_->baseline() / 0.2;
  for (std::size_t i = 0; i < features->size(); ++i) {
    if (!status[i] || !back_status[i] || !inside(right_pts[i], right.size())) continue;
    TrackedFeature& f = (*features)[i];
    const double row_error = std::abs(right_rect[i].y - f.left_rect.y);
    const double disparity = f.left_rect.x - right_rect[i].x;
    const bool ok = cv::norm(back_pts[i] - left_pts[i]) <= config_.klt_fb_threshold_px &&
                    row_error <= config_.stereo_epipolar_threshold_px && disparity > 0.0 &&
                    disparity < max_disparity;
    if (!ok) {
      ++stats->stereo_rejected;
      continue;
    }
    f.has_right = true;
    f.right_raw = right_pts[i];
    f.right_rect = right_rect[i];
    ++stats->stereo_matched;
  }
}

cv::Mat FeatureTracker::drawDebug(const cv::Mat& left, const FeatureFrame& frame) {
  cv::Mat canvas;
  cv::cvtColor(left, canvas, cv::COLOR_GRAY2BGR);
  for (const auto& f : frame.features) {
    // Red (new) → green (tracked for ≥ 20 frames).
    const double a = std::min(1.0, (f.track_length - 1) / 20.0);
    const cv::Scalar colour(0, 255 * a, 255 * (1.0 - a));
    if (f.track_length > 1) cv::line(canvas, f.prev_left_raw, f.left_raw, colour, 1, cv::LINE_AA);
    cv::circle(canvas, f.left_raw, 2, colour, cv::FILLED, cv::LINE_AA);
    if (f.has_right) cv::circle(canvas, f.left_raw, 5, cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
  }
  const auto& s = frame.stats;
  const std::string text = "features " + std::to_string(frame.features.size()) + "  stereo " +
                           std::to_string(s.stereo_matched) + "  new " +
                           std::to_string(s.detected) + "  " +
                           std::to_string(static_cast<int>(s.ms_total)) + " ms";
  cv::putText(canvas, text, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  return canvas;
}

}  // namespace kestrel
