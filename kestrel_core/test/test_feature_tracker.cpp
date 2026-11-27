// SPDX-License-Identifier: Apache-2.0
//
// Tests for kestrel/frontend/feature_tracker.hpp on synthetic images with exactly known motion.
//
// The nominal Gemini configuration is an ideal (undistorted, already rectified) stereo pair, so
// image motions can be generated exactly:
//   * camera translation parallel to a textured plane → a pure image shift,
//   * stereo → the right image is the left one shifted by the disparity d = f·b/z,
//   * pure camera rotation R → the homography K·Rᵀ·K⁻¹.

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <random>

#include "kestrel/config.hpp"
#include "kestrel/frontend/feature_tracker.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace kestrel {
namespace {

/// Random blobby texture: plenty of well-conditioned corners, no repetitive pattern.
cv::Mat makeTexture(int width, int height, unsigned seed) {
  cv::Mat noise(height / 4, width / 4, CV_8UC1);
  cv::RNG rng(seed);
  rng.fill(noise, cv::RNG::UNIFORM, 0, 255);
  cv::Mat big;
  cv::resize(noise, big, cv::Size(width, height), 0, 0, cv::INTER_CUBIC);
  cv::GaussianBlur(big, big, cv::Size(0, 0), 1.2);
  return big;
}

/// dst(x, y) = src(x + dx, y + dy) — sub-pixel shift by bilinear interpolation.
cv::Mat shifted(const cv::Mat& src, double dx, double dy) {
  cv::Mat out;
  const cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, dx, 0, 1, dy);
  cv::warpAffine(src, out, M, src.size(), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,
                 cv::BORDER_REFLECT);
  return out;
}

class FeatureTrackerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.frontend.use_clahe = false;  // keep synthetic intensities exact
    rig_ = std::make_shared<StereoRig>(config_.camera);
    w_ = config_.camera.width;
    h_ = config_.camera.height;
    // Larger canvas so shifted crops never run out of texture.
    texture_ = makeTexture(w_ + 400, h_ + 400, 1);
  }

  cv::Mat view(double x, double y) const {  // crop of the texture at sub-pixel offset (x, y)
    return shifted(texture_, 200 + x, 200 + y)(cv::Rect(0, 0, w_, h_)).clone();
  }

  Config config_;
  std::shared_ptr<StereoRig> rig_;
  cv::Mat texture_;
  int w_ = 0, h_ = 0;
};

TEST_F(FeatureTrackerTest, DetectsUniformlySpacedFeatures) {
  FeatureTracker tracker(config_.frontend, rig_);
  const cv::Mat img = view(0, 0);
  const auto frame = tracker.track(0.0, img, img);
  const int n = static_cast<int>(frame.features.size());
  EXPECT_LE(n, config_.frontend.max_features);
  EXPECT_GE(n, static_cast<int>(0.8 * config_.frontend.max_features));
  double min_dist = 1e9;
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      min_dist = std::min(min_dist, static_cast<double>(cv::norm(frame.features[i].left_raw -
                                                                 frame.features[j].left_raw)));
    }
  }
  // goodFeaturesToTrack enforces the spacing; the sub-pixel refinement afterwards may move a
  // corner by a pixel or two.
  EXPECT_GE(min_dist, config_.frontend.min_feature_distance_px - 3.0);
}

TEST_F(FeatureTrackerTest, TracksSubPixelShiftAndMatchesStereo) {
  FeatureTracker tracker(config_.frontend, rig_);
  const double disparity = 12.0;  // plane at z = f·b/d ≈ 3.4 m
  const double dx = 7.3, dy = -4.6;

  const cv::Mat l0 = view(0, 0), r0 = view(disparity, 0);
  const auto f0 = tracker.track(0.0, l0, r0);
  // Camera moves so that the image content moves by (−dx, −dy).
  const cv::Mat l1 = view(dx, dy), r1 = view(dx + disparity, dy);
  const auto f1 = tracker.track(0.033, l1, r1);

  std::map<int, cv::Point2f> before;
  for (const auto& f : f0.features) before[f.id] = f.left_raw;
  int carried = 0;
  std::vector<double> flow_err, disp_err, row_err;
  for (const auto& f : f1.features) {
    if (f.track_length < 2) continue;
    ++carried;
    const cv::Point2f expected = before.at(f.id) - cv::Point2f(dx, dy);
    flow_err.push_back(cv::norm(f.left_raw - expected));
    if (f.has_right) {
      disp_err.push_back(std::abs(f.disparity() - disparity));
      row_err.push_back(std::abs(f.right_rect.y - f.left_rect.y));
    }
  }
  auto median = [](std::vector<double> v) {
    std::sort(v.begin(), v.end());
    return v.empty() ? 1e9 : v[v.size() / 2];
  };
  EXPECT_GE(carried, static_cast<int>(0.85 * f0.features.size()));
  EXPECT_LT(median(flow_err), 0.05) << "KLT is not sub-pixel accurate";
  EXPECT_GE(static_cast<int>(disp_err.size()), static_cast<int>(0.85 * f1.features.size()));
  EXPECT_LT(median(disp_err), 0.05);
  EXPECT_LT(median(row_err), 0.05);
}

TEST_F(FeatureTrackerTest, GyroPredictionSurvivesFastRotation) {
  // An 11.5° yaw between frames (≈ 340°/s at 30 Hz: a hard but realistic whip-pan) moves the
  // image by ~90 px — beyond KLT's unaided capture range
  // (≈ half-window · 2^levels ≈ 80 px for the default 21 px window and 3 pyramid levels).
  const auto& K = *rig_->calibration();
  const cv::Mat Kcv = (cv::Mat_<double>(3, 3) << K.fx(), 0, K.px(), 0, K.fy(), K.py(), 0, 0, 1);
  const gtsam::Rot3 R_prev_cur = gtsam::Rot3::Ry(0.2);  // camera y axis = down → yaw
  cv::Mat Rt = cv::Mat::zeros(3, 3, CV_64F);
  const gtsam::Matrix3 R = R_prev_cur.matrix().transpose();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) Rt.at<double>(r, c) = R(r, c);
  }
  const cv::Mat H = Kcv * Rt * Kcv.inv();  // x_cur = K Rᵀ K⁻¹ x_prev

  const cv::Mat l0 = view(0, 0);
  cv::Mat l1;
  cv::warpPerspective(l0, l1, H, l0.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

  int still_visible = 0;  // features of frame 0 that remain well inside frame 1
  std::string losses;     // where the gyro-aided run lost tracks (diagnostics)
  auto run = [&](bool with_gyro) {
    FeatureTracker tracker(config_.frontend, rig_);
    const auto f0 = tracker.track(0.0, l0, l0);
    const auto f1 =
        with_gyro ? tracker.track(0.033, l1, l1, R_prev_cur) : tracker.track(0.033, l1, l1);
    std::map<int, cv::Point2f> expected;
    still_visible = 0;
    for (const auto& f : f0.features) {
      std::vector<cv::Point2f> src{f.left_raw}, dst;
      cv::perspectiveTransform(src, dst, H);
      expected[f.id] = dst[0];
      still_visible += cv::Rect(20, 20, w_ - 40, h_ - 40).contains(cv::Point(dst[0]));
    }
    int correct = 0;
    for (const auto& f : f1.features) {
      // Perspective distortion of the turn costs KLT some accuracy: allow 1 px.
      if (f.track_length >= 2 && cv::norm(expected.at(f.id) - f.left_raw) < 1.0) ++correct;
    }
    if (with_gyro) {
      const auto& s = f1.stats;
      losses = "lost_klt " + std::to_string(s.lost_klt) + ", lost_fb " + std::to_string(s.lost_fb) +
               ", lost_ransac " + std::to_string(s.lost_ransac) + ", tracked " +
               std::to_string(s.tracked);
    }
    return correct;
  };
  const int with_gyro = run(true);
  const int without_gyro = run(false);
  ASSERT_GT(still_visible, 50);
  // Not 100 %: with the 335L's wide field of view (≈ ±45° horizontally) an 11.5° turn stretches
  // patches near the image edge by up to ~60 % (cos² ratio), beyond KLT's translation-only
  // model, so those tracks fail the forward–backward check. Affine KLT would recover some of them.
  EXPECT_GE(with_gyro, 0.65 * still_visible)
      << "with gyro " << with_gyro << ", without " << without_gyro << ", visible " << still_visible
      << " (" << losses << ")";
  EXPECT_LT(without_gyro, 0.5 * with_gyro)
      << "with gyro " << with_gyro << ", without " << without_gyro;
}

TEST_F(FeatureTrackerTest, RansacRejectsIndependentlyMovingPatch) {
  // Background moves horizontally; the content of a 240x240 patch ROTATES by 10° (an
  // independently moving object). Its tracks violate the background's epipolar geometry.
  //
  // Why rotation and not a simple shift of the patch? Our synthetic background is a plane, and
  // points on a plane are a DEGENERATE configuration for the fundamental matrix: F is then only
  // determined up to a family F = [e']× H, and for a patch that merely translates there is always
  // a member of that family which explains the patch too, so RANSAC cannot reject it. Real
  // scenes have depth variation, but nearly-planar views and pure rotations are common — one
  // reason the estimator (M4) adds IMU-predicted gating on top of this check.
  FeatureTracker tracker(config_.frontend, rig_);
  const cv::Mat l0 = view(0, 0);
  const auto f0 = tracker.track(0.0, l0, l0);

  cv::Mat l1 = view(6.0, 0.0);
  const cv::Rect patch(300, 120, 240, 240);
  const cv::Point2f centre(patch.x + patch.width / 2.0f, patch.y + patch.height / 2.0f);
  const cv::Mat Rp = cv::getRotationMatrix2D(centre, 10.0, 1.0);
  cv::Mat rotated;
  cv::warpAffine(l0, rotated, Rp, l0.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
  rotated(patch).copyTo(l1(patch));
  const auto f1 = tracker.track(0.033, l1, l1);

  // Count patch features that move clearly differently from the background (≥ 30 px from the
  // rotation centre, i.e. ≥ 5 px of rotational motion) and stay inside the patch.
  int patch_before = 0;
  std::map<int, bool> in_patch;
  for (const auto& f : f0.features) {
    const double r = cv::norm(f.left_raw - centre);
    const bool inside = r > 30.0 && r < patch.width / 2.0 - 20.0;
    in_patch[f.id] = inside;
    patch_before += inside;
  }
  int patch_survived = 0;
  for (const auto& f : f1.features) {
    if (f.track_length >= 2 && in_patch[f.id]) ++patch_survived;
  }
  ASSERT_GT(patch_before, 5);
  EXPECT_LE(patch_survived, patch_before / 4)
      << patch_survived << " of " << patch_before << " moving-patch tracks survived";
  EXPECT_GT(f1.stats.lost_ransac, 0);
}

TEST_F(FeatureTrackerTest, MaskBlocksFeatures) {
  FeatureTracker tracker(config_.frontend, rig_);
  cv::Mat mask(h_, w_, CV_8UC1, cv::Scalar(0));
  mask(cv::Rect(0, 0, w_ / 2, h_)).setTo(255);  // exclude the left half
  const cv::Mat img = view(0, 0);
  const auto frame = tracker.track(0.0, img, img, std::nullopt, mask);
  ASSERT_FALSE(frame.features.empty());
  for (const auto& f : frame.features) EXPECT_GE(f.left_raw.x, w_ / 2.0f - 1.0f);
}

TEST_F(FeatureTrackerTest, DroppedTracksAreNotResurrected) {
  FeatureTracker tracker(config_.frontend, rig_);
  const auto f0 = tracker.track(0.0, view(0, 0), view(0, 0));
  std::vector<int> drop;
  for (std::size_t i = 0; i < f0.features.size(); i += 3) drop.push_back(f0.features[i].id);
  tracker.dropTracks(drop);
  const auto f1 = tracker.track(0.033, view(2, 1), view(2, 1));
  EXPECT_EQ(f1.stats.dropped_by_backend, static_cast<int>(drop.size()));
  for (const auto& f : f1.features) {
    EXPECT_EQ(std::count(drop.begin(), drop.end(), f.id), 0) << "id " << f.id << " came back";
  }
}

TEST_F(FeatureTrackerTest, DebugDrawingHasImageSize) {
  FeatureTracker tracker(config_.frontend, rig_);
  const cv::Mat img = view(0, 0);
  const auto frame = tracker.track(0.0, img, img);
  const cv::Mat canvas = FeatureTracker::drawDebug(img, frame);
  EXPECT_EQ(canvas.size(), img.size());
  EXPECT_EQ(canvas.type(), CV_8UC3);
}

}  // namespace
}  // namespace kestrel
