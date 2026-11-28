// SPDX-License-Identifier: Apache-2.0
//
// Tests for dynamic-object handling (M4, theory ch. 07):
//   * the dynamic heat grid (clustered flags heat a cell, scattered ones do not, heat decays),
//   * the tracker's detection-only mask (blocks new corners, keeps existing tracks),
//   * the synthetic overlay used on real data (stereo-consistent with the intended depth),
//   * the estimator in a simulated world with a moving object supplying 30 % of the features:
//     accuracy must stay within 10 % of an oracle run in which those features were removed,
//     and the object's tracks must be neutralized (flagged, or never used).
// Measured numbers are printed, so changes are visible in the test log.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <unordered_set>

#include "kestrel/config.hpp"
#include "kestrel/estimator/vio_estimator.hpp"
#include "kestrel/eval/dynamic_overlay.hpp"
#include "kestrel/frontend/dynamic_heat_grid.hpp"
#include "kestrel/frontend/feature_tracker.hpp"
#include "sim_world.hpp"

namespace kestrel {
namespace {

TrackedFeature featureAt(int id, float x, float y) {
  TrackedFeature f;
  f.id = id;
  f.left_raw = f.left_rect = cv::Point2f(x, y);
  return f;
}

TEST(DynamicHeatGrid, ClusteredFlagsHeatACellScatteredOnesDoNot) {
  DynamicConfig config;        // 6 × 8 cells, decay 0.8, threshold 0.5
  const int w = 640, h = 480;  // cells of 80 × 80 px
  DynamicHeatGrid grid(config, w, h);

  // Cell (row 1, col 2) holds 6 features, all flagged at every keyframe (an object). Elsewhere
  // 40 features spread over the image, one of them flagged per keyframe (random mismatches).
  int keyframes_to_hot = -1;
  for (int k = 0; k < 15; ++k) {
    std::vector<TrackedFeature> features;
    std::unordered_set<int> flagged;
    for (int i = 0; i < 6; ++i) {
      features.push_back(featureAt(i, 170.0f + 10 * i, 90.0f + 5 * i));
      flagged.insert(i);
    }
    for (int i = 0; i < 40; ++i) {
      features.push_back(featureAt(100 + i, 20.0f + (i % 8) * 80, 300.0f + (i / 8) * 30));
    }
    flagged.insert(100 + k % 40);
    grid.update(features, flagged);
    if (keyframes_to_hot < 0 && grid.heat(1, 2) > config.heat_mask_threshold)
      keyframes_to_hot = k + 1;
  }
  std::printf(
      "[heat] object cell heat %.2f (hot after %d keyframes), hot cells %d, blocked %.1f %%\n",
      grid.heat(1, 2), keyframes_to_hot, grid.hotCells(), 100 * grid.blockedFraction());
  // Steady state = flagged / (features + 1) = 6/7; crossing 0.5 takes ln(1 − 0.5·7/6)/ln 0.8 ≈ 4.
  EXPECT_NEAR(grid.heat(1, 2), 6.0 / 7.0, 0.05);
  EXPECT_LE(keyframes_to_hot, 5);
  EXPECT_EQ(grid.hotCells(), 1) << "scattered single outliers must not heat their cells";
  const cv::Mat mask = grid.detectionMask();
  ASSERT_FALSE(mask.empty());
  EXPECT_EQ(mask.at<uint8_t>(100, 200), 255);  // inside cell (1, 2)
  EXPECT_EQ(mask.at<uint8_t>(400, 600), 0);
  EXPECT_NEAR(grid.blockedFraction(), 1.0 / 48, 1e-9);

  // The object leaves: no more flags. The cell must cool down within ~1 s (10 keyframes).
  for (int k = 0; k < 10; ++k) grid.update({}, {});
  EXPECT_EQ(grid.hotCells(), 0);
  EXPECT_TRUE(grid.detectionMask().empty());
}

cv::Mat makeTexture(int w, int h, unsigned seed) {
  cv::Mat img(h, w, CV_8UC1, cv::Scalar(128));
  cv::RNG rng(seed);
  for (int i = 0; i < 600; ++i) {
    const cv::Point c(rng.uniform(0, w), rng.uniform(0, h));
    const int s = rng.uniform(4, 30);
    cv::rectangle(img, c, c + cv::Point(s, rng.uniform(4, 30)), cv::Scalar(rng.uniform(0, 255)),
                  cv::FILLED);
  }
  cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
  return img;
}

TEST(FeatureTracker, DetectionMaskBlocksNewCornersButKeepsTracks) {
  const Config config;
  auto rig = std::make_shared<StereoRig>(config.camera);
  const int w = config.camera.width, h = config.camera.height;
  const cv::Mat image = makeTexture(w, h, 3);
  cv::Mat left_half(h, w, CV_8UC1, cv::Scalar(0));
  left_half(cv::Rect(0, 0, w / 2, h)).setTo(255);

  // Fresh tracker, left half closed for detection: every feature is in the right half.
  {
    FeatureTracker tracker(config.frontend, rig);
    const auto frame = tracker.track(0.0, image, image, std::nullopt, cv::Mat(), left_half);
    ASSERT_GT(frame.features.size(), 20u);
    for (const auto& f : frame.features) EXPECT_GE(f.left_raw.x, w / 2 - 1.0f);
  }
  // Tracks that exist before the left half closes survive it (unlike the external mask): the
  // second frame keeps exactly the left-half tracks a run without the mask keeps.
  auto secondFrame = [&](const cv::Mat& detection_mask) {
    FeatureTracker tracker(config.frontend, rig);
    tracker.track(0.0, image, image);
    return tracker.track(0.05, image, image, std::nullopt, cv::Mat(), detection_mask);
  };
  const auto control = secondFrame(cv::Mat());
  const auto blocked = secondFrame(left_half);
  int kept_control = 0, kept_blocked = 0, new_left = 0;
  for (const auto& f : control.features) kept_control += f.left_raw.x < w / 2 && f.track_length > 1;
  for (const auto& f : blocked.features) {
    kept_blocked += f.left_raw.x < w / 2 && f.track_length > 1;
    new_left += f.left_raw.x < w / 2 && f.track_length == 1;
  }
  EXPECT_GT(kept_control, 10);
  EXPECT_EQ(kept_blocked, kept_control);
  EXPECT_EQ(blocked.stats.lost_mask, 0);
  EXPECT_EQ(new_left, 0);
}

TEST(DynamicOverlay, IsStereoConsistentAtItsDepth) {
  const Config config;
  const StereoRig rig(config.camera);
  const int w = config.camera.width, h = config.camera.height;
  DynamicOverlayOptions options;
  const DynamicOverlay overlay(rig, w, h, options);

  cv::Mat left(h, w, CV_8UC1, cv::Scalar(0)), right = left.clone();
  EXPECT_TRUE(overlay.apply(1.0, &left, &right).empty()) << "not crossing before start_s";
  EXPECT_EQ(cv::countNonZero(left), 0);

  const double t = options.start_s + 0.5 * options.crossing_s;  // halfway across
  const cv::Mat mask = overlay.apply(t, &left, &right);
  ASSERT_FALSE(mask.empty());
  const double covered = cv::countNonZero(mask) / static_cast<double>(w * h);
  EXPECT_NEAR(covered, options.width_fraction * options.height_fraction, 0.1);

  // In rectified images, the right view must be the left view shifted by the disparity.
  const cv::Mat rl = rig.rectifyLeftImage(left), rr = rig.rectifyRightImage(right);
  double x0 = 0;
  ASSERT_TRUE(overlay.position(t, &x0));
  const int d = static_cast<int>(std::lround(overlay.disparityPx()));
  const int y = h / 2, u0 = static_cast<int>(x0) + 15, u1 = static_cast<int>(x0 + 0.4 * w) - 15;
  double shifted = 0, unshifted = 0;
  for (int u = u0; u < u1; ++u) {
    shifted += std::abs(rl.at<uint8_t>(y, u) - rr.at<uint8_t>(y, u - d));
    unshifted += std::abs(rl.at<uint8_t>(y, u) - rr.at<uint8_t>(y, u));
  }
  shifted /= (u1 - u0);
  unshifted /= (u1 - u0);
  std::printf(
      "[overlay] covers %.0f %%, disparity %.1f px, row diff shifted %.1f vs unshifted %.1f\n",
      100 * covered, overlay.disparityPx(), shifted, unshifted);
  EXPECT_LT(shifted, 0.3 * unshifted);
}

struct DynamicRun {
  double rmse = 0, final_error = 0;
  int resets = 0;
  std::unordered_set<int> used, flagged;  ///< dynamic track ids that became landmarks / flagged
  int removed_factors = 0;
};

DynamicRun run(const Config& config, const sim::SimWorld& world,
               const std::vector<FeatureFrame>& frames) {
  auto rig = std::make_shared<StereoRig>(config.camera);
  VioEstimator estimator(config, rig);
  estimator.initialize(0.0, world.truth(0.0), gtsam::imuBias::ConstantBias());
  DynamicRun r;
  double sq = 0;
  int n = 0;
  for (const auto& frame : frames) {
    const auto out = estimator.process(frame, world.imu());
    if (!out.initialized) continue;
    r.resets += out.reset;
    r.removed_factors += out.removed_factors;
    for (int id : out.new_landmark_ids) {
      if (world.isDynamic(id)) r.used.insert(id);
    }
    for (int id : out.outlier_ids) {
      if (world.isDynamic(id)) r.flagged.insert(id);
    }
    const double e = (out.state.position() - world.truth(frame.t).position()).norm();
    sq += e * e;
    ++n;
    r.final_error = e;
  }
  r.rmse = n ? std::sqrt(sq / n) : 1e9;
  return r;
}

struct MoverResult {
  DynamicRun oracle, with, without;
  double neutralized_fraction = 0, flagged_of_used = 0;
};

MoverResult runMovingObject(const char* name, const sim::SimOptions& opt) {
  Config on;
  Config off = on;
  off.dynamic.enable = false;
  const sim::SimWorld world(on, opt);
  EXPECT_GT(world.dynamicIds().size(), 10u);

  MoverResult res;
  res.oracle = run(on, world, world.staticFrames());
  res.with = run(on, world, world.frames());
  res.without = run(off, world, world.frames());
  const DynamicRun& oracle = res.oracle;
  const DynamicRun& with = res.with;
  const DynamicRun& without = res.without;

  int neutralized = 0;
  for (int id : world.dynamicIds()) neutralized += !with.used.count(id) || with.flagged.count(id);
  int used_and_flagged = 0;
  for (int id : with.used) used_and_flagged += with.flagged.count(id) > 0;
  const double neutralized_fraction =
      static_cast<double>(neutralized) / static_cast<double>(world.dynamicIds().size());
  const double flagged_of_used =
      with.used.empty() ? 1.0 : static_cast<double>(used_and_flagged) / with.used.size();

  std::printf("[%s] RMSE oracle %.4f | M4 on %.4f (%.2f×) | off %.4f (%.2f×) m; resets %d/%d\n",
              name, oracle.rmse, with.rmse, with.rmse / oracle.rmse, without.rmse,
              without.rmse / oracle.rmse, with.resets, without.resets);
  std::printf(
      "[%s] %zu dynamic tracks: %.1f %% neutralized; %zu became landmarks, %.1f %% of "
      "those flagged; %d factors removed\n",
      name, world.dynamicIds().size(), 100 * neutralized_fraction, with.used.size(),
      100 * flagged_of_used, with.removed_factors);

  res.neutralized_fraction = neutralized_fraction;
  res.flagged_of_used = flagged_of_used;
  return res;
}

/// Roadmap M4 criteria: no resets, ATE within 10 % of the oracle (+1 mm for noise on very small
/// errors), ≥ 90 % of the object's tracks neutralized and ≥ 90 % of those used then flagged.
void expectHandled(const MoverResult& r) {
  EXPECT_EQ(r.with.resets, 0);
  EXPECT_LT(r.with.rmse, 1.1 * r.oracle.rmse + 0.001);
  EXPECT_GE(r.neutralized_fraction, 0.9);
  EXPECT_GE(r.flagged_of_used, 0.9);
}

TEST(DynamicObjects, FastMoverWithThirtyPercentOfFeatures) {
  sim::SimOptions opt;
  opt.duration_s = 15.0;
  opt.dynamic_fraction = 0.3;  // 45 of 150 features on the object
  // defaults: 0.6 m sideways at 2 m distance, 4 s period (up to ~0.9 m/s relative). At the
  // turning points it is almost still in the world: without the confirmation delay its points
  // entered the graph there and the estimator followed it (6 m RMSE with M4 off).
  expectHandled(runMovingObject("fast mover", opt));
}

TEST(DynamicObjects, WalkingSpeedDriftWithThirtyPercentOfFeatures) {
  sim::SimOptions opt;
  opt.duration_s = 15.0;
  opt.dynamic_fraction = 0.3;
  // A new box ("person") every 3 s, 2 m ahead, walking sideways at 0.5 m/s in the world.
  opt.dynamic_respawn_s = 3.0;
  opt.dynamic_world_speed_mps = 0.5;
  expectHandled(runMovingObject("walking speed", opt));
}

TEST(DynamicObjects, SlowCoherentMoverIsAKnownLimitation) {
  // A KNOWN LIMIT of geometry + IMU (theory ch. 07 §3.2): the same boxes drifting at 0.1 m/s.
  // 1 cm per keyframe passes every gate; over the window the estimate partly follows the box
  // (velocity and accelerometer bias absorb the mismatch), so neither the post-solve residuals
  // nor the pre-solve, IMU-predicted ones separate it from the static points. Measured: RMSE
  // 3.8× the oracle at 0.05 m/s, 7× at 0.1, 16× at 0.2, 27× at 0.3; handled from 0.5 m/s.
  // The remedy is a semantic mask (run_vio --masks). This test records the numbers and checks
  // only that M4 does not make it worse; tighten it when the limit is lifted.
  sim::SimOptions opt;
  opt.duration_s = 15.0;
  opt.dynamic_fraction = 0.3;
  opt.dynamic_respawn_s = 3.0;
  opt.dynamic_world_speed_mps = 0.1;
  const MoverResult r = runMovingObject("slow mover (limitation)", opt);
  EXPECT_EQ(r.with.resets, 0);
  EXPECT_LT(r.with.rmse, 1.1 * r.without.rmse);
}

}  // namespace
}  // namespace kestrel
