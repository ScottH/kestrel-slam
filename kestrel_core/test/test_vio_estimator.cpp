// SPDX-License-Identifier: Apache-2.0
//
// End-to-end tests of the VIO estimator in a synthetic world (test/sim_world.hpp): exact ground
// truth, realistic IMU noise and biases, pixel noise, mono-only observations and gross outliers.
// Measured numbers are printed so regressions are visible in the test log, not just pass/fail.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>

#include "kestrel/config.hpp"
#include "kestrel/estimator/vio_estimator.hpp"
#include "sim_world.hpp"

namespace kestrel {
namespace {

struct RunResult {
  double rmse_position = 0, final_position_error = 0, final_yaw_error_deg = 0, path_length = 0;
  double accel_bias_error = 0, gyro_bias_error = 0;
  gtsam::Vector3 gyro_bias_error_axes = gtsam::Vector3::Zero();  ///< body x, y (= vertical), z
  double mean_gyro_bias_error_late = 0;  ///< averaged over the second half of the run
  int keyframes = 0, resets = 0, outliers_reported = 0;
  double mean_solve_ms = 0;
  gtsam::Pose3 final_pose;
};

RunResult runEstimator(const Config& config, const sim::SimWorld& world, bool init_from_truth,
                       std::unordered_set<int>* reported = nullptr) {
  auto rig = std::make_shared<StereoRig>(config.camera);
  VioEstimator estimator(config, rig);
  if (init_from_truth) {
    // True pose and velocity, but unknown (zero) biases: the estimator must find them.
    estimator.initialize(0.0, world.truth(0.0), gtsam::imuBias::ConstantBias());
  }
  RunResult r;
  double sq = 0, solve_ms = 0, late_sum = 0;
  int n = 0, late_n = 0;
  const double t_half = world.frames().back().t / 2;
  gtsam::Point3 last = world.truth(0).position();
  for (const auto& frame : world.frames()) {
    const auto out = estimator.process(frame, world.imu());
    if (!out.initialized) continue;
    r.keyframes += out.is_keyframe;
    r.resets += out.reset;
    r.outliers_reported += static_cast<int>(out.outlier_ids.size());
    if (reported) reported->insert(out.outlier_ids.begin(), out.outlier_ids.end());
    if (out.is_keyframe) solve_ms += out.solve_ms;
    const gtsam::NavState truth = world.truth(frame.t);
    const double e = (out.state.position() - truth.position()).norm();
    sq += e * e;
    ++n;
    r.path_length += (truth.position() - last).norm();
    last = truth.position();
    r.final_position_error = e;
    r.final_yaw_error_deg =
        std::abs(gtsam::Rot3::Logmap(truth.attitude().between(out.state.attitude())).z()) * 180 /
        M_PI;
    r.final_pose = out.state.pose();
    r.accel_bias_error = (out.bias.accelerometer() - world.trueBias().accelerometer()).norm();
    r.gyro_bias_error = (out.bias.gyroscope() - world.trueBias().gyroscope()).norm();
    r.gyro_bias_error_axes = out.bias.gyroscope() - world.trueBias().gyroscope();
    if (frame.t > t_half) {
      late_sum += r.gyro_bias_error;
      ++late_n;
    }
  }
  r.rmse_position = n ? std::sqrt(sq / n) : 1e9;
  r.mean_solve_ms = r.keyframes ? solve_ms / r.keyframes : 0;
  r.mean_gyro_bias_error_late = late_n ? late_sum / late_n : 1e9;
  return r;
}

void print(const char* name, const RunResult& r) {
  std::printf(
      "[%s] path %.1f m | RMSE %.3f m | final %.3f m (%.2f %% of path), yaw %.2f deg | "
      "bias err acc %.4f gyr %.5f | %d keyframes, %d resets, %d outliers | solve %.1f ms\n",
      name, r.path_length, r.rmse_position, r.final_position_error,
      100.0 * r.final_position_error / std::max(r.path_length, 1e-9), r.final_yaw_error_deg,
      r.accel_bias_error, r.gyro_bias_error, r.keyframes, r.resets, r.outliers_reported,
      r.mean_solve_ms);
  std::printf("[%s] mean gyro bias error over 2nd half: %.5f rad/s\n", name,
              r.mean_gyro_bias_error_late);
  std::printf("[%s] gyro bias error per body axis: x %+.5f  y %+.5f  z %+.5f rad/s\n", name,
              r.gyro_bias_error_axes.x(), r.gyro_bias_error_axes.y(), r.gyro_bias_error_axes.z());
}

TEST(VioEstimator, StaticStartAlignsGravityAndStaysPut) {
  Config config;
  sim::SimOptions opt;
  opt.duration_s = 3.0;
  opt.speed_mps = 0.0;                                     // not moving at all
  opt.extra_tilt = gtsam::Rot3::RzRyRx(0.06, -0.09, 0.0);  // ~3-5° roll/pitch
  const sim::SimWorld world(config, opt);

  auto rig = std::make_shared<StereoRig>(config.camera);
  VioEstimator estimator(config, rig);
  VioEstimator::Output out;
  gtsam::Point3 first_position;
  bool have_first = false;
  for (const auto& frame : world.frames()) {
    out = estimator.process(frame, world.imu());
    if (out.initialized && !have_first) {
      first_position = out.state.position();
      have_first = true;
    }
  }
  ASSERT_TRUE(out.initialized);
  // Gravity direction in the body frame must match the truth (yaw is arbitrary).
  const gtsam::Unit3 up_est = out.state.attitude().unrotate(gtsam::Unit3(0, 0, 1));
  const gtsam::Unit3 up_true = world.truth(out.t).attitude().unrotate(gtsam::Unit3(0, 0, 1));
  const double tilt_error_deg = std::acos(std::clamp(up_est.dot(up_true), -1.0, 1.0)) * 180 / M_PI;
  const double drift = (out.state.position() - first_position).norm();
  std::printf("[static] tilt error %.3f deg, drift %.4f m, gyro bias error %.5f rad/s\n",
              tilt_error_deg, drift, (out.bias.gyroscope() - world.trueBias().gyroscope()).norm());
  EXPECT_LT(tilt_error_deg, 0.5);
  EXPECT_LT(drift, 0.02);
}

TEST(VioEstimator, CircleTrajectoryAccuracy) {
  Config config;
  sim::SimOptions opt;  // 20 s around a 3 m circle at 1 m/s
  const sim::SimWorld world(config, opt);
  const RunResult r = runEstimator(config, world, true);
  print("circle", r);
  EXPECT_EQ(r.resets, 0);
  EXPECT_LT(r.final_position_error, 0.01 * r.path_length) << "drift above 1 % of path length";
  EXPECT_LT(r.final_yaw_error_deg, 1.0);
  // Starting from zero, the gyro bias error is |b_g| ≈ 0.0037 rad/s. The latest keyframe's
  // estimate fluctuates (a single snapshot is noisy), so judge the average over the second half.
  EXPECT_LT(r.mean_gyro_bias_error_late, 0.5 * world.trueBias().gyroscope().norm())
      << "gyro bias not estimated";
}

TEST(VioEstimator, IsDeterministic) {
  Config config;
  sim::SimOptions opt;
  opt.duration_s = 5.0;
  const sim::SimWorld world(config, opt);
  const RunResult a = runEstimator(config, world, true);
  const RunResult b = runEstimator(config, world, true);
  EXPECT_TRUE(a.final_pose.equals(b.final_pose, 1e-9));
}

TEST(VioEstimator, GrossOutliersAreReportedAndDoNotHurt) {
  Config config;
  sim::SimOptions clean_opt;
  clean_opt.duration_s = 10.0;
  sim::SimOptions dirty_opt = clean_opt;
  dirty_opt.outlier_fraction = 0.03;  // 3 % of all observations are random mismatches
  const RunResult clean = runEstimator(config, sim::SimWorld(config, clean_opt), true);
  std::unordered_set<int> reported;
  const RunResult dirty = runEstimator(config, sim::SimWorld(config, dirty_opt), true, &reported);
  print("clean", clean);
  print("3% outliers", dirty);
  EXPECT_EQ(dirty.resets, 0);
  EXPECT_GT(dirty.outliers_reported, 0);
  EXPECT_LT(dirty.final_position_error, 2.0 * clean.final_position_error + 0.02);
}

}  // namespace
}  // namespace kestrel
