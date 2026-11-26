// SPDX-License-Identifier: Apache-2.0
//
// Tests for kestrel/imu/imu.hpp.
//
// These do not re-test GTSAM's preintegration (GTSAM has its own extensive tests). They test the
// parts Kestrel is responsible for:
//   * the YAML → GTSAM parameter conversion (gravity direction, noise units),
//   * cutting the IMU stream at arbitrary frame times (interpolation at the boundaries),
//   * that, with our noise conversion, GTSAM's predicted covariance matches Monte-Carlo
//     simulation of a real noisy IMU — i.e. the filter is neither over- nor under-confident.
// Trajectories come from GTSAM's own Scenario classes (analytic motion with exact IMU readings).

#include <gtest/gtest.h>
#include <gtsam/navigation/Scenario.h>

#include <random>
#include <vector>

#include "kestrel/config.hpp"
#include "kestrel/imu/imu.hpp"

namespace kestrel {
namespace {

using gtsam::imuBias::ConstantBias;

/// Exact IMU readings from an analytic scenario: specific force f_b = a_b − R_nbᵀ g_n.
std::vector<ImuSample> sampleScenario(const gtsam::Scenario& scenario, const gtsam::Vector3& g_n,
                                      double t_end, double rate_hz, double t_offset = 0.0) {
  std::vector<ImuSample> samples;
  const double dt = 1.0 / rate_hz;
  for (double t = t_offset; t <= t_end + 1e-12; t += dt) {
    ImuSample s;
    s.t = t;
    s.gyr = scenario.omega_b(t);
    s.acc = scenario.acceleration_b(t) - scenario.rotation(t).transpose() * g_n;
    samples.push_back(s);
  }
  return samples;
}

/// A non-trivial motion: rotating about all axes while accelerating in the world frame, so the
/// body-frame specific force changes over time (constant-reading motions would hide errors).
gtsam::AcceleratingScenario makeScenario() {
  return gtsam::AcceleratingScenario(gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3), gtsam::Point3(1, 2, 3),
                                     gtsam::Vector3(0.5, -0.3, 0.2), gtsam::Vector3(0.4, 0.2, -0.1),
                                     gtsam::Vector3(0.3, -0.4, 0.8));
}

TEST(ImuParams, GravityAndNoiseUnits) {
  ImuConfig cfg;
  cfg.gravity_magnitude = 9.80665;
  cfg.accel_noise_density = 0.02;
  cfg.gyro_noise_density = 0.003;
  cfg.accel_random_walk = 0.001;
  cfg.gyro_random_walk = 2e-5;
  const auto params = makeImuParams(cfg);
  EXPECT_TRUE(params->n_gravity.isApprox(gtsam::Vector3(0, 0, -9.80665)));
  EXPECT_NEAR(params->accelerometerCovariance(0, 0), 0.02 * 0.02, 1e-15);
  EXPECT_NEAR(params->gyroscopeCovariance(2, 2), 0.003 * 0.003, 1e-15);
  EXPECT_NEAR(params->biasAccCovariance(1, 1), 0.001 * 0.001, 1e-15);
  EXPECT_NEAR(params->biasOmegaCovariance(0, 0), 2e-5 * 2e-5, 1e-18);
}

TEST(ImuIntegration, PredictsAnalyticTrajectory) {
  const auto scenario = makeScenario();
  const auto params = makeImuParams(ImuConfig{});
  const auto samples = sampleScenario(scenario, params->n_gravity, 2.0, 200.0);

  // Frame times deliberately *between* IMU samples (camera and IMU clocks never line up).
  const double t0 = 0.0123, t1 = 1.0337;
  gtsam::PreintegratedCombinedMeasurements pim(params, ConstantBias());
  ASSERT_GT(integrateImu(samples, t0, t1, &pim), 0);
  EXPECT_NEAR(pim.deltaTij(), t1 - t0, 1e-12);

  const gtsam::NavState predicted = pim.predict(scenario.navState(t0), ConstantBias());
  const gtsam::NavState truth = scenario.navState(t1);
  // Midpoint integration at 200 Hz over ~1 s of rotating, accelerating motion.
  EXPECT_LT((predicted.position() - truth.position()).norm(), 1e-4);
  EXPECT_LT((predicted.velocity() - truth.velocity()).norm(), 1e-4);
  EXPECT_LT(gtsam::Rot3::Logmap(predicted.attitude().between(truth.attitude())).norm(), 1e-5);
}

TEST(ImuIntegration, AccuracyImprovesQuadraticallyWithRate) {
  // The midpoint rule is second order: doubling the IMU rate should cut the error ~4×.
  const auto scenario = makeScenario();
  const auto params = makeImuParams(ImuConfig{});
  auto error_at = [&](double rate) {
    const auto samples = sampleScenario(scenario, params->n_gravity, 2.0, rate);
    gtsam::PreintegratedCombinedMeasurements pim(params, ConstantBias());
    integrateImu(samples, 0.0, 1.5, &pim);
    const auto predicted = pim.predict(scenario.navState(0.0), ConstantBias());
    return (predicted.position() - scenario.navState(1.5).position()).norm();
  };
  const double e100 = error_at(100.0), e200 = error_at(200.0);
  EXPECT_GT(e100 / e200, 3.0) << "e100=" << e100 << " e200=" << e200;
}

TEST(ImuIntegration, SplittingIntervalsIsConsistent) {
  // Integrating [t0, t1] in one go must agree with [t0, tm] followed by [tm, t1] — this is what
  // happens frame after frame in the estimator.
  const auto scenario = makeScenario();
  const auto params = makeImuParams(ImuConfig{});
  const auto samples = sampleScenario(scenario, params->n_gravity, 2.0, 200.0);
  const double t0 = 0.1, tm = 0.4567, t1 = 0.9;

  gtsam::PreintegratedCombinedMeasurements whole(params, ConstantBias());
  integrateImu(samples, t0, t1, &whole);
  gtsam::PreintegratedCombinedMeasurements first(params, ConstantBias());
  gtsam::PreintegratedCombinedMeasurements second(params, ConstantBias());
  integrateImu(samples, t0, tm, &first);
  integrateImu(samples, tm, t1, &second);

  const auto x0 = scenario.navState(t0);
  const auto direct = whole.predict(x0, ConstantBias());
  const auto chained = second.predict(first.predict(x0, ConstantBias()), ConstantBias());
  EXPECT_NEAR(first.deltaTij() + second.deltaTij(), whole.deltaTij(), 1e-12);
  EXPECT_LT((direct.position() - chained.position()).norm(), 1e-6);
  EXPECT_LT((direct.velocity() - chained.velocity()).norm(), 1e-6);
}

TEST(ImuIntegration, RejectsUncoveredIntervals) {
  std::vector<ImuSample> samples(3);
  samples[0].t = 1.0;
  samples[1].t = 1.005;
  samples[2].t = 1.010;
  gtsam::PreintegratedCombinedMeasurements pim(makeImuParams(ImuConfig{}), ConstantBias());
  EXPECT_EQ(integrateImu(samples, 0.9, 1.005, &pim), -1);   // starts before the data
  EXPECT_EQ(integrateImu(samples, 1.0, 1.2, &pim), -1);     // ends after the data
  EXPECT_EQ(integrateImu(samples, 1.002, 1.008, &pim), 2);  // two partial pieces
}

TEST(ImuIntegration, CovarianceMatchesMonteCarlo) {
  // Simulate a noisy IMU exactly as the continuous-time model says: white noise of density σ
  // sampled at rate 1/Δt has per-sample standard deviation σ/√Δt. If makeImuParams() hands GTSAM
  // the right units, GTSAM's predicted covariance of the preintegrated deltas equals the
  // empirical covariance of the deltas computed from many noisy runs.
  ImuConfig cfg;
  cfg.accel_noise_density = 0.05;
  cfg.gyro_noise_density = 0.005;
  const auto params = makeImuParams(cfg);
  // makeImuParams() also adds uncertainty about the bias *linearization point* (biasAccOmegaInit),
  // which inflates the predicted rotation covariance. This simulation has perfectly known biases,
  // so switch that term off here to test the white-noise units in isolation.
  params->setBiasAccOmegaInit(Eigen::Matrix<double, 6, 6>::Identity() * 1e-14);
  const auto scenario = makeScenario();
  const double rate = 200.0, dt = 1.0 / rate, duration = 0.5;
  const auto clean = sampleScenario(scenario, params->n_gravity, duration, rate);

  gtsam::PreintegratedCombinedMeasurements reference(params, ConstantBias());
  integrateImu(clean, 0.0, duration, &reference);
  const gtsam::NavState delta_true = reference.deltaXij();

  std::mt19937 rng(42);
  std::normal_distribution<double> normal(0.0, 1.0);
  const double sa = cfg.accel_noise_density / std::sqrt(dt);
  const double sg = cfg.gyro_noise_density / std::sqrt(dt);
  const int trials = 2000;
  Eigen::Matrix<double, 9, 9> empirical = Eigen::Matrix<double, 9, 9>::Zero();
  for (int k = 0; k < trials; ++k) {
    auto noisy = clean;
    for (auto& s : noisy) {
      s.acc += sa * Eigen::Vector3d(normal(rng), normal(rng), normal(rng));
      s.gyr += sg * Eigen::Vector3d(normal(rng), normal(rng), normal(rng));
    }
    gtsam::PreintegratedCombinedMeasurements pim(params, ConstantBias());
    integrateImu(noisy, 0.0, duration, &pim);
    // Error in GTSAM's tangent space of NavState: [rotation, position, velocity].
    const gtsam::Vector9 e = delta_true.localCoordinates(pim.deltaXij());
    empirical += e * e.transpose();
  }
  empirical /= trials;

  const Eigen::Matrix<double, 9, 9> predicted = reference.preintMeasCov().topLeftCorner<9, 9>();
  for (int i = 0; i < 9; ++i) {
    const double ratio = empirical(i, i) / predicted(i, i);
    // ratio < 1: GTSAM predicts more spread than reality (under-confident, too conservative);
    // ratio > 1: less spread than reality (over-confident — the dangerous direction).
    EXPECT_GT(ratio, 0.8) << "state " << i << " (GTSAM under-confident? noise units wrong?)";
    EXPECT_LT(ratio, 1.25) << "state " << i << " (GTSAM over-confident? noise units wrong?)";
  }
}

}  // namespace
}  // namespace kestrel
