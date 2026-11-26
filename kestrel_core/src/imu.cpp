// SPDX-License-Identifier: Apache-2.0
//
// IMU parameter conversion and interval integration. See kestrel/imu/imu.hpp for the rationale.

#include "kestrel/imu/imu.hpp"

#include <algorithm>

namespace kestrel {

ImuParamsPtr makeImuParams(const ImuConfig& config) {
  // z-up navigation frame: gravity points along −z.
  auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(config.gravity_magnitude);
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  // Continuous-time white-noise densities → covariance "per Hz" (σ²). GTSAM divides by Δt
  // internally when it discretizes each sample, exactly as the theory chapter derives.
  params->setAccelerometerCovariance(I * config.accel_noise_density * config.accel_noise_density);
  params->setGyroscopeCovariance(I * config.gyro_noise_density * config.gyro_noise_density);

  // Bias random walks (the biases drift as Brownian motion).
  params->setBiasAccCovariance(I * config.accel_random_walk * config.accel_random_walk);
  params->setBiasOmegaCovariance(I * config.gyro_random_walk * config.gyro_random_walk);

  // Numerical integration error of position from velocity. GTSAM's documentation recommends a
  // tiny value; it mostly keeps the covariance well conditioned.
  params->setIntegrationCovariance(I * 1e-8);

  // Uncertainty of the bias used as the linearization point when integration starts. Keeping it
  // small (but non-zero) is the common choice; the bias estimate itself lives in the graph.
  params->setBiasAccOmegaInit(Eigen::Matrix<double, 6, 6>::Identity() * 1e-5);
  return params;
}

ImuSample interpolate(const ImuSample& a, const ImuSample& b, double t) {
  const double span = b.t - a.t;
  const double alpha = span > 0.0 ? std::clamp((t - a.t) / span, 0.0, 1.0) : 0.0;
  ImuSample s;
  s.t = t;
  s.acc = (1.0 - alpha) * a.acc + alpha * b.acc;
  s.gyr = (1.0 - alpha) * a.gyr + alpha * b.gyr;
  return s;
}

int integrateImu(const std::vector<ImuSample>& samples, double t0, double t1,
                 gtsam::PreintegratedCombinedMeasurements* pim) {
  if (samples.size() < 2 || t1 <= t0 || samples.front().t > t0 || samples.back().t < t1) {
    return -1;
  }

  // Index of the last sample at or before t0.
  auto upper = std::upper_bound(samples.begin(), samples.end(), t0,
                                [](double t, const ImuSample& s) { return t < s.t; });
  std::size_t i = static_cast<std::size_t>(std::distance(samples.begin(), upper)) - 1;

  // Walk the interval piece by piece: [t0, s_{i+1}.t], [s_{i+1}.t, s_{i+2}.t], …, [s_k.t, t1].
  ImuSample start = interpolate(samples[i], samples[i + 1], t0);
  int pieces = 0;
  while (start.t < t1) {
    // Advance to the sample that ends this piece (skip duplicates / zero-length pieces).
    while (i + 1 < samples.size() && samples[i + 1].t <= start.t) ++i;
    if (i + 1 >= samples.size()) break;
    const ImuSample end =
        samples[i + 1].t <= t1 ? samples[i + 1] : interpolate(samples[i], samples[i + 1], t1);
    const double dt = end.t - start.t;
    if (dt > 0.0) {
      const Eigen::Vector3d gyr_mid = 0.5 * (start.gyr + end.gyr);
      const Eigen::Vector3d acc_mid = 0.5 * (start.acc + end.acc);
      // GTSAM rotates the (constant) acceleration into the preintegration frame with the
      // orientation at the START of the step. The midpoint rule needs the orientation at the
      // MIDDLE of the step, R(t + dt/2) = R(t)·Exp(ω dt/2), so we pre-rotate the reading by
      // that half step. Without this the integration is only first order: with gravity in the
      // specific force and the body rotating at ~1 rad/s, 200 Hz drifts ~6 mm in position and
      // ~1.2 cm/s in velocity within 1 s (measured; see test_imu, AccuracyImprovesQuadratically).
      const Eigen::Vector3d acc_rotated = gtsam::Rot3::Expmap(0.5 * dt * gyr_mid) * acc_mid;
      pim->integrateMeasurement(acc_rotated, gyr_mid, dt);
      ++pieces;
    }
    start = end;
  }
  return pieces;
}

}  // namespace kestrel
