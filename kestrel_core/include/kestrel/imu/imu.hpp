// SPDX-License-Identifier: Apache-2.0
//
// kestrel/imu/imu.hpp — IMU samples, noise parameters and preintegration (via GTSAM).
//
// TUTORIAL NOTE
// -------------
// Theory: docs/theory/03_imu_and_preintegration.md.  Reference: [Forster17].
//
// We do NOT implement IMU preintegration ourselves. GTSAM's PreintegratedCombinedMeasurements is
// the reference implementation of Forster et al.'s on-manifold preintegration, written by the
// paper's authors and used by many production systems. Kestrel adds only two thin layers:
//
//   1. makeImuParams()  — translates our YAML noise model into GTSAM's parameter object. The
//      units matter: both Kalibr (where our numbers come from) and GTSAM use *continuous-time*
//      noise densities (σ_a in m/s²/√Hz, …); GTSAM squares them into covariances and handles the
//      discretization with the actual Δt of every sample. Getting this conversion wrong silently
//      makes the estimator over- or under-confident, so it is covered by a Monte-Carlo test.
//
//   2. integrateImu()   — cuts the IMU stream at camera timestamps. IMU samples (200 Hz) almost
//      never coincide with image times (30 Hz), so the first and last segment of every interval
//      must be *interpolated* at the frame time; otherwise each frame-to-frame interval silently
//      loses or double-counts up to one IMU period (5 ms) of motion. This bookkeeping is the
//      part libraries leave to the application, and it is a classic source of subtle VIO bugs —
//      hence it lives here, with tests. It also feeds GTSAM midpoint readings with the
//      accelerometer pre-rotated by half a step, which turns GTSAM's start-of-step (first-order)
//      update into a second-order scheme; see the comment in src/imu.cpp and the convergence test.

#pragma once

#include <gtsam/navigation/CombinedImuFactor.h>

#include <Eigen/Core>
#include <vector>

#include "kestrel/config.hpp"

namespace kestrel {

/// One IMU reading, expressed in the IMU (= body) frame.
struct ImuSample {
  /// Timestamp [s].
  double t = 0.0;
  /// Specific force [m/s²], i.e. a − g: at rest the accelerometer reads +9.81 pointing "up".
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  /// Angular velocity [rad/s].
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
};

/// Shared pointer type GTSAM uses for preintegration parameters (boost:: in GTSAM 4.2, std:: in
/// 4.3+); deduced so the code builds against either.
using ImuParamsPtr = decltype(gtsam::PreintegrationCombinedParams::MakeSharedU(9.81));

/// Build GTSAM preintegration parameters from our configuration.
///
/// World frame is z-up, so gravity is (0, 0, −g) ("MakeSharedU" in GTSAM's naming). The body
/// frame is the IMU frame itself, so no body_P_sensor transform is set.
ImuParamsPtr makeImuParams(const ImuConfig& config);

/// Linear interpolation of an IMU reading at time t (a.t <= t <= b.t).
ImuSample interpolate(const ImuSample& a, const ImuSample& b, double t);

/// Integrates all IMU motion in [t0, t1] into `pim`.
///
/// `samples` must be sorted by time and cover the interval (front().t <= t0 and back().t >= t1).
/// The interval is split at every sample time; each piece is integrated with the *average* of
/// the (interpolated) readings at its two ends — the midpoint rule, second-order accurate —
/// because GTSAM's integrateMeasurement() holds a reading constant over the given Δt.
///
/// Returns the number of pieces integrated, or −1 if the samples do not cover [t0, t1].
int integrateImu(const std::vector<ImuSample>& samples, double t0, double t1,
                 gtsam::PreintegratedCombinedMeasurements* pim);

}  // namespace kestrel
