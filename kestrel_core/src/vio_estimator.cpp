// SPDX-License-Identifier: Apache-2.0
//
// Sliding-window VIO on GTSAM's fixed-lag smoother. See kestrel/estimator/vio_estimator.hpp.

#include "kestrel/estimator/vio_estimator.hpp"

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include <chrono>

#include "kestrel/common/logging.hpp"

namespace kestrel {
namespace {

using gtsam::symbol_shorthand::B;  // IMU bias of keyframe k
using gtsam::symbol_shorthand::L;  // landmark with front-end track id
using gtsam::symbol_shorthand::V;  // velocity of keyframe k
using gtsam::symbol_shorthand::X;  // pose of keyframe k (world ← body)

using StereoFactor = gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>;
using MonoFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>;

}  // namespace

VioEstimator::VioEstimator(const Config& config, std::shared_ptr<const StereoRig> rig)
    : config_(config), rig_(std::move(rig)), imu_params_(makeImuParams(config.imu)) {
  const auto& K = *rig_->calibration();
  mono_calibration_ =
      gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(K.fx(), K.fy(), 0.0, K.px(), K.py()));

  // Pixel noise, wrapped in a Huber kernel: quadratic for small errors, linear beyond k·σ, so
  // a mismatched feature contributes a bounded pull instead of a squared one.
  const double sigma = config_.estimator.observation_sigma_px;
  const auto huber =
      gtsam::noiseModel::mEstimator::Huber::Create(config_.estimator.robust_kernel_k);
  stereo_noise_ =
      gtsam::noiseModel::Robust::Create(huber, gtsam::noiseModel::Isotropic::Sigma(3, sigma));
  mono_noise_ =
      gtsam::noiseModel::Robust::Create(huber, gtsam::noiseModel::Isotropic::Sigma(2, sigma));
  landmark_prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 5.0);
  smoother_ = makeSmoother();
}

std::unique_ptr<gtsam::IncrementalFixedLagSmoother> VioEstimator::makeSmoother() const {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;   // relinearize variables that moved more than this
  params.relinearizeSkip = 1;           // check every update (keyframes are only ~10 Hz)
  params.findUnusedFactorSlots = true;  // reuse slots of marginalized factors (bounded memory)
  return std::make_unique<gtsam::IncrementalFixedLagSmoother>(config_.estimator.smoother_lag_s,
                                                              params);
}

void VioEstimator::reset() {
  landmark_prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 5.0);
  smoother_ = makeSmoother();
  pending_init_.reset();
  num_keyframes_ = 0;
  last_observations_.clear();
  candidates_.clear();
  rejected_.clear();
}

void VioEstimator::initialize(double t, const gtsam::NavState& state,
                              const gtsam::imuBias::ConstantBias& bias) {
  reset();
  pending_init_ = PendingInit{t, state, bias};
}

bool VioEstimator::initializeStatic(double t, const std::vector<ImuSample>& imu) {
  Eigen::Vector3d acc = Eigen::Vector3d::Zero(), gyr = Eigen::Vector3d::Zero();
  int n = 0;
  for (const auto& s : imu) {
    if (s.t < t - config_.estimator.init_static_window_s || s.t > t) continue;
    acc += s.acc;
    gyr += s.gyr;
    ++n;
  }
  if (n < 10) return false;
  acc /= n;
  gyr /= n;
  // Was the body really still? If it rotated, the gyro average measures the motion, not the
  // bias — using it would put a large, confidently wrong bias into the prior.
  double gyr_var = 0.0;
  for (const auto& s : imu) {
    if (s.t < t - config_.estimator.init_static_window_s || s.t > t) continue;
    gyr_var += (s.gyr - gyr).squaredNorm();
  }
  const double gyr_std = std::sqrt(gyr_var / n);
  // Per-sample white-noise spread of a still gyro: density · √rate, per axis (×√3 for the norm).
  const double noise_std =
      config_.imu.gyro_noise_density * std::sqrt(config_.imu.rate_hz) * std::sqrt(3.0);
  const bool still = gyr_std < 3.0 * noise_std + 0.01 && gyr.norm() < 0.05;  // biases ≲ 0.02
  if (!still) {
    KLOG_WARN("static init: body is moving (gyro mean " << gyr.norm() << " rad/s, std " << gyr_std
                                                        << "); gyro bias left at zero");
    gyr.setZero();
  }
  if (std::abs(acc.norm() - config_.imu.gravity_magnitude) > 0.5) {
    KLOG_WARN("static init: |a| = " << acc.norm()
                                    << " m/s² differs from gravity; is the body "
                                       "moving? Roll/pitch may be inaccurate.");
  }
  // At rest the accelerometer measures the reaction to gravity, which points UP in the world.
  // The rotation that takes this body-frame vector onto world +z gives roll and pitch; yaw is
  // unobservable from gravity and set to zero (minimal rotation).
  const Eigen::Quaterniond q_wb = Eigen::Quaterniond::FromTwoVectors(acc, Eigen::Vector3d::UnitZ());
  const gtsam::NavState state(gtsam::Pose3(gtsam::Rot3(q_wb), gtsam::Point3::Zero()),
                              gtsam::Vector3::Zero());
  // Averaging the gyro of a still body measures its bias directly.
  initialize(t, state, gtsam::imuBias::ConstantBias(gtsam::Vector3::Zero(), gyr));
  KLOG_INFO("static init: roll/pitch from gravity, gyro bias " << gyr.transpose());
  return true;
}

VioEstimator::Output VioEstimator::process(const FeatureFrame& frame,
                                           const std::vector<ImuSample>& imu) {
  Output out;
  out.t = frame.t;
  if (!initialized() && !initializeStatic(frame.t, imu)) return out;

  if (num_keyframes_ == 0) {
    createFirstKeyframe(frame, imu, &out);
    return out;
  }

  // Predict the current state from the last keyframe with the IMU.
  gtsam::PreintegratedCombinedMeasurements pim(imu_params_, kf_bias_);
  if (integrateImu(imu, kf_time_, frame.t, &pim) <= 0) {
    KLOG_WARN("VIO: no IMU data covering [" << kf_time_ << ", " << frame.t << "]; frame skipped");
    out.initialized = true;
    out.state = kf_state_;
    out.bias = kf_bias_;
    out.keyframe_index = num_keyframes_ - 1;
    return out;
  }
  const gtsam::NavState predicted = pim.predict(kf_state_, kf_bias_);

  // Keyframe policy: regularly in time, or early when the landmarks in view are running out.
  const auto& stamps = smoother_->timestamps();
  int tracked = 0;
  for (const auto& f : frame.features) tracked += stamps.count(L(f.id)) > 0;
  const bool keyframe = frame.t - kf_time_ >= config_.estimator.keyframe_interval_s - 1e-6 ||
                        tracked < config_.estimator.min_keyframe_tracks;

  if (!keyframe) {
    out.initialized = true;
    out.state = predicted;
    out.bias = kf_bias_;
    out.keyframe_index = num_keyframes_ - 1;
    return out;
  }
  addKeyframe(frame, pim, predicted, &out);
  return out;
}

void VioEstimator::createFirstKeyframe(const FeatureFrame& frame, const std::vector<ImuSample>& imu,
                                       Output* out) {
  const PendingInit init = *pending_init_;
  gtsam::NavState state = init.state;
  if (frame.t > init.t + 1e-9) {  // initialized slightly before this frame: propagate
    gtsam::PreintegratedCombinedMeasurements pim(imu_params_, init.bias);
    if (integrateImu(imu, init.t, frame.t, &pim) > 0) state = pim.predict(init.state, init.bias);
  }

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  gtsam::FixedLagSmoother::KeyTimestampMap stamps;

  // Priors fix the gauge (where the world frame is, and its yaw) and encode what the
  // initialization knows: roll/pitch to ~0.5°, velocity and biases to the configured sigmas.
  const auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 0.01, 0.01, 0.01, 1e-3, 1e-3, 1e-3).finished());
  const auto vel_noise =
      gtsam::noiseModel::Isotropic::Sigma(3, config_.estimator.init_velocity_sigma);
  const auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << gtsam::Vector3::Constant(config_.estimator.init_accel_bias_sigma),
       gtsam::Vector3::Constant(config_.estimator.init_gyro_bias_sigma))
          .finished());
  graph.addPrior(X(0), state.pose(), pose_noise);
  graph.addPrior(V(0), state.velocity(), vel_noise);
  graph.addPrior(B(0), init.bias, bias_noise);
  values.insert(X(0), state.pose());
  values.insert(V(0), state.velocity());
  values.insert(B(0), init.bias);
  stamps[X(0)] = stamps[V(0)] = stamps[B(0)] = frame.t;

  last_observations_.clear();
  addVisualFactors(frame, 0, state.pose(), &graph, &values, &stamps, out);
  kf_time_ = frame.t;
  kf_state_ = state;
  kf_bias_ = init.bias;
  if (!solve(graph, values, stamps, out)) {
    reset();
    initialize(frame.t, state, init.bias);
    out->reset = true;
    return;
  }
  pending_init_.reset();
  num_keyframes_ = 1;
  readKeyframeState(0);
  collectOutliers(0, out);

  out->initialized = true;
  out->is_keyframe = true;
  out->keyframe_index = 0;
  out->state = kf_state_;
  out->bias = kf_bias_;
}

void VioEstimator::addKeyframe(const FeatureFrame& frame,
                               const gtsam::PreintegratedCombinedMeasurements& pim,
                               const gtsam::NavState& predicted, Output* out) {
  const int k = num_keyframes_;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  gtsam::FixedLagSmoother::KeyTimestampMap stamps;

  graph.emplace_shared<gtsam::CombinedImuFactor>(X(k - 1), V(k - 1), X(k), V(k), B(k - 1), B(k),
                                                 pim);
  values.insert(X(k), predicted.pose());
  values.insert(V(k), predicted.velocity());
  values.insert(B(k), kf_bias_);
  stamps[X(k)] = stamps[V(k)] = stamps[B(k)] = frame.t;

  last_observations_.clear();
  addVisualFactors(frame, k, predicted.pose(), &graph, &values, &stamps, out);

  if (!solve(graph, values, stamps, out)) {
    // Recovery: restart the window from the IMU prediction, keeping the bias estimate. The
    // trajectory continues in the same world frame with re-inflated uncertainty.
    const gtsam::imuBias::ConstantBias bias = kf_bias_;
    initialize(frame.t, predicted, bias);
    out->reset = true;
    out->initialized = true;
    out->state = predicted;
    out->bias = bias;
    return;
  }
  num_keyframes_ = k + 1;
  kf_time_ = frame.t;
  readKeyframeState(k);
  collectOutliers(k, out);

  out->initialized = true;
  out->is_keyframe = true;
  out->keyframe_index = k;
  out->state = kf_state_;
  out->bias = kf_bias_;
}

void VioEstimator::readKeyframeState(int k) {
  kf_state_ = gtsam::NavState(smoother_->calculateEstimate<gtsam::Pose3>(X(k)),
                              smoother_->calculateEstimate<gtsam::Vector3>(V(k)));
  kf_bias_ = smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(k));
}

void VioEstimator::addVisualFactors(const FeatureFrame& frame, int k, const gtsam::Pose3& T_w_b,
                                    gtsam::NonlinearFactorGraph* graph, gtsam::Values* values,
                                    gtsam::FixedLagSmoother::KeyTimestampMap* stamps, Output* out) {
  const auto& K = rig_->calibration();
  const gtsam::Pose3& T_b_c = rig_->bodyToRectifiedLeft();
  const gtsam::Pose3 T_w_c = T_w_b * T_b_c;
  const auto& alive = smoother_->timestamps();
  const auto& cfg = config_.estimator;
  const gtsam::StereoCamera stereo_camera(T_w_c, K);
  const gtsam::PinholeCamera<gtsam::Cal3_S2> mono_camera(T_w_c, *mono_calibration_);

  // Reprojection error of a world point against an observation, using the PREDICTED pose.
  auto predictedError = [&](const gtsam::Point3& p_w, const TrackedFeature& f) {
    try {
      if (f.has_right) return (stereo_camera.project(p_w) - f.stereo()).vector().norm();
      return (mono_camera.project(p_w) - gtsam::Point2(f.left_rect.x, f.left_rect.y)).norm();
    } catch (const std::exception&) {
      return std::numeric_limits<double>::infinity();  // behind the camera
    }
  };
  auto reject = [&](int id) {
    rejected_[id] = true;
    out->outlier_ids.push_back(id);
  };

  // Forget landmark candidates whose first keyframe has left the window.
  for (auto it = candidates_.begin(); it != candidates_.end();) {
    it = alive.count(X(it->second.keyframe)) ? std::next(it) : candidates_.erase(it);
  }

  for (const auto& f : frame.features) {
    if (rejected_.count(f.id)) continue;
    const gtsam::Key l = L(f.id);
    if (alive.count(l) > 0) {
      // Known landmark. Gate first: project its current estimate with the IMU-predicted pose.
      // A large error means the track jumped (mismatch, occlusion edge, moving object): adding
      // it would drag the solution, so the observation is dropped and the track rejected.
      if (predictedError(smoother_->calculateEstimate<gtsam::Point3>(l), f) >
          cfg.gate_reprojection_px) {
        reject(f.id);
        continue;
      }
      const gtsam::Point2 uv(f.left_rect.x, f.left_rect.y);
      if (f.has_right && f.disparity() > 0.0f) {
        graph->emplace_shared<StereoFactor>(f.stereo(), stereo_noise_, X(k), l, K, T_b_c);
        last_observations_.push_back({f.id, true, f.stereo(), uv});
      } else {
        graph->emplace_shared<MonoFactor>(uv, mono_noise_, X(k), l, mono_calibration_, T_b_c);
        last_observations_.push_back({f.id, false, gtsam::StereoPoint2(), uv});
      }
      (*stamps)[l] = frame.t;  // refresh: keeps the landmark in the window while it is seen
      ++out->visual_factors;
      continue;
    }
    // New landmark: needs a stereo depth inside the trusted range ...
    if (!f.has_right) continue;
    const auto p_c = rig_->triangulate(f.stereo());
    if (!p_c || p_c->z() < cfg.min_depth_m || p_c->z() > cfg.max_depth_m) continue;
    const gtsam::Point3 p_w = T_w_c.transformFrom(*p_c);

    // ... and CONFIRMATION by a second keyframe. A landmark with a single observation always
    // looks self-consistent, so one gross mismatch would otherwise enter the graph with a
    // random depth, fight the correct observations that follow, and can end up behind the
    // camera with no usable constraint (an indeterminate system). We therefore hold the first
    // stereo observation as a candidate and create the landmark only when a later keyframe sees
    // it where the candidate predicts. Random mismatches practically never confirm themselves.
    const auto cand = candidates_.find(f.id);
    if (cand == candidates_.end() || cand->second.keyframe == k ||
        predictedError(cand->second.p_w, f) > cfg.gate_reprojection_px) {
      candidates_[f.id] = Candidate{k, f.stereo(), p_w};  // (re)start the candidate
      continue;
    }
    values->insert(l, cand->second.p_w);
    // A very weak prior (σ = 5 m) keeps the problem well-posed in a corner case: when a point
    // ends up behind the camera, GTSAM's stereo factor reports zero Jacobians (cheirality), and a
    // landmark whose factors all do so would be unconstrained → IndeterminantLinearSystem. At
    // 5 m the prior has no measurable pull on a point stereo pins down to millimetres.
    graph->addPrior(l, cand->second.p_w, landmark_prior_noise_);
    graph->emplace_shared<StereoFactor>(cand->second.z, stereo_noise_, X(cand->second.keyframe), l,
                                        K, T_b_c);
    graph->emplace_shared<StereoFactor>(f.stereo(), stereo_noise_, X(k), l, K, T_b_c);
    (*stamps)[l] = frame.t;
    last_observations_.push_back(
        {f.id, true, f.stereo(), gtsam::Point2(f.left_rect.x, f.left_rect.y)});
    candidates_.erase(cand);
    out->visual_factors += 2;
    ++out->new_landmarks;
  }
}

bool VioEstimator::solve(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                         const gtsam::FixedLagSmoother::KeyTimestampMap& stamps, Output* out) {
  const auto t0 = std::chrono::steady_clock::now();
  try {
    smoother_->update(graph, values, stamps);
    smoother_->update();  // one extra iSAM2 iteration: noticeably better at negligible cost
  } catch (const std::exception& e) {
    KLOG_WARN("VIO: smoother update failed (" << e.what() << "); resetting");
    return false;
  }
  out->solve_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  int landmarks = 0;
  for (const auto& [key, stamp] : smoother_->timestamps()) {
    landmarks += gtsam::Symbol(key).chr() == 'l';
  }
  out->landmarks_in_window = landmarks;
  // Forget rejections of landmarks that have left the window.
  for (auto it = rejected_.begin(); it != rejected_.end();) {
    it = smoother_->timestamps().count(L(it->first)) ? std::next(it) : rejected_.erase(it);
  }
  return true;
}

void VioEstimator::collectOutliers(int /*k*/, Output* out) {
  // After the solve, re-project every landmark observed at this keyframe. Observations the
  // robust kernel had to down-weight heavily (error > outlier_reprojection_px) are reported to
  // the front-end, which stops tracking them, and the landmark receives no further factors.
  // Only the variables needed are read back: computing the full estimate of every landmark
  // in the window after each keyframe would dominate the run time.
  const gtsam::Pose3 T_w_c = kf_state_.pose() * rig_->bodyToRectifiedLeft();
  const auto& alive = smoother_->timestamps();
  const double threshold = config_.estimator.outlier_reprojection_px;
  const gtsam::StereoCamera stereo_camera(T_w_c, rig_->calibration());
  const gtsam::PinholeCamera<gtsam::Cal3_S2> mono_camera(T_w_c, *mono_calibration_);
  for (const auto& obs : last_observations_) {
    const int id = obs.id;
    const gtsam::Key l = L(id);
    if (!alive.count(l)) continue;
    const gtsam::Point3 p_w = smoother_->calculateEstimate<gtsam::Point3>(l);
    double error = 0.0;
    try {
      error = obs.stereo ? (stereo_camera.project(p_w) - obs.z).vector().norm()
                         : (mono_camera.project(p_w) - obs.uv).norm();
    } catch (const std::exception&) {
      error = std::numeric_limits<double>::infinity();  // behind the camera
    }
    if (error > threshold && !rejected_.count(id)) {
      rejected_[id] = true;
      out->outlier_ids.push_back(id);
    }
  }
}

}  // namespace kestrel
