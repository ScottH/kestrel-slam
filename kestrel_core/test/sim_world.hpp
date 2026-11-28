// SPDX-License-Identifier: Apache-2.0
//
// sim_world.hpp — a synthetic world for testing the estimator without images.
//
// Motion: GTSAM's ConstantTwistScenario (exact kinematics) drives the body around a horizontal
// circle while the camera (= body; nominal config has T_imu_left = identity) looks radially
// OUTWARD at a cylindrical "wall" of landmarks. From that we generate:
//   * a 200 Hz IMU stream with constant biases and white noise of the configured densities,
//   * 30 Hz FeatureFrames: rectified stereo observations with pixel noise, persistent track ids
//     (a landmark gets a NEW id whenever it re-enters the view — just like the real front-end),
//     a fraction of mono-only observations, and optional gross outliers,
//   * optionally a MOVING OBJECT (M4): a rigid box of points whose features take
//     `dynamic_fraction` of the feature budget; isDynamic(track id) tells them apart. Two modes:
//       - dynamic_respawn_s == 0: the box travels with the camera, 2 m ahead, oscillating
//         sideways, like a person walking next to the robot (fast in the world, except briefly
//         at the turning points);
//       - dynamic_respawn_s > 0: every dynamic_respawn_s a new box (new track ids, a new person)
//         appears 2 m in front of the camera and drifts slowly IN THE WORLD at
//         dynamic_world_speed_mps — slow enough to pass every per-keyframe gate.
// Ground truth is available at any time via truth(t).

#pragma once

#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/navigation/Scenario.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <random>
#include <unordered_set>
#include <vector>

#include "kestrel/config.hpp"
#include "kestrel/frontend/feature_tracker.hpp"
#include "kestrel/imu/imu.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace kestrel::sim {

struct SimOptions {
  double duration_s = 20.0;
  double imu_rate_hz = 200.0;
  double camera_rate_hz = 30.0;
  double radius_m = 3.0;  ///< radius of the circular path (0 speed → static)
  double speed_mps = 1.0;
  double wall_distance_m = 3.0;  ///< landmark wall distance beyond the path
  int landmarks = 4000;
  int max_features = 150;
  double pixel_sigma = 0.5;
  double mono_fraction = 0.1;            ///< observations without a right-image match
  double outlier_fraction = 0.0;         ///< observations replaced by random pixels
  double dynamic_fraction = 0.0;         ///< share of max_features on the moving object
  double dynamic_distance_m = 2.0;       ///< object distance in front of the camera
  double dynamic_amplitude_m = 0.6;      ///< sideways oscillation amplitude (camera x)
  double dynamic_period_s = 4.0;         ///< ... and period
  double dynamic_respawn_s = 0.0;        ///< > 0: world-drifting boxes, a new one this often
  double dynamic_world_speed_mps = 0.1;  ///< drift speed of those boxes (sideways)
  gtsam::Rot3 extra_tilt;                ///< extra initial body rotation (static-init tests)
  gtsam::Vector3 accel_bias{0.03, -0.02, 0.05};
  gtsam::Vector3 gyro_bias{0.002, -0.003, 0.001};
  unsigned seed = 1;
};

class SimWorld {
 public:
  SimWorld(const Config& config, const SimOptions& opt) : opt_(opt), rig_(config.camera) {
    // Body axes in the world at t = 0: camera forward (z) looks along −x (outward), y is down.
    const gtsam::Rot3 R0 =
        gtsam::Rot3(gtsam::Point3(0, 1, 0), gtsam::Point3(0, 0, -1), gtsam::Point3(-1, 0, 0)) *
        opt.extra_tilt;
    const double omega = opt.radius_m > 0 ? opt.speed_mps / opt.radius_m : 0.0;
    scenario_ = std::make_unique<gtsam::ConstantTwistScenario>(
        gtsam::Vector3(0, -omega, 0), gtsam::Vector3(-opt.speed_mps, 0, 0),
        gtsam::Pose3(R0, gtsam::Point3(0, 0, 1.0)));

    // Circle centre = average position over one revolution (or the start, if static).
    gtsam::Point3 centre = scenario_->pose(0).translation();
    if (omega > 0) {
      centre = gtsam::Point3::Zero();
      const int n = 360;
      for (int i = 0; i < n; ++i) centre += scenario_->pose(2 * M_PI / omega * i / n).translation();
      centre /= n;
    }
    std::mt19937 rng(opt.seed);
    std::uniform_real_distribution<double> angle(0, 2 * M_PI), height(-1.5, 1.5);
    const double wall_r = opt.radius_m + opt.wall_distance_m;
    for (int i = 0; i < opt.landmarks; ++i) {
      const double a = angle(rng);
      landmarks_.emplace_back(centre.x() + wall_r * std::cos(a), centre.y() + wall_r * std::sin(a),
                              centre.z() + height(rng));
    }
    // The moving object: a 1.2 m × 1.0 m × 0.2 m box of points, in its own frame.
    std::uniform_real_distribution<double> ox(-0.6, 0.6), oy(-0.5, 0.5), oz(-0.1, 0.1);
    for (int i = 0; opt.dynamic_fraction > 0 && i < 400; ++i) {
      object_points_.emplace_back(ox(rng), oy(rng), oz(rng));
    }
    makeImu(config, rng);
    makeFrames(config, rng);
  }

  gtsam::NavState truth(double t) const { return scenario_->navState(t); }
  /// True if the track id belongs to a feature on the moving object.
  bool isDynamic(int track_id) const { return dynamic_ids_.count(track_id) > 0; }
  const std::unordered_set<int>& dynamicIds() const { return dynamic_ids_; }
  /// The frames with every moving-object feature removed (an "oracle" segmentation).
  std::vector<FeatureFrame> staticFrames() const {
    std::vector<FeatureFrame> out = frames_;
    for (auto& f : out) {
      f.features.erase(std::remove_if(f.features.begin(), f.features.end(),
                                      [&](const TrackedFeature& x) { return isDynamic(x.id); }),
                       f.features.end());
    }
    return out;
  }
  const std::vector<ImuSample>& imu() const { return imu_; }
  const std::vector<FeatureFrame>& frames() const { return frames_; }
  gtsam::imuBias::ConstantBias trueBias() const {
    return gtsam::imuBias::ConstantBias(opt_.accel_bias, opt_.gyro_bias);
  }

 private:
  void makeImu(const Config& config, std::mt19937& rng) {
    std::normal_distribution<double> n01(0.0, 1.0);
    const double dt = 1.0 / opt_.imu_rate_hz;
    const double sa = config.imu.accel_noise_density / std::sqrt(dt);
    const double sg = config.imu.gyro_noise_density / std::sqrt(dt);
    const gtsam::Vector3 g(0, 0, -config.imu.gravity_magnitude);
    // IMU starts a little before the first frame so the first interval is covered.
    for (double t = -1.0; t <= opt_.duration_s + 0.1; t += dt) {
      ImuSample s;
      s.t = t;
      s.gyr = scenario_->omega_b(t) + opt_.gyro_bias +
              sg * Eigen::Vector3d(n01(rng), n01(rng), n01(rng));
      s.acc = scenario_->acceleration_b(t) - scenario_->rotation(t).transpose() * g +
              opt_.accel_bias + sa * Eigen::Vector3d(n01(rng), n01(rng), n01(rng));
      imu_.push_back(s);
    }
  }

  void makeFrames(const Config& config, std::mt19937& rng) {
    std::normal_distribution<double> noise(0.0, opt_.pixel_sigma);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const auto& K = rig_.calibration();
    const gtsam::Pose3 T_b_c = rig_.bodyToRectifiedLeft();
    const double w = config.camera.width, h = config.camera.height;
    std::map<int, std::pair<int, int>> track;  // landmark → (track id, length) while visible
    int next_id = 0;
    const int n_dynamic = static_cast<int>(std::lround(opt_.dynamic_fraction * opt_.max_features));
    constexpr int kObjectKey = 1000000;  // object points use keys kObjectKey + i in `track`
    for (double t = 0.0; t <= opt_.duration_s + 1e-9; t += 1.0 / opt_.camera_rate_hz) {
      FeatureFrame frame;
      frame.t = t;
      const gtsam::StereoCamera camera(scenario_->pose(t) * T_b_c, K);
      std::map<int, std::pair<int, int>> now;
      // Object pose in the camera frame: straight ahead, sliding sideways.
      const gtsam::Pose3 T_c_o(
          gtsam::Rot3(),
          gtsam::Point3(opt_.dynamic_amplitude_m * std::sin(2 * M_PI * t / opt_.dynamic_period_s),
                        0.0, opt_.dynamic_distance_m));
      std::vector<gtsam::Point3> points;  // object points first (up to n_dynamic), then the wall
      std::vector<int> keys;
      gtsam::Pose3 T_w_o = camera.pose() * T_c_o;
      int spawn = 0;
      if (opt_.dynamic_respawn_s > 0) {
        spawn = static_cast<int>(std::floor(t / opt_.dynamic_respawn_s));
        const double t_s = spawn * opt_.dynamic_respawn_s;
        const gtsam::Pose3 T_w_c0 = scenario_->pose(t_s) * T_b_c;
        T_w_o = T_w_c0 *
                gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(opt_.dynamic_world_speed_mps * (t - t_s),
                                                          0, opt_.dynamic_distance_m));
      }
      for (int i = 0; i < static_cast<int>(object_points_.size()); ++i) {
        points.push_back(T_w_o.transformFrom(object_points_[i]));
        keys.push_back(kObjectKey + 1000 * spawn + i);
      }
      for (int i = 0; i < static_cast<int>(landmarks_.size()); ++i) {
        points.push_back(landmarks_[i]);
        keys.push_back(i);
      }
      int dynamic_used = 0;
      for (std::size_t j = 0;
           j < points.size() && static_cast<int>(frame.features.size()) < opt_.max_features; ++j) {
        const int i = keys[j];
        const bool on_object = i >= kObjectKey;
        if (on_object && dynamic_used >= n_dynamic) continue;
        const gtsam::Point3& point = points[j];
        const gtsam::Point3 p_c = camera.pose().transformTo(point);
        if (p_c.z() < 0.5 || p_c.z() > 15.0) continue;
        const gtsam::StereoPoint2 z = camera.project(point);
        auto in = [&](double u, double v) { return u > 10 && v > 10 && u < w - 10 && v < h - 10; };
        if (!in(z.uL(), z.v()) || !in(z.uR(), z.v())) continue;

        const auto prev = track.find(i);
        const std::pair<int, int> id_len =
            prev != track.end() ? std::make_pair(prev->second.first, prev->second.second + 1)
                                : std::make_pair(next_id++, 1);
        now[i] = id_len;
        if (on_object) {
          ++dynamic_used;
          dynamic_ids_.insert(id_len.first);
        }

        TrackedFeature f;
        f.id = id_len.first;
        f.track_length = id_len.second;
        double uL = z.uL() + noise(rng), uR = z.uR() + noise(rng), v = z.v() + noise(rng);
        if (u01(rng) < opt_.outlier_fraction) {  // gross mismatch somewhere in the image
          uL = 10 + u01(rng) * (w - 20);
          v = 10 + u01(rng) * (h - 20);
          uR = uL - (1 + u01(rng) * 40);
        }
        f.left_rect = f.left_raw = cv::Point2f(static_cast<float>(uL), static_cast<float>(v));
        f.has_right = u01(rng) >= opt_.mono_fraction;
        if (f.has_right) {
          f.right_rect = f.right_raw = cv::Point2f(static_cast<float>(uR), static_cast<float>(v));
        }
        frame.features.push_back(f);
      }
      track.swap(now);
      frames_.push_back(std::move(frame));
    }
  }

  SimOptions opt_;
  StereoRig rig_;
  std::unique_ptr<gtsam::ConstantTwistScenario> scenario_;
  std::vector<gtsam::Point3> landmarks_;
  std::vector<gtsam::Point3> object_points_;  ///< moving object, in its own frame
  std::unordered_set<int> dynamic_ids_;
  std::vector<ImuSample> imu_;
  std::vector<FeatureFrame> frames_;
};

}  // namespace kestrel::sim
