// SPDX-License-Identifier: Apache-2.0
//
// kestrel/config.hpp — every tunable parameter of the system, in one typed structure.
//
// TUTORIAL NOTE
// -------------
// SLAM systems have a lot of knobs. Two failure modes are common:
//   1. parameters scattered as magic numbers through the code, and
//   2. untyped key/value lookups (`cfg["max_feat"]`) that silently fall back on typos.
// We avoid both: all parameters live in plain structs with *documented defaults*, the YAML loader
// fills them in, and `validate()` rejects inconsistent values up-front with a readable message.
//
// The same YAML file is used by the offline tools (run_euroc, ...) and by the ROS 2 node, so a
// configuration that was tuned on a dataset is exactly the one that runs on the robot.
//
// Conventions used throughout Kestrel:
//   * Frames:  W = world (gravity-aligned, z up), B = body = IMU frame, C = camera (optical:
//              x right, y down, z forward).  `T_a_b` maps points from frame b into frame a:
//              p_a = T_a_b * p_b.  So `T_imu_left` maps left-camera points into the IMU frame.
//   * Units:   SI everywhere (m, s, rad, m/s^2, rad/s); pixels for image quantities.

#pragma once

#include <Eigen/Core>
#include <stdexcept>
#include <string>
#include <vector>

namespace kestrel {

/// Thrown when a configuration file cannot be read or contains invalid values.
class ConfigError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

/// Intrinsics of a single camera.
struct CameraIntrinsics {
  /// Focal lengths and principal point in pixels: [fx, fy, cx, cy].
  double fx = 430.0, fy = 430.0, cx = 424.0, cy = 240.0;
  /// Distortion coefficients. For "pinhole-radtan": [k1, k2, p1, p2] (optionally k3).
  /// For "pinhole-equidistant" (fisheye): [k1, k2, k3, k4].
  std::vector<double> distortion{0.0, 0.0, 0.0, 0.0};
};

/// The stereo camera rig. Defaults are *nominal* Gemini 335L(g) values at 848x480 — replace them
/// with a real calibration (see docs/hardware/gemini_335lg.md) or let the ROS node read them from
/// the driver's camera_info / TF.
struct CameraConfig {
  std::string model = "pinhole-radtan";  ///< "pinhole-radtan" or "pinhole-equidistant"
  int width = 848;
  int height = 480;
  double rate_hz = 30.0;
  CameraIntrinsics left;
  CameraIntrinsics right;
  /// Camera → IMU extrinsics (4x4 homogeneous). Nominal: identity rotation, left camera at the IMU,
  /// right camera 95 mm to the right (the Gemini 335L stereo baseline).
  Eigen::Matrix4d T_imu_left = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_imu_right = [] {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = 0.095;
    return T;
  }();
  /// Time offset t_imu = t_cam + time_offset_s (the 335L syncs IMU and cameras in hardware, so ~0).
  double time_offset_s = 0.0;
};

/// Continuous-time IMU noise model (units as in Kalibr's imu.yaml).
struct ImuConfig {
  double accel_noise_density = 0.08;  ///< σ_a   [m/s^2/√Hz]
  double gyro_noise_density = 0.004;  ///< σ_g   [rad/s/√Hz]
  double accel_random_walk = 0.002;   ///< σ_ba  [m/s^3/√Hz]
  double gyro_random_walk = 4.0e-5;   ///< σ_bg  [rad/s^2/√Hz]
  double gravity_magnitude = 9.81;    ///< [m/s^2]
  double rate_hz = 200.0;
};

/// Visual front-end (feature detection and tracking).
struct FrontendConfig {
  int max_features = 200;            ///< target number of tracked features per frame
  int min_feature_distance_px = 25;  ///< minimum spacing between corners (uniform coverage)
  int klt_window_px = 21;            ///< Lucas–Kanade window size
  int klt_pyramid_levels = 3;
  double klt_fb_threshold_px = 0.5;  ///< forward–backward consistency threshold
  double ransac_threshold_px = 1.0;  ///< fundamental-matrix RANSAC threshold
  double stereo_epipolar_threshold_px = 2.0;
  bool use_clahe = true;              ///< contrast-limited histogram equalization (helps IR images)
  bool gyro_aided_prediction = true;  ///< predict feature motion from gyro rotation
};

/// Sliding-window visual-inertial estimator.
struct EstimatorConfig {
  int window_size = 10;                ///< number of keyframes in the optimization window
  double keyframe_parallax_px = 10.0;  ///< mean parallax that makes a frame a keyframe
  double observation_sigma_px = 1.5;   ///< reprojection noise std-dev
  double min_depth_m = 0.1;
  double max_depth_m = 40.0;
  int max_solver_iterations = 8;
  double max_solver_time_s = 0.04;   ///< hard real-time budget per optimization
  bool estimate_extrinsics = false;  ///< refine camera–IMU extrinsics online
};

/// Short-term dynamic-object handling.
struct DynamicConfig {
  bool enable = true;
  double outlier_reprojection_px = 3.0;  ///< mean reprojection error that flags a feature
  int grid_rows = 6;                     ///< resolution of the "dynamic heat" grid
  int grid_cols = 8;
  double heat_decay = 0.8;           ///< per-frame decay of the heat map
  double heat_mask_threshold = 0.5;  ///< cells above this block new features
};

/// Place recognition and loop closure.
struct LoopClosureConfig {
  bool enable = true;
  std::string vocabulary_path;  ///< empty → loop closure disabled with a warning
  int orb_features = 1000;
  double min_bow_score = 0.015;
  int min_inliers = 25;               ///< PnP-RANSAC inliers to accept a loop
  int exclude_recent_keyframes = 30;  ///< don't match against the immediate past
  int temporal_consistency = 2;       ///< consecutive consistent detections required
};

/// Lifelong map management and the dense voxel map.
struct MappingConfig {
  double voxel_size_m = 0.05;
  double max_integration_range_m = 5.0;
  double grid_min_height_m = 0.05;  ///< height band projected into the 2D occupancy grid
  double grid_max_height_m = 1.8;
  double persistence_expected_lifetime_s = 7.0 * 24 * 3600;  ///< survival prior of map points
  double persistence_miss_probability = 0.2;                 ///< P(not detected | still there)
  double persistence_false_alarm_probability = 0.05;         ///< P(detected | gone)
  double persistence_prune_threshold = 0.1;  ///< posterior below which points are removed
  std::string map_path;                      ///< map to load at start-up (empty → none)
};

/// The complete configuration.
struct Config {
  CameraConfig camera;
  ImuConfig imu;
  FrontendConfig frontend;
  EstimatorConfig estimator;
  DynamicConfig dynamic;
  LoopClosureConfig loop_closure;
  MappingConfig mapping;

  /// Returns a list of human-readable problems; empty if the configuration is usable.
  std::vector<std::string> validate() const;
};

/// Load a configuration from a YAML file. Keys that are absent keep their defaults; unknown keys
/// produce a warning (they are usually typos). Throws ConfigError on I/O, parse or validation
/// errors.
Config loadConfig(const std::string& yaml_path);

/// Same as loadConfig, but from a YAML string (handy for tests).
Config loadConfigFromString(const std::string& yaml_text);

/// Serialize a configuration back to YAML (used to record the exact settings of every run).
std::string toYaml(const Config& config);

}  // namespace kestrel
