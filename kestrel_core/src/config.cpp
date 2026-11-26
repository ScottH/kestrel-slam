// SPDX-License-Identifier: Apache-2.0
//
// YAML loading, validation and serialization of kestrel::Config.
//
// TUTORIAL NOTE — the "field visitor" pattern
// --------------------------------------------
// Loading and saving a config usually means writing every field name twice (once in the reader,
// once in the writer) — and the two lists inevitably drift apart. Here each section declares its
// fields exactly *once*, in a `visit()` function templated on a visitor:
//
//     template <class V> void visit(V& v, ImuConfig& c) {
//       v.field("accel_noise_density", c.accel_noise_density);
//       ...
//     }
//
// A `Reader` visitor pulls values out of a YAML node; a `Writer` visitor emits them. Adding a
// parameter is then a one-line change, and unknown keys in the file can be reported because the
// reader knows the complete list of valid names.

#include "kestrel/config.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // determinant() lives in Eigen's LU module
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

#include "kestrel/common/logging.hpp"

namespace kestrel {
namespace {

// Forward declarations of the field lists (defined below the visitors that call them).
template <class V>
void visit(V& v, CameraIntrinsics& c);
template <class V>
void visit(V& v, CameraConfig& c);
template <class V>
void visit(V& v, ImuConfig& c);
template <class V>
void visit(V& v, FrontendConfig& c);
template <class V>
void visit(V& v, EstimatorConfig& c);
template <class V>
void visit(V& v, DynamicConfig& c);
template <class V>
void visit(V& v, LoopClosureConfig& c);
template <class V>
void visit(V& v, MappingConfig& c);
template <class V>
void visit(V& v, Config& c);

// ---------------------------------------------------------------------------------------------
// Value conversion helpers
// ---------------------------------------------------------------------------------------------

// A 4x4 transform can be written either as 16 numbers (row-major) or as 4 rows of 4 numbers.
Eigen::Matrix4d parseMatrix4(const YAML::Node& node, const std::string& key) {
  std::vector<double> values;
  if (node.IsSequence() && node.size() == 4 && node[0].IsSequence()) {
    for (const auto& row : node) {
      for (const auto& v : row) values.push_back(v.as<double>());
    }
  } else if (node.IsSequence()) {
    values = node.as<std::vector<double>>();
  }
  if (values.size() != 16) {
    throw ConfigError("'" + key + "' must contain 16 numbers (a row-major 4x4 matrix)");
  }
  Eigen::Matrix4d T;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) T(r, c) = values[4 * r + c];
  }
  return T;
}

// ---------------------------------------------------------------------------------------------
// Visitors
// ---------------------------------------------------------------------------------------------

/// Reads fields from a YAML map, remembering which keys were consumed.
class Reader {
 public:
  Reader(const YAML::Node& node, std::string path) : node_(node), path_(std::move(path)) {}

  template <typename T>
  void field(const char* key, T& value) {
    known_.insert(key);
    const YAML::Node child = node_[key];
    if (!child) return;  // absent → keep the documented default
    try {
      value = child.as<T>();
    } catch (const YAML::Exception& e) {
      throw ConfigError("'" + qualified(key) + "': " + e.msg);
    }
  }

  void field(const char* key, Eigen::Matrix4d& value) {
    known_.insert(key);
    const YAML::Node child = node_[key];
    if (child) value = parseMatrix4(child, qualified(key));
  }

  /// Nested section, e.g. camera.left.
  template <typename Section>
  void section(const char* key, Section& value) {
    known_.insert(key);
    const YAML::Node child = node_[key];
    if (!child) return;
    if (!child.IsMap()) throw ConfigError("'" + qualified(key) + "' must be a map");
    Reader nested(child, qualified(key));
    visit(nested, value);
    nested.warnUnknownKeys();
  }

  void warnUnknownKeys() const {
    if (!node_.IsMap()) return;
    for (const auto& kv : node_) {
      const auto key = kv.first.as<std::string>();
      if (!known_.count(key)) {
        KLOG_WARN("config: unknown key '" << qualified(key) << "' ignored (typo?)");
      }
    }
  }

 private:
  std::string qualified(const std::string& key) const {
    return path_.empty() ? key : path_ + "." + key;
  }

  YAML::Node node_;
  std::string path_;
  std::set<std::string> known_;
};

/// Emits fields into a YAML::Emitter.
class Writer {
 public:
  explicit Writer(YAML::Emitter& out) : out_(out) {}

  template <typename T>
  void field(const char* key, const T& value) {
    out_ << YAML::Key << key << YAML::Value << value;
  }

  void field(const char* key, const std::vector<double>& value) {
    out_ << YAML::Key << key << YAML::Value << YAML::Flow << value;
  }

  void field(const char* key, const Eigen::Matrix4d& T) {
    out_ << YAML::Key << key << YAML::Value << YAML::BeginSeq;
    for (int r = 0; r < 4; ++r) {
      out_ << YAML::Flow << std::vector<double>{T(r, 0), T(r, 1), T(r, 2), T(r, 3)};
    }
    out_ << YAML::EndSeq;
  }

  template <typename Section>
  void section(const char* key, const Section& value) {
    out_ << YAML::Key << key << YAML::Value << YAML::BeginMap;
    visit(*this, const_cast<Section&>(value));  // Writer never mutates
    out_ << YAML::EndMap;
  }

 private:
  YAML::Emitter& out_;
};

// ---------------------------------------------------------------------------------------------
// Field lists — the single source of truth for parameter names.
// ---------------------------------------------------------------------------------------------

template <class V>
void visit(V& v, CameraIntrinsics& c) {
  v.field("fx", c.fx);
  v.field("fy", c.fy);
  v.field("cx", c.cx);
  v.field("cy", c.cy);
  v.field("distortion", c.distortion);
}

template <class V>
void visit(V& v, CameraConfig& c) {
  v.field("model", c.model);
  v.field("width", c.width);
  v.field("height", c.height);
  v.field("rate_hz", c.rate_hz);
  v.section("left", c.left);
  v.section("right", c.right);
  v.field("T_imu_left", c.T_imu_left);
  v.field("T_imu_right", c.T_imu_right);
  v.field("time_offset_s", c.time_offset_s);
}

template <class V>
void visit(V& v, ImuConfig& c) {
  v.field("accel_noise_density", c.accel_noise_density);
  v.field("gyro_noise_density", c.gyro_noise_density);
  v.field("accel_random_walk", c.accel_random_walk);
  v.field("gyro_random_walk", c.gyro_random_walk);
  v.field("gravity_magnitude", c.gravity_magnitude);
  v.field("rate_hz", c.rate_hz);
}

template <class V>
void visit(V& v, FrontendConfig& c) {
  v.field("max_features", c.max_features);
  v.field("min_feature_distance_px", c.min_feature_distance_px);
  v.field("klt_window_px", c.klt_window_px);
  v.field("klt_pyramid_levels", c.klt_pyramid_levels);
  v.field("klt_fb_threshold_px", c.klt_fb_threshold_px);
  v.field("ransac_threshold_px", c.ransac_threshold_px);
  v.field("stereo_epipolar_threshold_px", c.stereo_epipolar_threshold_px);
  v.field("use_clahe", c.use_clahe);
  v.field("gyro_aided_prediction", c.gyro_aided_prediction);
}

template <class V>
void visit(V& v, EstimatorConfig& c) {
  v.field("window_size", c.window_size);
  v.field("keyframe_parallax_px", c.keyframe_parallax_px);
  v.field("observation_sigma_px", c.observation_sigma_px);
  v.field("min_depth_m", c.min_depth_m);
  v.field("max_depth_m", c.max_depth_m);
  v.field("max_solver_iterations", c.max_solver_iterations);
  v.field("max_solver_time_s", c.max_solver_time_s);
  v.field("estimate_extrinsics", c.estimate_extrinsics);
}

template <class V>
void visit(V& v, DynamicConfig& c) {
  v.field("enable", c.enable);
  v.field("outlier_reprojection_px", c.outlier_reprojection_px);
  v.field("grid_rows", c.grid_rows);
  v.field("grid_cols", c.grid_cols);
  v.field("heat_decay", c.heat_decay);
  v.field("heat_mask_threshold", c.heat_mask_threshold);
}

template <class V>
void visit(V& v, LoopClosureConfig& c) {
  v.field("enable", c.enable);
  v.field("vocabulary_path", c.vocabulary_path);
  v.field("orb_features", c.orb_features);
  v.field("min_bow_score", c.min_bow_score);
  v.field("min_inliers", c.min_inliers);
  v.field("exclude_recent_keyframes", c.exclude_recent_keyframes);
  v.field("temporal_consistency", c.temporal_consistency);
}

template <class V>
void visit(V& v, MappingConfig& c) {
  v.field("voxel_size_m", c.voxel_size_m);
  v.field("max_integration_range_m", c.max_integration_range_m);
  v.field("grid_min_height_m", c.grid_min_height_m);
  v.field("grid_max_height_m", c.grid_max_height_m);
  v.field("persistence_expected_lifetime_s", c.persistence_expected_lifetime_s);
  v.field("persistence_miss_probability", c.persistence_miss_probability);
  v.field("persistence_false_alarm_probability", c.persistence_false_alarm_probability);
  v.field("persistence_prune_threshold", c.persistence_prune_threshold);
  v.field("map_path", c.map_path);
}

template <class V>
void visit(V& v, Config& c) {
  v.section("camera", c.camera);
  v.section("imu", c.imu);
  v.section("frontend", c.frontend);
  v.section("estimator", c.estimator);
  v.section("dynamic", c.dynamic);
  v.section("loop_closure", c.loop_closure);
  v.section("mapping", c.mapping);
}

Config parseRoot(const YAML::Node& root) {
  Config config;
  if (root && !root.IsNull()) {
    if (!root.IsMap()) throw ConfigError("configuration root must be a YAML map");
    Reader reader(root, "");
    visit(reader, config);
    reader.warnUnknownKeys();
  }
  const auto problems = config.validate();
  if (!problems.empty()) {
    std::ostringstream msg;
    msg << "invalid configuration:";
    for (const auto& p : problems) msg << "\n  - " << p;
    throw ConfigError(msg.str());
  }
  return config;
}

// Checks that the rotation block of a transform is orthonormal with det = +1.
bool isRigidTransform(const Eigen::Matrix4d& T) {
  const Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
  const bool orthonormal = (R.transpose() * R - Eigen::Matrix3d::Identity()).norm() < 1e-4;
  const bool proper = std::abs(R.determinant() - 1.0) < 1e-4;
  const bool last_row = (T.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9;
  return orthonormal && proper && last_row;
}

}  // namespace

// -----------------------------------------------------------------------------------------------

std::vector<std::string> Config::validate() const {
  std::vector<std::string> p;  // problems
  auto require = [&p](bool ok, const std::string& what) {
    if (!ok) p.push_back(what);
  };

  // Camera
  require(camera.model == "pinhole-radtan" || camera.model == "pinhole-equidistant",
          "camera.model must be 'pinhole-radtan' or 'pinhole-equidistant'");
  require(camera.width > 0 && camera.height > 0, "camera.width/height must be positive");
  require(camera.rate_hz > 0, "camera.rate_hz must be positive");
  for (const auto* cam : {&camera.left, &camera.right}) {
    const char* side = cam == &camera.left ? "left" : "right";
    require(cam->fx > 0 && cam->fy > 0,
            std::string("camera.") + side + " focal lengths must be > 0");
    const auto n = cam->distortion.size();
    if (camera.model == "pinhole-radtan") {
      require(n == 4 || n == 5,
              std::string("camera.") + side + ".distortion needs 4 or 5 coefficients for radtan");
    } else {
      require(n == 4,
              std::string("camera.") + side + ".distortion needs 4 coefficients for equidistant");
    }
  }
  require(isRigidTransform(camera.T_imu_left), "camera.T_imu_left is not a rigid transform");
  require(isRigidTransform(camera.T_imu_right), "camera.T_imu_right is not a rigid transform");
  const double baseline =
      (camera.T_imu_left.topRightCorner<3, 1>() - camera.T_imu_right.topRightCorner<3, 1>()).norm();
  require(baseline > 0.01, "stereo baseline (from T_imu_left/T_imu_right) must be > 1 cm");

  // IMU
  require(imu.accel_noise_density > 0 && imu.gyro_noise_density > 0 && imu.accel_random_walk > 0 &&
              imu.gyro_random_walk > 0,
          "imu noise parameters must all be > 0");
  require(imu.gravity_magnitude > 9.0 && imu.gravity_magnitude < 10.5,
          "imu.gravity_magnitude should be ~9.81");
  require(imu.rate_hz >= camera.rate_hz, "imu.rate_hz must be >= camera.rate_hz");

  // Front-end
  require(frontend.max_features >= 30, "frontend.max_features must be >= 30");
  require(frontend.min_feature_distance_px > 0, "frontend.min_feature_distance_px must be > 0");
  require(frontend.klt_window_px >= 5 && frontend.klt_window_px % 2 == 1,
          "frontend.klt_window_px must be odd and >= 5");

  // Estimator
  require(estimator.window_size >= 4 && estimator.window_size <= 30,
          "estimator.window_size must be in [4, 30]");
  require(estimator.min_depth_m > 0 && estimator.max_depth_m > estimator.min_depth_m,
          "estimator depth range must satisfy 0 < min_depth_m < max_depth_m");
  require(estimator.observation_sigma_px > 0, "estimator.observation_sigma_px must be > 0");

  // Dynamic
  require(dynamic.heat_decay >= 0 && dynamic.heat_decay < 1, "dynamic.heat_decay must be in [0,1)");
  require(dynamic.grid_rows > 0 && dynamic.grid_cols > 0, "dynamic grid size must be positive");

  // Loop closure
  require(loop_closure.min_inliers >= 6, "loop_closure.min_inliers must be >= 6 (PnP)");
  require(loop_closure.temporal_consistency >= 1, "loop_closure.temporal_consistency must be >= 1");

  // Mapping
  require(mapping.voxel_size_m > 0.005, "mapping.voxel_size_m must be > 5 mm");
  require(mapping.grid_max_height_m > mapping.grid_min_height_m,
          "mapping.grid_max_height_m must exceed grid_min_height_m");
  auto is_prob = [](double x) { return x > 0.0 && x < 1.0; };
  require(is_prob(mapping.persistence_miss_probability) &&
              is_prob(mapping.persistence_false_alarm_probability) &&
              is_prob(mapping.persistence_prune_threshold),
          "mapping.persistence_* probabilities must be in (0, 1)");
  require(mapping.persistence_expected_lifetime_s > 0,
          "mapping.persistence_expected_lifetime_s must be > 0");
  return p;
}

Config loadConfig(const std::string& yaml_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::BadFile&) {
    throw ConfigError("cannot open configuration file '" + yaml_path + "'");
  } catch (const YAML::Exception& e) {
    throw ConfigError("YAML error in '" + yaml_path + "': " + e.what());
  }
  return parseRoot(root);
}

Config loadConfigFromString(const std::string& yaml_text) {
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception& e) {
    throw ConfigError(std::string("YAML error: ") + e.what());
  }
  return parseRoot(root);
}

std::string toYaml(const Config& config) {
  YAML::Emitter out;
  out.SetDoublePrecision(10);
  out << YAML::BeginMap;
  Writer writer(out);
  visit(writer, const_cast<Config&>(config));
  out << YAML::EndMap;
  return std::string(out.c_str()) + "\n";
}

}  // namespace kestrel
