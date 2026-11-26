// SPDX-License-Identifier: Apache-2.0
//
// Tests for the configuration loader and the logging facade.

#include <gtest/gtest.h>

#include <cstdlib>
#include <string>
#include <vector>

#include "kestrel/common/logging.hpp"
#include "kestrel/config.hpp"

namespace kestrel {
namespace {

TEST(Config, DefaultsAreValid) {
  const Config config;
  EXPECT_TRUE(config.validate().empty());
}

TEST(Config, EmptyYamlGivesDefaults) {
  const Config config = loadConfigFromString("");
  EXPECT_EQ(config.frontend.max_features, Config{}.frontend.max_features);
}

TEST(Config, OverridesNestedValues) {
  const Config config = loadConfigFromString(R"(
camera:
  width: 1280
  height: 800
  left: { fx: 640.5, fy: 641.0, cx: 640.0, cy: 400.0, distortion: [0.1, -0.05, 0.001, 0.002] }
imu:
  rate_hz: 400
frontend:
  max_features: 300
)");
  EXPECT_EQ(config.camera.width, 1280);
  EXPECT_DOUBLE_EQ(config.camera.left.fx, 640.5);
  EXPECT_DOUBLE_EQ(config.camera.left.distortion[0], 0.1);
  EXPECT_DOUBLE_EQ(config.camera.right.fx, CameraIntrinsics{}.fx);  // untouched → default
  EXPECT_DOUBLE_EQ(config.imu.rate_hz, 400.0);
  EXPECT_EQ(config.frontend.max_features, 300);
}

TEST(Config, ParsesTransformsInBothLayouts) {
  const Config flat = loadConfigFromString(R"(
camera:
  T_imu_right: [1,0,0,0.05,  0,1,0,0,  0,0,1,0,  0,0,0,1]
)");
  EXPECT_DOUBLE_EQ(flat.camera.T_imu_right(0, 3), 0.05);

  const Config nested = loadConfigFromString(R"(
camera:
  T_imu_right:
    - [1, 0, 0, 0.07]
    - [0, 1, 0, 0]
    - [0, 0, 1, 0]
    - [0, 0, 0, 1]
)");
  EXPECT_DOUBLE_EQ(nested.camera.T_imu_right(0, 3), 0.07);
}

TEST(Config, RejectsInvalidValues) {
  // Window too small and a non-rigid extrinsic: both problems should be reported together.
  try {
    loadConfigFromString(R"(
estimator: { window_size: 2 }
camera:
  T_imu_left: [2,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
)");
    FAIL() << "expected ConfigError";
  } catch (const ConfigError& e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("window_size"), std::string::npos) << msg;
    EXPECT_NE(msg.find("T_imu_left"), std::string::npos) << msg;
  }
}

TEST(Config, RejectsWrongTypes) {
  EXPECT_THROW(loadConfigFromString("frontend: { max_features: lots }"), ConfigError);
}

TEST(Config, MissingFileThrows) {
  EXPECT_THROW(loadConfig("/nonexistent/kestrel.yaml"), ConfigError);
}

TEST(Config, YamlRoundTrip) {
  Config original;
  original.frontend.max_features = 321;
  original.camera.T_imu_right(1, 3) = 0.002;
  original.loop_closure.vocabulary_path = "/data/vocab.bin";
  const Config reloaded = loadConfigFromString(toYaml(original));
  EXPECT_EQ(reloaded.frontend.max_features, 321);
  EXPECT_DOUBLE_EQ(reloaded.camera.T_imu_right(1, 3), 0.002);
  EXPECT_EQ(reloaded.loop_closure.vocabulary_path, "/data/vocab.bin");
}

// Every configuration shipped in the repository must load and validate.
TEST(Config, ShippedConfigsAreValid) {
  const char* repo = std::getenv("KESTREL_REPO_ROOT");
  ASSERT_NE(repo, nullptr) << "KESTREL_REPO_ROOT not set by CMake";
  for (const std::string rel :
       {"kestrel_core/config/euroc.yaml", "kestrel_ros/config/gemini_335lg.yaml"}) {
    SCOPED_TRACE(rel);
    const Config config = loadConfig(std::string(repo) + "/" + rel);
    EXPECT_TRUE(config.validate().empty());
  }
  // Spot-check that values really came from the file.
  const Config euroc = loadConfig(std::string(repo) + "/kestrel_core/config/euroc.yaml");
  EXPECT_EQ(euroc.camera.width, 752);
  const double baseline = (euroc.camera.T_imu_left.col(3) - euroc.camera.T_imu_right.col(3)).norm();
  EXPECT_NEAR(baseline, 0.110, 0.005);  // EuRoC stereo baseline ≈ 11 cm
}

TEST(Logging, UnknownKeysAreWarnedAndSinkIsReplaceable) {
  std::vector<std::string> messages;
  setLogSink([&](LogLevel level, const char*, int, const std::string& msg) {
    if (level == LogLevel::kWarn) messages.push_back(msg);
  });
  loadConfigFromString("frontend: { max_featurez: 10 }");
  setLogSink({});  // restore default sink
  ASSERT_EQ(messages.size(), 1u);
  EXPECT_NE(messages[0].find("frontend.max_featurez"), std::string::npos);
}

TEST(Logging, LevelFiltersMessages) {
  int count = 0;
  setLogSink([&](LogLevel, const char*, int, const std::string&) { ++count; });
  setLogLevel(LogLevel::kWarn);
  KLOG_INFO("dropped");
  KLOG_WARN("kept");
  setLogLevel(LogLevel::kInfo);
  setLogSink({});
  EXPECT_EQ(count, 1);
}

}  // namespace
}  // namespace kestrel
