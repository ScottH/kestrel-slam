// SPDX-License-Identifier: Apache-2.0
//
// Smoke test for kestrel/viz/foxglove_visualizer.hpp: drives the visualizer with synthetic
// frames and checks that a non-trivial MCAP file is produced and that the WebSocket server can
// be started (built only when the Foxglove SDK is available).

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <opencv2/core.hpp>

#include "kestrel/config.hpp"
#include "kestrel/viz/foxglove_visualizer.hpp"

namespace kestrel {
namespace {

namespace fs = std::filesystem;

TEST(FoxgloveVisualizer, WritesMcapAndServesWebSocket) {
  const Config config;
  const StereoRig rig(config.camera);
  const fs::path path =
      fs::temp_directory_path() / ("kestrel_viz_" + std::to_string(::getpid()) + ".mcap");
  {
    FoxgloveOptions options;
    options.mcap_path = path.string();
    options.websocket = true;
    options.host = "127.0.0.1";
    options.port = 0;  // any free port
    FoxgloveVisualizer viz(options, rig);

    cv::Mat image(config.camera.height, config.camera.width, CV_8UC1);
    cv::randu(image, 0, 255);
    for (int i = 0; i < 20; ++i) {
      const double t = 100.0 + i * 0.05;
      FeatureFrame frame;
      frame.t = t;
      for (int k = 0; k < 30; ++k) {
        TrackedFeature f;
        f.id = k;
        f.track_length = i + 1;
        f.left_raw = f.left_rect = cv::Point2f(20.0f + 10 * k, 50.0f + i);
        f.prev_left_raw = f.left_raw - cv::Point2f(0, 1);
        f.has_right = k % 2 == 0;
        frame.features.push_back(f);
      }
      VioEstimator::Output out;
      out.t = t;
      out.initialized = true;
      out.is_keyframe = i % 3 == 0;
      out.state = gtsam::NavState(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.1 * i, 0, 0)),
                                  gtsam::Vector3(2, 0, 0));
      FrameExtras extras;
      extras.ground_truth = out.state.pose();
      viz.logFrame(t, image, frame, out, extras);
      if (out.is_keyframe) viz.logLandmarks(t, {{1, gtsam::Point3(1, 2, 3)}, {2, gtsam::Point3()}});
    }
    VioEstimator::WindowGraph graph;
    graph.keyframes = {{0, gtsam::Pose3()}, {1, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0))}};
    graph.landmarks = {{5, gtsam::Point3(2, 1, 3)}, {6, gtsam::Point3(2, -1, 3)}};
    graph.newest_observations = {5, 6, 7};  // 7 is not in the window: must be skipped
    graph.imu_factors = 1;
    graph.visual_factors = 4;
    viz.logWindowGraph(100.5, graph);
    viz.logLoopClosure(100.5, gtsam::Pose3(), {gtsam::Pose3(), gtsam::Pose3()},
                       {{gtsam::Point3(0, 0, 0), gtsam::Point3(1, 1, 0)}});
    viz.logGroundTruthPath(100.0, {gtsam::Pose3(), gtsam::Pose3()});
    viz.logMessage(100.0, LogLevel::kInfo, "hello from the test");
  }  // destructor finishes the MCAP file
  ASSERT_TRUE(fs::exists(path));
  EXPECT_GT(fs::file_size(path), 20000u) << "MCAP suspiciously small";
  fs::remove(path);
}

TEST(FoxgloveVisualizer, ReportsUnusableMcapPath) {
  const Config config;
  const StereoRig rig(config.camera);
  FoxgloveOptions options;
  options.mcap_path = "/nonexistent-dir/x.mcap";
  EXPECT_THROW(FoxgloveVisualizer(options, rig), std::runtime_error);
}

}  // namespace
}  // namespace kestrel
