// SPDX-License-Identifier: Apache-2.0
//
// kestrel/viz/foxglove_visualizer.hpp — live (WebSocket) and recorded (MCAP) visualization of
// the SLAM internals in Foxglove, using the Foxglove SDK.
//
// TUTORIAL NOTE
// -------------
// A SLAM system is hard to debug from numbers alone: "ATE got worse" says nothing about *why*.
// This class streams what the algorithms see and decide, using Foxglove's built-in message
// types (no ROS needed), to two sinks at once:
//   * a WebSocket server: open Foxglove → "Open connection" → ws://<host>:8765 and watch live,
//   * an MCAP file: open it later in Foxglove on any machine and scrub through time.
//
// Topics (see docs/visualization.md for the matching Foxglove layout):
//   /tf                    world → body (estimate), world → body_gt (ground truth, if known),
//                          body → camera_left (rectified left camera)
//   /camera/left/image     left image (JPEG)
//   /camera/left/features  tracks as image annotations: colour = track age (red new → green
//                          old), lines = motion since the previous frame, rings = stereo match
//   /vio/pose              current pose estimate
//   /vio/path              keyframe trajectory;  /groundtruth/path: ground truth (aligned)
//   /vio/landmarks         3D landmarks in the estimation window
//   /vio/camera            camera frustum (3D panel)
//   /vio/stats             per-frame numbers for Plot panels (JSON): features, losses by cause,
//                          stereo matches, landmarks, solve time, biases, error vs. ground truth
//   /log                   estimator messages (initialization, resets, warnings)
//
// Timestamps are the sensor times, so a recording replays with the original timing.

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "kestrel/common/logging.hpp"
#include "kestrel/estimator/vio_estimator.hpp"
#include "kestrel/frontend/feature_tracker.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace kestrel {

struct FoxgloveOptions {
  bool websocket = false;        ///< serve live data on ws://host:port
  std::string host = "0.0.0.0";  ///< 0.0.0.0: reachable from other machines on the network
  uint16_t port = 8765;
  std::string mcap_path;  ///< record to this MCAP file (empty: no recording)
  int jpeg_quality = 80;
};

/// Extra per-frame numbers for the stats topic that the pipeline outputs don't carry.
struct FrameExtras {
  double frame_ms = 0.0;                     ///< total processing time of this frame
  std::optional<gtsam::Pose3> ground_truth;  ///< aligned to the estimate's world frame
};

class FoxgloveVisualizer {
 public:
  /// Starts the requested sinks. Throws std::runtime_error if e.g. the port is in use.
  FoxgloveVisualizer(const FoxgloveOptions& options, const StereoRig& rig);
  ~FoxgloveVisualizer();  // finishes the MCAP file

  FoxgloveVisualizer(const FoxgloveVisualizer&) = delete;
  FoxgloveVisualizer& operator=(const FoxgloveVisualizer&) = delete;

  /// Logs one processed frame: image + tracks, pose, TF, stats (and the path at keyframes).
  void logFrame(double t, const cv::Mat& left_raw, const FeatureFrame& frame,
                const VioEstimator::Output& output, const FrameExtras& extras = {});

  /// Logs the landmarks currently in the estimation window (call at keyframes; it is not free).
  void logLandmarks(double t, const std::vector<std::pair<int, gtsam::Point3>>& landmarks);

  /// Logs the full ground-truth trajectory once (already aligned to the estimate's world frame).
  void logGroundTruthPath(double t, const std::vector<gtsam::Pose3>& poses);

  /// Forwards a log message to the /log topic.
  void logMessage(double t, LogLevel level, const std::string& message);

 private:
  struct Impl;  // keeps the Foxglove SDK headers out of this header's users
  std::unique_ptr<Impl> impl_;
};

}  // namespace kestrel
