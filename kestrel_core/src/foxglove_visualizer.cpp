// SPDX-License-Identifier: Apache-2.0
//
// Foxglove SDK visualization. See kestrel/viz/foxglove_visualizer.hpp.

#include "kestrel/viz/foxglove_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/messages.hpp>
#include <foxglove/server.hpp>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace kestrel {
namespace {

namespace fm = foxglove::messages;

template <typename T>
T must(foxglove::FoxgloveResult<T>&& result, const std::string& what) {
  if (!result.has_value()) {
    throw std::runtime_error("Foxglove: " + what +
                             " failed: " + foxglove::strerror(result.error()));
  }
  return std::move(result.value());
}

uint64_t toNs(double t) {
  return static_cast<uint64_t>(std::llround(std::max(t, 0.0) * 1e9));
}

fm::Timestamp stamp(double t) {
  const uint64_t ns = toNs(t);
  return fm::Timestamp{static_cast<uint32_t>(ns / 1000000000ULL),
                       static_cast<uint32_t>(ns % 1000000000ULL)};
}

fm::Pose toPose(const gtsam::Pose3& T) {
  const gtsam::Quaternion q = T.rotation().toQuaternion();
  fm::Pose p;
  p.position = fm::Vector3{T.x(), T.y(), T.z()};
  p.orientation = fm::Quaternion{q.x(), q.y(), q.z(), q.w()};
  return p;
}

fm::FrameTransform toTf(double t, const std::string& parent, const std::string& child,
                        const gtsam::Pose3& T) {
  fm::FrameTransform tf;
  tf.timestamp = stamp(t);
  tf.parent_frame_id = parent;
  tf.child_frame_id = child;
  const auto p = toPose(T);
  tf.translation = p.position;
  tf.rotation = p.orientation;
  return tf;
}

fm::Color colour(double r, double g, double b, double a = 1.0) {
  return fm::Color{r, g, b, a};
}

/// Red (new) → green (tracked for ≥ 20 frames).
fm::Color ageColour(int track_length) {
  const double a = std::min(1.0, (track_length - 1) / 20.0);
  return colour(1.0 - a, a, 0.0);
}

// JSON schema for /vio/stats, so Foxglove's Plot panel can list the fields.
constexpr const char* kStatsSchema = R"({
  "type": "object",
  "properties": {
    "t": {"type": "number"}, "initialized": {"type": "boolean"},
    "keyframe": {"type": "boolean"}, "reset": {"type": "boolean"},
    "features": {"type": "integer"}, "tracked": {"type": "integer"},
    "detected": {"type": "integer"}, "stereo_matched": {"type": "integer"},
    "lost_klt": {"type": "integer"}, "lost_fb": {"type": "integer"},
    "lost_ransac": {"type": "integer"}, "thinned": {"type": "integer"},
    "landmarks_in_window": {"type": "integer"}, "new_landmarks": {"type": "integer"},
    "visual_factors": {"type": "integer"}, "outliers": {"type": "integer"},
    "frontend_ms": {"type": "number"}, "solve_ms": {"type": "number"},
    "frame_ms": {"type": "number"},
    "speed_mps": {"type": "number"},
    "bias_acc": {"type": "object", "properties": {"x": {"type": "number"},
      "y": {"type": "number"}, "z": {"type": "number"}}},
    "bias_gyr": {"type": "object", "properties": {"x": {"type": "number"},
      "y": {"type": "number"}, "z": {"type": "number"}}},
    "error_m": {"type": "number"}, "loops": {"type": "integer"}
  }
})";

constexpr const char* kFactorStatsSchema = R"({
  "type": "object",
  "properties": {
    "keyframes": {"type": "integer"}, "landmarks": {"type": "integer"},
    "imu_factors": {"type": "integer"}, "visual_factors": {"type": "integer"},
    "prior_factors": {"type": "integer"}, "other_factors": {"type": "integer"},
    "total_factors": {"type": "integer"}, "newest_observations": {"type": "integer"}
  }
})";

fm::SpherePrimitive sphere(const gtsam::Point3& p, double size, const fm::Color& c) {
  fm::SpherePrimitive s;
  fm::Pose pose;
  pose.position = fm::Vector3{p.x(), p.y(), p.z()};
  pose.orientation = fm::Quaternion{0, 0, 0, 1};
  s.pose = pose;
  s.size = fm::Vector3{size, size, size};
  s.color = c;
  return s;
}

fm::Point3 pt(const gtsam::Point3& p) {
  return fm::Point3{p.x(), p.y(), p.z()};
}

}  // namespace

struct FoxgloveVisualizer::Impl {
  explicit Impl(const FoxgloveOptions& opt, const StereoRig& rig_in)
      : options(opt), context(foxglove::Context::create()), rig(rig_in) {
    if (opt.websocket) {
      foxglove::WebSocketServerOptions ws;
      ws.context = context;
      ws.name = "kestrel";
      ws.host = opt.host;
      ws.port = opt.port;
      server.emplace(must(foxglove::WebSocketServer::create(std::move(ws)), "WebSocket server"));
    }
    if (!opt.mcap_path.empty()) {
      foxglove::McapWriterOptions mo;
      mo.context = context;
      mo.path = opt.mcap_path;
      mcap.emplace(must(foxglove::McapWriter::create(mo), "MCAP writer " + opt.mcap_path));
    }
    tf.emplace(must(fm::FrameTransformChannel::create("/tf", context), "channel /tf"));
    image.emplace(
        must(fm::CompressedImageChannel::create("/camera/left/image", context), "channel image"));
    features.emplace(must(fm::ImageAnnotationsChannel::create("/camera/left/features", context),
                          "channel features"));
    pose.emplace(must(fm::PoseInFrameChannel::create("/vio/pose", context), "channel pose"));
    path.emplace(must(fm::PosesInFrameChannel::create("/vio/path", context), "channel path"));
    gt_path.emplace(
        must(fm::PosesInFrameChannel::create("/groundtruth/path", context), "channel gt path"));
    landmarks.emplace(
        must(fm::PointCloudChannel::create("/vio/landmarks", context), "channel landmarks"));
    camera.emplace(must(fm::SceneUpdateChannel::create("/vio/camera", context), "channel camera"));
    log.emplace(must(fm::LogChannel::create("/log", context), "channel log"));
    slam_path.emplace(
        must(fm::PosesInFrameChannel::create("/slam/path", context), "channel slam path"));
    loops.emplace(must(fm::SceneUpdateChannel::create("/slam/graph", context), "channel graph"));
    factors.emplace(
        must(fm::SceneUpdateChannel::create("/vio/factors", context), "channel factors"));
    const foxglove::Schema fschema{"kestrel.FactorStats", "jsonschema",
                                   reinterpret_cast<const std::byte*>(kFactorStatsSchema),
                                   std::strlen(kFactorStatsSchema)};
    factor_stats.emplace(must(
        foxglove::RawChannel::create("/vio/factor_stats", "json", fschema, context), "channel"));
    const foxglove::Schema schema{"kestrel.FrameStats", "jsonschema",
                                  reinterpret_cast<const std::byte*>(kStatsSchema),
                                  std::strlen(kStatsSchema)};
    stats.emplace(
        must(foxglove::RawChannel::create("/vio/stats", "json", schema, context), "channel stats"));
  }

  ~Impl() {
    if (mcap) mcap->close();
    if (server) server->stop();
  }

  FoxgloveOptions options;
  foxglove::Context context;
  const StereoRig& rig;
  std::optional<foxglove::WebSocketServer> server;
  std::optional<foxglove::McapWriter> mcap;
  std::optional<fm::FrameTransformChannel> tf;
  std::optional<fm::CompressedImageChannel> image;
  std::optional<fm::ImageAnnotationsChannel> features;
  std::optional<fm::PoseInFrameChannel> pose;
  std::optional<fm::PosesInFrameChannel> path, gt_path;
  std::optional<fm::PointCloudChannel> landmarks;
  std::optional<fm::SceneUpdateChannel> camera;
  std::optional<fm::LogChannel> log;
  std::optional<fm::PosesInFrameChannel> slam_path;
  std::optional<fm::SceneUpdateChannel> loops;
  std::optional<fm::SceneUpdateChannel> factors;
  std::optional<foxglove::RawChannel> factor_stats;
  std::optional<foxglove::RawChannel> stats;
  std::vector<fm::Pose> keyframe_path;
  bool sent_camera = false;
  std::mutex mutex;  // logMessage() may be called from the logging sink of other threads
};

FoxgloveVisualizer::FoxgloveVisualizer(const FoxgloveOptions& options, const StereoRig& rig)
    : impl_(std::make_unique<Impl>(options, rig)) {
  if (options.websocket) {
    KLOG_INFO("Foxglove: live at ws://" << options.host << ":" << impl_->server->port()
                                        << " (Foxglove → Open connection)");
  }
  if (!options.mcap_path.empty()) KLOG_INFO("Foxglove: recording to " << options.mcap_path);
}

FoxgloveVisualizer::~FoxgloveVisualizer() = default;

void FoxgloveVisualizer::logFrame(double t, const cv::Mat& left_raw, const FeatureFrame& frame,
                                  const VioEstimator::Output& out, const FrameExtras& extras) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto& I = *impl_;
  const uint64_t ns = toNs(t);

  // --- image + tracks ----------------------------------------------------------------------------
  std::vector<uchar> jpeg;
  cv::imencode(".jpg", left_raw, jpeg, {cv::IMWRITE_JPEG_QUALITY, I.options.jpeg_quality});
  fm::CompressedImage img;
  img.timestamp = stamp(t);
  img.frame_id = "camera_left";
  img.format = "jpeg";
  img.data.resize(jpeg.size());
  std::memcpy(img.data.data(), jpeg.data(), jpeg.size());
  I.image->log(img, ns);

  fm::ImageAnnotations ann;
  ann.timestamp = stamp(t);
  fm::PointsAnnotation points, flow;
  points.timestamp = flow.timestamp = stamp(t);
  points.type = fm::PointsAnnotation::PointsAnnotationType::POINTS;
  points.thickness = 4.0;
  flow.type = fm::PointsAnnotation::PointsAnnotationType::LINE_LIST;
  flow.thickness = 1.0;
  for (const auto& f : frame.features) {
    const fm::Color c = ageColour(f.track_length);
    points.points.push_back(fm::Point2{f.left_raw.x, f.left_raw.y});
    points.outline_colors.push_back(c);
    if (f.track_length > 1) {
      flow.points.push_back(fm::Point2{f.prev_left_raw.x, f.prev_left_raw.y});
      flow.points.push_back(fm::Point2{f.left_raw.x, f.left_raw.y});
      flow.outline_colors.push_back(c);
      flow.outline_colors.push_back(c);
    }
    if (f.has_right) {
      fm::CircleAnnotation ring;
      ring.timestamp = stamp(t);
      ring.position = fm::Point2{f.left_raw.x, f.left_raw.y};
      ring.diameter = 9.0;
      ring.thickness = 1.0;
      ring.outline_color = colour(0.0, 0.8, 1.0, 0.8);
      ann.circles.push_back(std::move(ring));
    }
  }
  ann.points.push_back(std::move(flow));
  ann.points.push_back(std::move(points));
  I.features->log(ann, ns);

  // --- poses and frames
  // ----------------------------------------------------------------------------
  if (out.initialized) {
    I.tf->log(toTf(t, "world", "body", out.state.pose()), ns);
    I.tf->log(toTf(t, "body", "camera_left", I.rig.bodyToRectifiedLeft()), ns);
    fm::PoseInFrame p;
    p.timestamp = stamp(t);
    p.frame_id = "world";
    p.pose = toPose(out.state.pose());
    I.pose->log(p, ns);
    if (out.is_keyframe) {
      I.keyframe_path.push_back(toPose(out.state.pose()));
      fm::PosesInFrame path;
      path.timestamp = stamp(t);
      path.frame_id = "world";
      path.poses = I.keyframe_path;
      I.path->log(path, ns);
    }
  }
  if (extras.ground_truth) I.tf->log(toTf(t, "world", "body_gt", *extras.ground_truth), ns);

  // Camera frustum, attached to the camera frame (frame_locked: follows the TF).
  {
    const auto& K = *I.rig.calibration();
    const double w = 2.0 * K.px(), h = 2.0 * K.py(), d = 0.3;  // 30 cm deep
    auto ray = [&](double u, double v) {
      return fm::Point3{(u - K.px()) / K.fx() * d, (v - K.py()) / K.fy() * d, d};
    };
    fm::LinePrimitive lines;
    lines.type = fm::LinePrimitive::LineType::LINE_LIST;
    lines.thickness = 2.0;
    lines.scale_invariant = true;
    lines.color = colour(1.0, 0.6, 0.0);
    const fm::Point3 o{0, 0, 0};
    const fm::Point3 c[4] = {ray(0, 0), ray(w, 0), ray(w, h), ray(0, h)};
    for (int i = 0; i < 4; ++i) {
      lines.points.insert(lines.points.end(), {o, c[i], c[i], c[(i + 1) % 4]});
    }
    fm::SceneEntity entity;
    entity.timestamp = stamp(t);
    entity.frame_id = "camera_left";
    entity.id = "frustum";
    entity.frame_locked = true;
    entity.lines.push_back(std::move(lines));
    fm::SceneUpdate update;
    update.entities.push_back(std::move(entity));
    if (!I.sent_camera || out.is_keyframe) I.camera->log(update, ns);  // refresh occasionally
    I.sent_camera = true;
  }

  // --- numbers for Plot panels
  // ----------------------------------------------------------------------
  const auto& s = frame.stats;
  std::ostringstream js;
  js.precision(9);
  js << "{\"t\":" << t << ",\"initialized\":" << (out.initialized ? "true" : "false")
     << ",\"keyframe\":" << (out.is_keyframe ? "true" : "false")
     << ",\"reset\":" << (out.reset ? "true" : "false") << ",\"features\":" << frame.features.size()
     << ",\"tracked\":" << s.tracked << ",\"detected\":" << s.detected
     << ",\"stereo_matched\":" << s.stereo_matched << ",\"lost_klt\":" << s.lost_klt
     << ",\"lost_fb\":" << s.lost_fb << ",\"lost_ransac\":" << s.lost_ransac
     << ",\"thinned\":" << s.thinned << ",\"landmarks_in_window\":" << out.landmarks_in_window
     << ",\"new_landmarks\":" << out.new_landmarks << ",\"visual_factors\":" << out.visual_factors
     << ",\"outliers\":" << out.outlier_ids.size() << ",\"frontend_ms\":" << s.ms_total
     << ",\"solve_ms\":" << out.solve_ms << ",\"frame_ms\":" << extras.frame_ms
     << ",\"speed_mps\":" << out.state.velocity().norm();
  const auto ba = out.bias.accelerometer(), bg = out.bias.gyroscope();
  js << ",\"bias_acc\":{\"x\":" << ba.x() << ",\"y\":" << ba.y() << ",\"z\":" << ba.z() << "}"
     << ",\"bias_gyr\":{\"x\":" << bg.x() << ",\"y\":" << bg.y() << ",\"z\":" << bg.z() << "}";
  if (extras.loops >= 0) js << ",\"loops\":" << extras.loops;
  if (extras.ground_truth && out.initialized) {
    js << ",\"error_m\":" << (out.state.position() - extras.ground_truth->translation()).norm();
  }
  js << "}";
  const std::string json = js.str();
  I.stats->log(reinterpret_cast<const std::byte*>(json.data()), json.size(), ns);
}

void FoxgloveVisualizer::logLandmarks(double t,
                                      const std::vector<std::pair<int, gtsam::Point3>>& lms) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  fm::PointCloud cloud;
  cloud.timestamp = stamp(t);
  cloud.frame_id = "world";
  cloud.point_stride = 16;  // x, y, z, id as float32
  using NT = fm::PackedElementField::NumericType;
  cloud.fields = {
      {"x", 0, NT::FLOAT32}, {"y", 4, NT::FLOAT32}, {"z", 8, NT::FLOAT32}, {"id", 12, NT::FLOAT32}};
  cloud.data.resize(lms.size() * 16);
  for (std::size_t i = 0; i < lms.size(); ++i) {
    const float v[4] = {static_cast<float>(lms[i].second.x()),
                        static_cast<float>(lms[i].second.y()),
                        static_cast<float>(lms[i].second.z()), static_cast<float>(lms[i].first)};
    std::memcpy(cloud.data.data() + i * 16, v, 16);
  }
  impl_->landmarks->log(cloud, toNs(t));
}

void FoxgloveVisualizer::logGroundTruthPath(double t, const std::vector<gtsam::Pose3>& poses) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  fm::PosesInFrame path;
  path.timestamp = stamp(t);
  path.frame_id = "world";
  for (const auto& p : poses) path.poses.push_back(toPose(p));
  impl_->gt_path->log(path, toNs(t));
}

void FoxgloveVisualizer::logWindowGraph(double t, const VioEstimator::WindowGraph& g) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto& I = *impl_;
  const uint64_t ns = toNs(t);
  auto keyframes = g.keyframes;
  std::sort(keyframes.begin(), keyframes.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  fm::SceneEntity entity;
  entity.timestamp = stamp(t);
  entity.frame_id = "world";
  entity.id = "window";
  // Nodes: one sphere per keyframe state in the window.
  for (const auto& kf : keyframes) {
    entity.spheres.push_back(sphere(kf.second.translation(), 0.04, colour(1.0, 0.6, 0.0)));
  }
  // IMU factors connect consecutive keyframes (green).
  fm::LinePrimitive imu;
  imu.type = fm::LinePrimitive::LineType::LINE_STRIP;
  imu.thickness = 3.0;
  imu.scale_invariant = true;
  imu.color = colour(0.1, 0.9, 0.2);
  for (const auto& kf : keyframes) imu.points.push_back(pt(kf.second.translation()));
  entity.lines.push_back(std::move(imu));
  // Visual factors of the newest keyframe: rays from its camera to each observed landmark.
  if (!keyframes.empty()) {
    const gtsam::Point3 cam = (keyframes.back().second * I.rig.bodyToRectifiedLeft()).translation();
    std::unordered_map<int, gtsam::Point3> where(g.landmarks.begin(), g.landmarks.end());
    fm::LinePrimitive rays;
    rays.type = fm::LinePrimitive::LineType::LINE_LIST;
    rays.thickness = 1.0;
    rays.scale_invariant = true;
    rays.color = colour(0.0, 0.8, 1.0, 0.35);
    for (const int id : g.newest_observations) {
      const auto it = where.find(id);
      if (it == where.end()) continue;
      rays.points.push_back(pt(cam));
      rays.points.push_back(pt(it->second));
    }
    entity.lines.push_back(std::move(rays));
  }
  fm::SceneUpdate update;
  update.entities.push_back(std::move(entity));
  I.factors->log(update, ns);

  std::ostringstream js;
  js << "{\"keyframes\":" << keyframes.size() << ",\"landmarks\":" << g.landmarks.size()
     << ",\"imu_factors\":" << g.imu_factors << ",\"visual_factors\":" << g.visual_factors
     << ",\"prior_factors\":" << g.prior_factors << ",\"other_factors\":" << g.other_factors
     << ",\"total_factors\":"
     << g.imu_factors + g.visual_factors + g.prior_factors + g.other_factors
     << ",\"newest_observations\":" << g.newest_observations.size() << "}";
  const std::string json = js.str();
  I.factor_stats->log(reinterpret_cast<const std::byte*>(json.data()), json.size(), ns);
}

void FoxgloveVisualizer::logLoopClosure(
    double t, const gtsam::Pose3& T_map_odom, const std::vector<gtsam::Pose3>& map_keyframe_path,
    const std::vector<std::pair<gtsam::Point3, gtsam::Point3>>& loop_edges) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  auto& I = *impl_;
  const uint64_t ns = toNs(t);
  I.tf->log(toTf(t, "map", "world", T_map_odom), ns);

  fm::PosesInFrame path;
  path.timestamp = stamp(t);
  path.frame_id = "map";
  for (const auto& p : map_keyframe_path) path.poses.push_back(toPose(p));
  I.slam_path->log(path, ns);

  fm::SceneEntity entity;
  entity.timestamp = stamp(t);
  entity.frame_id = "map";
  entity.id = "pose_graph";
  // Odometry edges between consecutive keyframes (grey) and the keyframe nodes.
  fm::LinePrimitive odometry;
  odometry.type = fm::LinePrimitive::LineType::LINE_STRIP;
  odometry.thickness = 1.5;
  odometry.scale_invariant = true;
  odometry.color = colour(0.6, 0.6, 0.6);
  for (const auto& p : map_keyframe_path) {
    odometry.points.push_back(pt(p.translation()));
    entity.spheres.push_back(sphere(p.translation(), 0.025, colour(0.8, 0.8, 0.8)));
  }
  entity.lines.push_back(std::move(odometry));
  // Verified loop closures (magenta).
  fm::LinePrimitive lines;
  lines.type = fm::LinePrimitive::LineType::LINE_LIST;
  lines.thickness = 3.0;
  lines.scale_invariant = true;
  lines.color = colour(1.0, 0.0, 1.0);
  for (const auto& edge : loop_edges) {
    lines.points.push_back(pt(edge.first));
    lines.points.push_back(pt(edge.second));
  }
  entity.lines.push_back(std::move(lines));
  fm::SceneUpdate update;
  update.entities.push_back(std::move(entity));
  I.loops->log(update, ns);
}

void FoxgloveVisualizer::logMessage(double t, LogLevel level, const std::string& message) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  fm::Log entry;
  entry.timestamp = stamp(t);
  entry.name = "kestrel";
  entry.message = message;
  switch (level) {
    case LogLevel::kDebug:
      entry.level = fm::Log::LogLevel::DEBUG;
      break;
    case LogLevel::kInfo:
      entry.level = fm::Log::LogLevel::INFO;
      break;
    case LogLevel::kWarn:
      entry.level = fm::Log::LogLevel::WARNING;
      break;
    case LogLevel::kError:
      entry.level = fm::Log::LogLevel::ERROR;
      break;
  }
  impl_->log->log(entry, toNs(t));
}

}  // namespace kestrel
