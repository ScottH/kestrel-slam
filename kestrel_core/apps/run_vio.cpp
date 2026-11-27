// SPDX-License-Identifier: Apache-2.0
//
// run_vio — runs the visual-inertial odometry on an ASL-format dataset (EuRoC, TUM-VI) and
// writes trajectories for evaluation; optionally visualizes everything in Foxglove.
//
//   run_vio <dataset_root> <config.yaml> <output_dir> [options]
//
// Options:
//   --start-frame S     skip the first S frames (e.g. an incomplete download)
//   --max-frames N      process at most N frames
//   --video FILE.mp4    write the front-end debug video
//   --foxglove          serve live visualization on ws://0.0.0.0:8765 (Foxglove → Open connection)
//   --port P            WebSocket port for --foxglove (default 8765)
//   --mcap FILE.mcap    record the visualization to an MCAP file (open it in Foxglove)
//   --rate R            pace processing at R× real time (e.g. 1.0 to watch live); default: as fast
//                       as possible
//   --async             run loop closure in its own thread as on the robot (default: inline, so
//                       results are reproducible)
//
// Writes, in TUM format ("t x y z qx qy qz qw", world ← body):
//   <output_dir>/estimate.tum      every processed frame (IMU-predicted between keyframes)
//   <output_dir>/keyframes.tum     keyframes only (smoothed estimates)
//   <output_dir>/estimate_map.tum  every frame in the loop-corrected map frame (online)
//   <output_dir>/keyframes_map.tum final pose-graph keyframe poses (after all loop closures)
//   <output_dir>/groundtruth.tum   the dataset's ground truth, if it has one
//   <output_dir>/config.yaml       the exact configuration used
// Score with:  tools/evaluate.py <output_dir>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <memory>
#include <numeric>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <vector>

#include "kestrel/common/logging.hpp"
#include "kestrel/config.hpp"
#include "kestrel/io/euroc.hpp"
#include "kestrel/slam_system.hpp"
#ifdef KESTREL_WITH_FOXGLOVE
#include "kestrel/viz/foxglove_visualizer.hpp"
#endif

namespace {

void writeTum(std::ofstream& out, double t, const gtsam::Pose3& T) {
  const gtsam::Quaternion q = T.rotation().toQuaternion();
  const gtsam::Point3& p = T.translation();
  out << std::fixed << std::setprecision(9) << t << ' ' << std::setprecision(6) << p.x() << ' '
      << p.y() << ' ' << p.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
      << '\n';
}

double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  return v[static_cast<std::size_t>(p * (v.size() - 1))];
}

/// Ground-truth pose nearest in time (ground truth is ≥ 100 Hz, so nearest is good enough for
/// display; evaluation uses evo's proper association).
const kestrel::GroundTruthPose* nearestGroundTruth(const std::vector<kestrel::GroundTruthPose>& gt,
                                                   double t) {
  if (gt.empty()) return nullptr;
  auto it = std::lower_bound(gt.begin(), gt.end(), t,
                             [](const kestrel::GroundTruthPose& g, double v) { return g.t < v; });
  if (it == gt.end()) it = std::prev(it);
  if (it != gt.begin() && std::abs(std::prev(it)->t - t) < std::abs(it->t - t)) it = std::prev(it);
  return std::abs(it->t - t) < 0.05 ? &*it : nullptr;
}

int usage() {
  std::cerr << "usage: run_vio <dataset_root> <config.yaml> <output_dir> [--start-frame S] "
               "[--max-frames N] [--video FILE.mp4] [--foxglove] [--port P] [--mcap FILE.mcap] "
               "[--rate R]\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) return usage();
  std::size_t max_frames = 0, start_frame = 0;
  std::string video_path, mcap_path;
  bool foxglove_live = false, async_loops = false;
  int port = 8765;
  double rate = 0.0;
  for (int i = 4; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--start-frame" && i + 1 < argc) {
      start_frame = std::stoul(argv[++i]);
    } else if (a == "--max-frames" && i + 1 < argc) {
      max_frames = std::stoul(argv[++i]);
    } else if (a == "--video" && i + 1 < argc) {
      video_path = argv[++i];
    } else if (a == "--foxglove") {
      foxglove_live = true;
    } else if (a == "--port" && i + 1 < argc) {
      port = std::stoi(argv[++i]);
    } else if (a == "--mcap" && i + 1 < argc) {
      mcap_path = argv[++i];
    } else if (a == "--async") {
      async_loops = true;
    } else if (a == "--rate" && i + 1 < argc) {
      rate = std::stod(argv[++i]);
    } else {
      return usage();
    }
  }

  try {
    kestrel::Config config = kestrel::loadConfig(argv[2]);
    config.loop_closure.synchronous = !async_loops;
    const kestrel::EurocDataset dataset(argv[1]);
    const std::filesystem::path out_dir(argv[3]);
    std::filesystem::create_directories(out_dir);
    std::ofstream(out_dir / "config.yaml") << kestrel::toYaml(config);
    std::ofstream est(out_dir / "estimate.tum"), kfs(out_dir / "keyframes.tum"),
        est_map(out_dir / "estimate_map.tum");
    const auto& ground_truth = dataset.groundTruth();
    if (!ground_truth.empty()) {
      std::ofstream gt(out_dir / "groundtruth.tum");
      for (const auto& g : ground_truth) writeTum(gt, g.t, g.T_world_body);
    }

    kestrel::SlamSystem slam(config);
    kestrel::VisualInertialOdometry& vio = slam.odometry();

    // --- optional Foxglove visualization -----------------------------------------------------
#ifdef KESTREL_WITH_FOXGLOVE
    std::unique_ptr<kestrel::FoxgloveVisualizer> viz;
    // Sensor time of the frame being processed, used to stamp log messages. Starts at the first
    // frame so messages logged before it do not land at t = 0 (1970) on the Foxglove timeline.
    double viz_time = start_frame < dataset.images().size() ? dataset.images()[start_frame].t : 0.0;
    if (foxglove_live || !mcap_path.empty()) {
      kestrel::FoxgloveOptions fo;
      fo.websocket = foxglove_live;
      fo.port = static_cast<uint16_t>(port);
      fo.mcap_path = mcap_path;
      viz = std::make_unique<kestrel::FoxgloveVisualizer>(fo, vio.rig());
      // Mirror log messages into Foxglove's Log panel (and keep printing them).
      kestrel::setLogSink(
          [&viz, &viz_time](kestrel::LogLevel level, const char*, int, const std::string& msg) {
            std::cerr << "[kestrel] [" << kestrel::toString(level) << "] " << msg << '\n';
            if (viz) viz->logMessage(viz_time, level, msg);
          });
    }
#else
    if (foxglove_live || !mcap_path.empty()) {
      KLOG_ERROR("built without the Foxglove SDK: --foxglove/--mcap unavailable");
      return 1;
    }
#endif

    const auto& images = dataset.images();
    const auto& imu = dataset.imu();
    // Frames [start_frame, end): e.g. to skip a damaged or incomplete beginning of a sequence.
    const std::size_t end =
        max_frames > 0 ? std::min(images.size(), start_frame + max_frames) : images.size();
    const std::size_t n = end > start_frame ? end - start_frame : 0;
    KLOG_INFO("run_vio: " << n << " frames, " << imu.size() << " IMU samples → " << out_dir);

    cv::VideoWriter video;
    std::vector<double> frame_ms, solve_ms;
    int keyframes = 0, resets = 0, outliers = 0;
    std::size_t next_imu = 0;
    // Ground truth lives in its own world frame; for display it is aligned to the estimate's
    // world frame with the first estimated pose (evaluation does a proper SE(3) fit instead).
    std::optional<gtsam::Pose3> T_est_gt;
    const auto wall_start = std::chrono::steady_clock::now();
    const double t_first = start_frame < images.size() ? images[start_frame].t : 0.0;

    for (std::size_t i = start_frame; i < end; ++i) {
      const double t = images[i].t;
      if (rate > 0.0) {  // pace to `rate` × real time
        const auto due = wall_start + std::chrono::duration<double>((t - t_first) / rate);
        std::this_thread::sleep_until(due);
      }
      // Deliver IMU up to (and a little past) the frame, as a live driver would.
      while (next_imu < imu.size() && imu[next_imu].t <= t + 0.02) vio.addImu(imu[next_imu++]);
      const auto [left, right] = dataset.loadStereo(i);
#ifdef KESTREL_WITH_FOXGLOVE
      viz_time = t;
#endif

      const auto t0 = std::chrono::steady_clock::now();
      const auto slam_out = slam.addStereo(t, left, right);
      const std::optional<kestrel::VioEstimator::Output> out =
          slam_out ? std::optional<kestrel::VioEstimator::Output>(slam_out->odometry) : std::nullopt;
      const double ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      frame_ms.push_back(ms);
      if (!out) continue;

#ifdef KESTREL_WITH_FOXGLOVE
      if (viz) {
        kestrel::FrameExtras extras;
        extras.frame_ms = ms;
        const auto* gt = nearestGroundTruth(ground_truth, t);
        if (gt && out->initialized) {
          if (!T_est_gt) {
            T_est_gt = out->state.pose() * gt->T_world_body.inverse();
            std::vector<gtsam::Pose3> path;
            for (std::size_t k = 0; k < ground_truth.size(); k += 10) {
              path.push_back(*T_est_gt * ground_truth[k].T_world_body);
            }
            viz->logGroundTruthPath(t, path);
          }
          extras.ground_truth = *T_est_gt * gt->T_world_body;
        }
        viz->logFrame(t, left, vio.lastFrame(), *out, extras);
        if (out->is_keyframe) viz->logLandmarks(t, vio.estimator().landmarksInWindow());
        if (out->is_keyframe && slam.loopCloser()) {
          const auto* lc = slam.loopCloser();
          std::vector<gtsam::Pose3> path;
          std::map<int, gtsam::Point3> position;
          for (const auto& [id, kt, pose] : lc->keyframePoses()) {
            path.push_back(pose);
            position[id] = pose.translation();
          }
          std::vector<std::pair<gtsam::Point3, gtsam::Point3>> edges;
          for (const auto& loop : lc->loops()) {
            if (position.count(loop.query_id) && position.count(loop.match_id)) {
              edges.emplace_back(position[loop.match_id], position[loop.query_id]);
            }
          }
          viz->logLoopClosure(t, slam_out->T_map_odom, path, edges);
        }
      }
#endif
      if (!out->initialized) continue;

      writeTum(est, t, out->state.pose());
      writeTum(est_map, t, slam_out->T_map_body);
      if (out->is_keyframe) {
        writeTum(kfs, t, out->state.pose());
        ++keyframes;
        solve_ms.push_back(out->solve_ms);
      }
      resets += out->reset;
      outliers += static_cast<int>(out->outlier_ids.size());
      if (!video_path.empty()) {
        const cv::Mat canvas = kestrel::FeatureTracker::drawDebug(left, vio.lastFrame());
        if (!video.isOpened()) {
          video.open(video_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), config.camera.rate_hz,
                     canvas.size());
        }
        video.write(canvas);
      }
      if (i % 200 == 0) {
        const auto& p = out->state.position();
        KLOG_INFO("frame " << i << "/" << end << "  t=" << std::fixed << std::setprecision(2) << t
                           << "  p=(" << p.x() << ", " << p.y() << ", " << p.z() << ")  landmarks "
                           << out->landmarks_in_window);
      }
    }
    (void)T_est_gt;
    // Let loop closure finish, then write the final optimized keyframe trajectory.
    slam.finish();
    int loops = 0;
    if (const auto* lc = slam.loopCloser()) {
      std::ofstream kfm(out_dir / "keyframes_map.tum");
      for (const auto& [id, kt, pose] : lc->keyframePoses()) writeTum(kfm, kt, pose);
      loops = static_cast<int>(lc->loops().size());
      // Every verified loop, for analysis: t_query t_match x y z qx qy qz qw inliers score, where
      // the pose is the query body in the match body frame (tools/analyze_loops.py).
      std::ofstream lf(out_dir / "loops.txt");
      lf << "# t_query t_match x y z qx qy qz qw inliers score\n";
      for (const auto& l : lc->loops()) {
        const gtsam::Quaternion q = l.T_match_query.rotation().toQuaternion();
        const gtsam::Point3& p = l.T_match_query.translation();
        lf << std::fixed << std::setprecision(9) << l.t_query << ' ' << l.t_match << ' '
           << std::setprecision(6) << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.x() << ' '
           << q.y() << ' ' << q.z() << ' ' << q.w() << ' ' << l.inliers << ' ' << l.score << '\n';
      }
    }
    std::printf("loop closures %d\n", loops);
    std::printf("frames %zu, keyframes %d, resets %d, outliers reported %d\n", n, keyframes, resets,
                outliers);
    std::printf("time / frame [ms]     median %.1f  p95 %.1f\n", percentile(frame_ms, 0.5),
                percentile(frame_ms, 0.95));
    std::printf("solve / keyframe [ms] median %.1f  p95 %.1f\n", percentile(solve_ms, 0.5),
                percentile(solve_ms, 0.95));
#ifdef KESTREL_WITH_FOXGLOVE
    kestrel::setLogSink({});  // the visualizer is about to be destroyed
#endif
  } catch (const std::exception& e) {
    kestrel::setLogSink({});
    KLOG_ERROR(e.what());
    return 1;
  }
  return 0;
}
