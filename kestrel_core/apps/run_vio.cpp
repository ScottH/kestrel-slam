// SPDX-License-Identifier: Apache-2.0
//
// run_vio — runs the visual-inertial odometry on an ASL-format dataset (EuRoC, TUM-VI) and
// writes trajectories for evaluation.
//
//   run_vio <dataset_root> <config.yaml> <output_dir> [--max-frames N] [--video tracks.mp4]
//
// Writes, in TUM format ("t x y z qx qy qz qw", world ← body):
//   <output_dir>/estimate.tum      every processed frame (IMU-predicted between keyframes)
//   <output_dir>/keyframes.tum     keyframes only (smoothed estimates)
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
#include <numeric>
#include <opencv2/videoio.hpp>
#include <string>
#include <vector>

#include "kestrel/common/logging.hpp"
#include "kestrel/config.hpp"
#include "kestrel/io/euroc.hpp"
#include "kestrel/odometry.hpp"

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

int usage() {
  std::cerr << "usage: run_vio <dataset_root> <config.yaml> <output_dir> [--max-frames N] "
               "[--video tracks.mp4]\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) return usage();
  std::size_t max_frames = 0;
  std::string video_path;
  for (int i = 4; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--max-frames" && i + 1 < argc) {
      max_frames = std::stoul(argv[++i]);
    } else if (a == "--video" && i + 1 < argc) {
      video_path = argv[++i];
    } else {
      return usage();
    }
  }

  try {
    const kestrel::Config config = kestrel::loadConfig(argv[2]);
    const kestrel::EurocDataset dataset(argv[1]);
    const std::filesystem::path out_dir(argv[3]);
    std::filesystem::create_directories(out_dir);
    std::ofstream(out_dir / "config.yaml") << kestrel::toYaml(config);
    std::ofstream est(out_dir / "estimate.tum"), kfs(out_dir / "keyframes.tum");
    if (!dataset.groundTruth().empty()) {
      std::ofstream gt(out_dir / "groundtruth.tum");
      for (const auto& g : dataset.groundTruth()) writeTum(gt, g.t, g.T_world_body);
    }

    kestrel::VisualInertialOdometry vio(config);
    const auto& images = dataset.images();
    const auto& imu = dataset.imu();
    const std::size_t n = max_frames > 0 ? std::min(max_frames, images.size()) : images.size();
    KLOG_INFO("run_vio: " << n << " frames, " << imu.size() << " IMU samples → " << out_dir);

    cv::VideoWriter video;
    std::vector<double> frame_ms, solve_ms;
    int keyframes = 0, resets = 0, outliers = 0;
    std::size_t next_imu = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const double t = images[i].t;
      // Deliver IMU up to (and a little past) the frame, as a live driver would.
      while (next_imu < imu.size() && imu[next_imu].t <= t + 0.02) vio.addImu(imu[next_imu++]);
      const auto [left, right] = dataset.loadStereo(i);

      const auto t0 = std::chrono::steady_clock::now();
      const auto out = vio.addStereo(t, left, right);
      frame_ms.push_back(
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
      if (!out || !out->initialized) continue;

      writeTum(est, t, out->state.pose());
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
        KLOG_INFO("frame " << i << "/" << n << "  t=" << std::fixed << std::setprecision(2) << t
                           << "  p=(" << p.x() << ", " << p.y() << ", " << p.z() << ")  landmarks "
                           << out->landmarks_in_window);
      }
    }
    std::printf("frames %zu, keyframes %d, resets %d, outliers reported %d\n", n, keyframes, resets,
                outliers);
    std::printf("time / frame [ms]     median %.1f  p95 %.1f\n", percentile(frame_ms, 0.5),
                percentile(frame_ms, 0.95));
    std::printf("solve / keyframe [ms] median %.1f  p95 %.1f\n", percentile(solve_ms, 0.5),
                percentile(solve_ms, 0.95));
  } catch (const std::exception& e) {
    KLOG_ERROR(e.what());
    return 1;
  }
  return 0;
}
