// SPDX-License-Identifier: Apache-2.0
//
// run_frontend — runs the stereo front-end on a EuRoC-format sequence and reports the statistics
// used by the M2 exit criteria (features per frame, stereo matches, track lengths, timing).
//
//   run_frontend <dataset_root> <config.yaml> [--video out.mp4] [--max-frames N] [--no-gyro]
//
// Gyro-aided prediction uses the dataset's IMU: the rotation between consecutive frames comes
// from GTSAM preintegration of the gyro samples (via kestrel::integrateImu).

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <opencv2/videoio.hpp>
#include <optional>
#include <string>
#include <vector>

#include "kestrel/common/logging.hpp"
#include "kestrel/config.hpp"
#include "kestrel/frontend/feature_tracker.hpp"
#include "kestrel/imu/imu.hpp"
#include "kestrel/io/euroc.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace {

double percentile(std::vector<double> v, double p) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  return v[static_cast<std::size_t>(p * (v.size() - 1))];
}

int usage() {
  std::cerr << "usage: run_frontend <dataset_root> <config.yaml> [--video out.mp4] "
               "[--max-frames N] [--no-gyro]\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) return usage();
  std::string video_path;
  std::size_t max_frames = 0;
  bool use_gyro = true;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--video" && i + 1 < argc) {
      video_path = argv[++i];
    } else if (a == "--max-frames" && i + 1 < argc) {
      max_frames = std::stoul(argv[++i]);
    } else if (a == "--no-gyro") {
      use_gyro = false;
    } else {
      return usage();
    }
  }

  try {
    const kestrel::Config config = kestrel::loadConfig(argv[2]);
    const kestrel::EurocDataset dataset(argv[1]);
    auto rig = std::make_shared<kestrel::StereoRig>(config.camera);
    kestrel::FeatureTracker tracker(config.frontend, rig);
    const auto imu_params = kestrel::makeImuParams(config.imu);

    const std::size_t n =
        max_frames > 0 ? std::min(max_frames, dataset.images().size()) : dataset.images().size();
    KLOG_INFO("run_frontend: " << n << " stereo frames, " << dataset.imu().size()
                               << " IMU samples, gyro aid " << (use_gyro ? "on" : "off"));

    cv::VideoWriter video;
    std::vector<double> features, stereo, ms, new_per_frame;
    std::vector<double> finished_track_lengths;
    std::map<int, int> alive;  // id → length, to measure how long tracks live

    for (std::size_t i = 0; i < n; ++i) {
      const auto [left, right] = dataset.loadStereo(i);
      const double t = dataset.images()[i].t;

      std::optional<gtsam::Rot3> R_prev_cur;
      if (use_gyro && i > 0) {
        gtsam::PreintegratedCombinedMeasurements pim(imu_params);
        if (kestrel::integrateImu(dataset.imu(), dataset.images()[i - 1].t, t, &pim) > 0) {
          R_prev_cur = pim.deltaRij();
        }
      }
      const auto frame = tracker.track(t, left, right, R_prev_cur);

      std::map<int, int> now;
      for (const auto& f : frame.features) now[f.id] = f.track_length;
      for (const auto& [id, len] : alive) {
        if (!now.count(id)) finished_track_lengths.push_back(len);
      }
      alive.swap(now);

      features.push_back(static_cast<double>(frame.features.size()));
      stereo.push_back(frame.stats.stereo_matched);
      new_per_frame.push_back(frame.stats.detected);
      ms.push_back(frame.stats.ms_total);

      if (!video_path.empty()) {
        const cv::Mat canvas = kestrel::FeatureTracker::drawDebug(left, frame);
        if (!video.isOpened()) {
          video.open(video_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), config.camera.rate_hz,
                     canvas.size());
        }
        video.write(canvas);
      }
    }
    for (const auto& [id, len] : alive) finished_track_lengths.push_back(len);

    auto mean = [](const std::vector<double>& v) {
      return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };
    std::printf("frames                 %zu\n", n);
    std::printf("features / frame       mean %.1f   p5 %.0f\n", mean(features),
                percentile(features, 0.05));
    std::printf("stereo matches / frame mean %.1f   p5 %.0f\n", mean(stereo),
                percentile(stereo, 0.05));
    std::printf("new features / frame   mean %.1f\n", mean(new_per_frame));
    std::printf("track length [frames]  median %.0f   p90 %.0f\n",
                percentile(finished_track_lengths, 0.5), percentile(finished_track_lengths, 0.9));
    std::printf("time / frame [ms]      median %.2f   p95 %.2f\n", percentile(ms, 0.5),
                percentile(ms, 0.95));
  } catch (const std::exception& e) {
    KLOG_ERROR(e.what());
    return 1;
  }
  return 0;
}
