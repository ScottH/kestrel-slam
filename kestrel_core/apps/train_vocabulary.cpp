// SPDX-License-Identifier: Apache-2.0
//
// train_vocabulary — trains the bag-of-words vocabulary used by loop closure.
//
//   train_vocabulary <output.fbow> <dataset_root> [<dataset_root> ...]
//                    [--every N] [--k K] [--levels L] [--features F] [--threads T]
//
// Samples every N-th left image of each ASL-format dataset (EuRoC, TUM-VI, our converted
// recordings), extracts ORB descriptors exactly as the loop closer does, and builds a k-means tree
// with branching K and depth L (FBoW): K^L leaf words, e.g. 10^5 = 100 000 for the defaults.
//
// Recipe (docs/theory/08): train on places that are NOT the ones you evaluate on, so benchmark
// numbers measure generalization. Kestrel's reference vocabulary is trained on EuRoC MH_01+MH_02
// (a machine hall) and evaluated on the TUM-VI rooms.

#include <chrono>
#include <cstdio>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <string>
#include <thread>
#include <vector>

#include "kestrel/common/logging.hpp"
#include "kestrel/io/euroc.hpp"
#include "kestrel/loop/place_recognizer.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: train_vocabulary <output.fbow> <dataset_root> [...] [--every N] [--k K] "
                 "[--levels L] [--features F] [--threads T]\n";
    return 2;
  }
  const std::string output = argv[1];
  std::vector<std::string> roots;
  int every = 10, k = 10, levels = 5, features = 1000;
  int threads = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() { return std::stoi(argv[++i]); };
    if (a == "--every" && i + 1 < argc) {
      every = next();
    } else if (a == "--k" && i + 1 < argc) {
      k = next();
    } else if (a == "--levels" && i + 1 < argc) {
      levels = next();
    } else if (a == "--features" && i + 1 < argc) {
      features = next();
    } else if (a == "--threads" && i + 1 < argc) {
      threads = next();
    } else {
      roots.push_back(a);
    }
  }

  try {
    auto orb = cv::ORB::create(features);
    std::vector<cv::Mat> descriptors;
    std::size_t total = 0;
    for (const auto& root : roots) {
      const kestrel::EurocDataset dataset(root);
      for (std::size_t i = 0; i < dataset.images().size(); i += static_cast<std::size_t>(every)) {
        const auto [left, right] = dataset.loadStereo(i);
        std::vector<cv::KeyPoint> kps;
        cv::Mat d;
        orb->detectAndCompute(left, cv::noArray(), kps, d);
        if (d.empty()) continue;
        total += static_cast<std::size_t>(d.rows);
        descriptors.push_back(d);
      }
      KLOG_INFO(root << ": " << descriptors.size() << " images so far, " << total << " descriptors");
    }
    if (descriptors.empty()) {
      KLOG_ERROR("no descriptors extracted");
      return 1;
    }
    KLOG_INFO("training k=" << k << ", L=" << levels << " (" << threads << " threads) ...");
    const auto t0 = std::chrono::steady_clock::now();
    kestrel::trainFbowVocabulary(descriptors, k, levels, output, threads);
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("vocabulary %s: %zu images, %zu descriptors, k=%d L=%d, %.0f s\n", output.c_str(),
                descriptors.size(), total, k, levels, s);
  } catch (const std::exception& e) {
    KLOG_ERROR(e.what());
    return 1;
  }
  return 0;
}
