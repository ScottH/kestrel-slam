// SPDX-License-Identifier: Apache-2.0
//
// kestrel_info — prints library versions and a fully-resolved configuration.
//
// Usage:
//   kestrel_info                     # defaults
//   kestrel_info path/to/config.yaml # defaults overridden by the file, after validation
//
// Handy for checking that a container has the right dependencies, and for seeing exactly which
// parameter values a run will use (every key, including the ones the file did not mention).

#include <iostream>

#include "kestrel/common/logging.hpp"
#include "kestrel/config.hpp"
#include "kestrel/version.hpp"

int main(int argc, char** argv) {
  std::cout << "# Kestrel SLAM " << KESTREL_VERSION << "\n"
            << "#   Eigen  " << KESTREL_EIGEN_VERSION << "\n"
            << "#   OpenCV " << KESTREL_OPENCV_VERSION << "\n"
            << "#   Ceres  " << KESTREL_CERES_VERSION << "\n";
  try {
    const kestrel::Config config = argc > 1 ? kestrel::loadConfig(argv[1]) : kestrel::Config{};
    std::cout << "# Resolved configuration" << (argc > 1 ? std::string(" from ") + argv[1] : "")
              << ":\n"
              << kestrel::toYaml(config);
  } catch (const kestrel::ConfigError& e) {
    KLOG_ERROR(e.what());
    return 1;
  }
  return 0;
}
