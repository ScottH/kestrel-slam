// SPDX-License-Identifier: Apache-2.0
//
// Tests for the EuRoC (ASL format) reader on a tiny dataset written to a temporary folder.

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>

#include "kestrel/io/euroc.hpp"

namespace kestrel {
namespace {

namespace fs = std::filesystem;

class EurocTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() / ("kestrel_euroc_" + std::to_string(::getpid()));
    const fs::path mav = root_ / "mav0";
    for (const char* cam : {"cam0", "cam1"}) {
      fs::create_directories(mav / cam / "data");
      std::ofstream csv(mav / cam / "data.csv");
      csv << "#timestamp [ns],filename\r\n";  // CRLF, as some exports have
      for (long long ns : {1000000000LL, 1050000000LL}) {
        const std::string name = std::to_string(ns) + ".png";
        cv::imwrite((mav / cam / "data" / name).string(), cv::Mat(4, 6, CV_8UC1, cv::Scalar(7)));
        csv << ns << "," << name << "\r\n";
      }
    }
    // cam1 has one extra frame without a cam0 partner: must be ignored.
    {
      std::ofstream csv(mav / "cam1" / "data.csv", std::ios::app);
      csv << "1100000000,1100000000.png\n";
    }
    fs::create_directories(mav / "imu0");
    std::ofstream imu(mav / "imu0" / "data.csv");
    imu << "#timestamp [ns],w_RS_S_x,w_RS_S_y,w_RS_S_z,a_RS_S_x,a_RS_S_y,a_RS_S_z\n";
    imu << "1000000000,0.1,0.2,0.3,9.7,0.1,-0.2\n1005000000,0.1,0.2,0.3,9.8,0.1,-0.2\n";
    fs::create_directories(mav / "state_groundtruth_estimate0");
    std::ofstream gt(mav / "state_groundtruth_estimate0" / "data.csv");
    gt << "#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,bw_x,bw_y,bw_z,ba_x,ba_y,ba_z\n";
    gt << "1000000000,1,2,3,1,0,0,0,0,0,0,0,0,0,0,0,0\n";
  }
  void TearDown() override { fs::remove_all(root_); }
  fs::path root_;
};

TEST_F(EurocTest, ReadsStereoImuAndGroundTruth) {
  const EurocDataset ds(root_.string());
  ASSERT_EQ(ds.images().size(), 2u);
  EXPECT_NEAR(ds.images()[1].t, 1.05, 1e-12);
  ASSERT_EQ(ds.imu().size(), 2u);
  EXPECT_NEAR(ds.imu()[0].gyr.z(), 0.3, 1e-12);
  EXPECT_NEAR(ds.imu()[1].acc.x(), 9.8, 1e-12);
  ASSERT_EQ(ds.groundTruth().size(), 1u);
  EXPECT_NEAR(ds.groundTruth()[0].T_world_body.translation().y(), 2.0, 1e-12);
  const auto [l, r] = ds.loadStereo(0);
  EXPECT_EQ(l.cols, 6);
  EXPECT_EQ(r.at<uint8_t>(0, 0), 7);
}

TEST(Euroc, MissingFolderThrows) {
  EXPECT_THROW(EurocDataset("/nonexistent/euroc"), std::runtime_error);
}

}  // namespace
}  // namespace kestrel
