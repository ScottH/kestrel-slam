// SPDX-License-Identifier: Apache-2.0
//
// Tests for kestrel/sensors/stereo_rig.hpp.
//
// Ground truth is generated with OpenCV's *forward* projection models (projectPoints and
// fisheye::projectPoints) from known 3D points; the rig must then recover row-aligned stereo
// matches and the original 3D points through rectification + GTSAM triangulation. This checks
// our frame bookkeeping (T_R_L, R1, T_B_L'), which is where stereo pipelines usually go wrong.

#include <gtest/gtest.h>

#include <cstdlib>
#include <opencv2/calib3d.hpp>
#include <random>
#include <string>

#include "kestrel/config.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace kestrel {
namespace {

Config eurocConfig() {
  const char* repo = std::getenv("KESTREL_REPO_ROOT");
  if (repo == nullptr) throw std::runtime_error("KESTREL_REPO_ROOT not set");
  return loadConfig(std::string(repo) + "/kestrel_core/config/euroc.yaml");
}

/// Same rig geometry as EuRoC, but with a fisheye (equidistant) lens model.
Config fisheyeConfig() {
  Config c = eurocConfig();
  c.camera.model = "pinhole-equidistant";
  c.camera.left.distortion = {-0.01, 0.002, 0.0005, -0.0001};
  c.camera.right.distortion = {-0.012, 0.003, 0.0004, -0.0002};
  return c;
}

struct Projection {
  std::vector<Eigen::Vector3d> points_L;  // ground truth, raw left camera frame
  std::vector<cv::Point2f> left, right;   // raw (distorted) pixels
};

/// Random points in front of the rig, projected into both raw images; keeps those visible in both.
Projection projectRandomPoints(const CameraConfig& cam, int n) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> depth(1.0, 8.0), lateral(-0.35, 0.35);
  const Eigen::Matrix4d T_R_L = cam.T_imu_right.inverse() * cam.T_imu_left;
  auto K = [](const CameraIntrinsics& c) {
    return cv::Mat((cv::Mat_<double>(3, 3) << c.fx, 0, c.cx, 0, c.fy, c.cy, 0, 0, 1));
  };
  auto D = [](const CameraIntrinsics& c) { return cv::Mat(c.distortion, true).reshape(1, 1); };
  const bool fisheye = cam.model == "pinhole-equidistant";
  auto project = [&](const std::vector<cv::Point3d>& pts, const CameraIntrinsics& c) {
    std::vector<cv::Point2d> uv;
    const cv::Mat zero = cv::Mat::zeros(3, 1, CV_64F);
    if (fisheye) {
      cv::fisheye::projectPoints(pts, uv, zero, zero, K(c), D(c));
    } else {
      cv::projectPoints(pts, zero, zero, K(c), D(c), uv);
    }
    return uv;
  };

  Projection out;
  for (int i = 0; i < n; ++i) {
    const double z = depth(rng);
    const Eigen::Vector3d p_L(lateral(rng) * z, lateral(rng) * z * 0.6, z);
    const Eigen::Vector3d p_R = T_R_L.topLeftCorner<3, 3>() * p_L + T_R_L.topRightCorner<3, 1>();
    const auto uv_l = project({cv::Point3d(p_L.x(), p_L.y(), p_L.z())}, cam.left)[0];
    const auto uv_r = project({cv::Point3d(p_R.x(), p_R.y(), p_R.z())}, cam.right)[0];
    auto inside = [&](const cv::Point2d& p) {
      return p.x > 5 && p.y > 5 && p.x < cam.width - 5 && p.y < cam.height - 5;
    };
    if (!inside(uv_l) || !inside(uv_r)) continue;
    out.points_L.push_back(p_L);
    out.left.emplace_back(static_cast<float>(uv_l.x), static_cast<float>(uv_l.y));
    out.right.emplace_back(static_cast<float>(uv_r.x), static_cast<float>(uv_r.y));
  }
  return out;
}

void checkRig(const Config& config) {
  const StereoRig rig(config.camera);
  const Projection proj = projectRandomPoints(config.camera, 400);
  ASSERT_GT(proj.points_L.size(), 200u);

  const auto rect_l = rig.rectifyLeft(proj.left);
  const auto rect_r = rig.rectifyRight(proj.right);
  const gtsam::Pose3 T_B_L(config.camera.T_imu_left);

  double max_row_err = 0.0, max_point_err = 0.0, max_roundtrip = 0.0;
  const auto back_l = rig.unrectifyLeft(rect_l);
  for (std::size_t i = 0; i < proj.points_L.size(); ++i) {
    // Use the mean row, as a real matcher would after the row check.
    const gtsam::StereoPoint2 z_mean(rect_l[i].x, rect_r[i].x, 0.5 * (rect_l[i].y + rect_r[i].y));
    max_row_err = std::max(max_row_err, static_cast<double>(std::abs(rect_l[i].y - rect_r[i].y)));
    max_roundtrip = std::max(max_roundtrip, cv::norm(back_l[i] - proj.left[i]));

    const auto p_Lrect = rig.triangulate(z_mean);
    ASSERT_TRUE(p_Lrect.has_value());
    // Compare in the body frame: exercises T_B_L' as well as triangulation.
    const gtsam::Point3 p_B_est = rig.bodyToRectifiedLeft().transformFrom(*p_Lrect);
    const gtsam::Point3 p_B_true = T_B_L.transformFrom(gtsam::Point3(proj.points_L[i]));
    max_point_err = std::max(max_point_err, (p_B_est - p_B_true).norm() / proj.points_L[i].z());
  }
  // Pixels are stored as float, so ~1e-4 px quantization is expected.
  EXPECT_LT(max_row_err, 5e-3) << "rectified matches are not row-aligned";
  EXPECT_LT(max_roundtrip, 5e-3) << "rectify → unrectify does not round-trip";
  EXPECT_LT(max_point_err, 1e-4) << "triangulated points do not match ground truth";
}

TEST(StereoRig, EurocRadtanRectificationAndTriangulation) {
  checkRig(eurocConfig());
}

TEST(StereoRig, FisheyeRectificationAndTriangulation) {
  checkRig(fisheyeConfig());
}

TEST(StereoRig, BaselineAndFocalLength) {
  const StereoRig euroc(eurocConfig().camera);
  EXPECT_NEAR(euroc.baseline(), 0.110, 0.002);  // EuRoC VI-sensor baseline ≈ 11 cm

  const Config nominal;  // built-in Gemini 335L defaults: ideal pinhole, 95 mm baseline
  const StereoRig gemini(nominal.camera);
  EXPECT_NEAR(gemini.baseline(), 0.095, 1e-9);
  EXPECT_NEAR(gemini.focalLength(), 430.0, 1e-3);  // OpenCV rectification: float-level rounding
  // Already-rectified cameras: the rectified left camera coincides with the raw one.
  EXPECT_TRUE(gemini.bodyToRectifiedLeft().equals(gtsam::Pose3(nominal.camera.T_imu_left), 1e-9));
}

TEST(StereoRig, RejectsTinyDisparity) {
  const StereoRig rig(Config{}.camera);
  EXPECT_FALSE(rig.triangulate(gtsam::StereoPoint2(300.0, 299.9, 200.0)).has_value());
  const auto p = rig.triangulate(gtsam::StereoPoint2(300.0, 290.0, 200.0));
  ASSERT_TRUE(p.has_value());
  EXPECT_NEAR(p->z(), rig.focalLength() * rig.baseline() / 10.0, 1e-9);  // z = f·b / disparity
}

TEST(StereoRig, RejectsSwappedCameras) {
  Config c;
  std::swap(c.camera.T_imu_left, c.camera.T_imu_right);
  EXPECT_THROW(StereoRig rig(c.camera), std::runtime_error);
}

}  // namespace
}  // namespace kestrel
