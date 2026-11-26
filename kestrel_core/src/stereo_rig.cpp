// SPDX-License-Identifier: Apache-2.0
//
// Stereo rectification glue between OpenCV and GTSAM. See kestrel/sensors/stereo_rig.hpp.

#include "kestrel/sensors/stereo_rig.hpp"

#include <gtsam/geometry/StereoCamera.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace kestrel {
namespace {

cv::Mat toK(const CameraIntrinsics& c) {
  return (cv::Mat_<double>(3, 3) << c.fx, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0);
}

cv::Mat toD(const CameraIntrinsics& c) {
  cv::Mat D(1, static_cast<int>(c.distortion.size()), CV_64F);
  for (std::size_t i = 0; i < c.distortion.size(); ++i)
    D.at<double>(0, static_cast<int>(i)) = c.distortion[i];
  return D;
}

// Undistortion is iterative; OpenCV's default (5 iterations) is not enough for strongly
// distorted lenses near the image border, so we ask for convergence explicitly.
const cv::TermCriteria kUndistortCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
                                          1e-12);

}  // namespace

StereoRig::StereoRig(const CameraConfig& config) {
  fisheye_ = config.model == "pinhole-equidistant";
  size_ = cv::Size(config.width, config.height);
  K_left_ = toK(config.left);
  D_left_ = toD(config.left);
  K_right_ = toK(config.right);
  D_right_ = toD(config.right);

  // Transform from the left to the right camera: p_R = R · p_L + t  (OpenCV's convention).
  const Eigen::Matrix4d T_R_L = config.T_imu_right.inverse() * config.T_imu_left;
  cv::Mat R, t;
  cv::eigen2cv(Eigen::Matrix3d(T_R_L.topLeftCorner<3, 3>()), R);
  cv::eigen2cv(Eigen::Vector3d(T_R_L.topRightCorner<3, 1>()), t);

  cv::Mat Q;
  if (fisheye_) {
    // balance = 0 keeps only valid pixels; fov_scale = 1 keeps the original focal length scale.
    cv::fisheye::stereoRectify(K_left_, D_left_, K_right_, D_right_, size_, R, t, R1_, R2_, P1_,
                               P2_, Q, cv::CALIB_ZERO_DISPARITY, size_, 0.0, 1.0);
  } else {
    // alpha = 0: crop to the region valid in both images (no black borders).
    cv::stereoRectify(K_left_, D_left_, K_right_, D_right_, size_, R, t, R1_, R2_, P1_, P2_, Q,
                      cv::CALIB_ZERO_DISPARITY, 0.0, size_);
  }

  // P1 = [f 0 cx 0; 0 f cy 0; 0 0 1 0],  P2 = [f 0 cx −f·b; ...]  →  b = −P2(0,3) / f.
  const double f = P1_.at<double>(0, 0);
  const double baseline = -P2_.at<double>(0, 3) / f;
  if (!(baseline > 0.0)) {
    throw std::runtime_error(
        "stereo rectification produced a non-positive baseline: is the right camera really to "
        "the right of the left camera (check T_imu_left / T_imu_right)?");
  }
  // (shared_ptr is boost:: in GTSAM 4.2 and std:: in 4.3; constructing it directly works for both.)
  calibration_ = gtsam::Cal3_S2Stereo::shared_ptr(new gtsam::Cal3_S2Stereo(
      f, P1_.at<double>(1, 1), 0.0, P1_.at<double>(0, 2), P1_.at<double>(1, 2), baseline));

  // T_B_L' = T_B_L · T_L_L',  where T_L_L' rotates rectified-left coordinates back: R1ᵀ.
  Eigen::Matrix3d R1;
  cv::cv2eigen(R1_, R1);
  const gtsam::Pose3 T_B_L(config.T_imu_left);
  T_B_Lrect_ = T_B_L * gtsam::Pose3(gtsam::Rot3(R1.transpose()), gtsam::Point3::Zero());

  // Look-up tables for whole-image rectification (cheap to build, ~ms).
  if (fisheye_) {
    cv::fisheye::initUndistortRectifyMap(K_left_, D_left_, R1_, P1_, size_, CV_32FC1, map_left_x_,
                                         map_left_y_);
    cv::fisheye::initUndistortRectifyMap(K_right_, D_right_, R2_, P2_, size_, CV_32FC1,
                                         map_right_x_, map_right_y_);
  } else {
    cv::initUndistortRectifyMap(K_left_, D_left_, R1_, P1_, size_, CV_32FC1, map_left_x_,
                                map_left_y_);
    cv::initUndistortRectifyMap(K_right_, D_right_, R2_, P2_, size_, CV_32FC1, map_right_x_,
                                map_right_y_);
  }
}

std::vector<cv::Point2f> StereoRig::undistortToRectified(const std::vector<cv::Point2f>& raw,
                                                         bool left) const {
  std::vector<cv::Point2f> out;
  if (raw.empty()) return out;
  const cv::Mat& K = left ? K_left_ : K_right_;
  const cv::Mat& D = left ? D_left_ : D_right_;
  const cv::Mat& Rr = left ? R1_ : R2_;
  const cv::Mat& P = left ? P1_ : P2_;
  if (fisheye_) {
    cv::fisheye::undistortPoints(raw, out, K, D, Rr, P, kUndistortCriteria);
  } else {
    cv::undistortPoints(raw, out, K, D, Rr, P, kUndistortCriteria);
  }
  return out;
}

std::vector<cv::Point2f> StereoRig::distortFromRectified(const std::vector<cv::Point2f>& rect,
                                                         bool left) const {
  std::vector<cv::Point2f> out;
  if (rect.empty()) return out;
  // Rectified pixel → ray in the rectified camera → rotate into the raw camera → project with
  // the raw intrinsics and distortion.
  const cv::Mat& P = left ? P1_ : P2_;
  const cv::Mat& Rr = left ? R1_ : R2_;
  const double f = P.at<double>(0, 0), fy = P.at<double>(1, 1);
  const double cx = P.at<double>(0, 2), cy = P.at<double>(1, 2);
  std::vector<cv::Point3d> rays;
  rays.reserve(rect.size());
  for (const auto& p : rect) rays.emplace_back((p.x - cx) / f, (p.y - cy) / fy, 1.0);
  cv::Mat rvec;
  cv::Rodrigues(Rr.t(), rvec);  // raw = R^T · rectified
  const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  std::vector<cv::Point2d> projected;
  if (fisheye_) {
    cv::fisheye::projectPoints(rays, projected, rvec, tvec, left ? K_left_ : K_right_,
                               left ? D_left_ : D_right_);
  } else {
    cv::projectPoints(rays, rvec, tvec, left ? K_left_ : K_right_, left ? D_left_ : D_right_,
                      projected);
  }
  out.reserve(projected.size());
  for (const auto& p : projected)
    out.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
  return out;
}

std::vector<cv::Point2f> StereoRig::rectifyLeft(const std::vector<cv::Point2f>& raw) const {
  return undistortToRectified(raw, true);
}
std::vector<cv::Point2f> StereoRig::rectifyRight(const std::vector<cv::Point2f>& raw) const {
  return undistortToRectified(raw, false);
}
std::vector<cv::Point2f> StereoRig::unrectifyLeft(const std::vector<cv::Point2f>& rect) const {
  return distortFromRectified(rect, true);
}
std::vector<cv::Point2f> StereoRig::unrectifyRight(const std::vector<cv::Point2f>& rect) const {
  return distortFromRectified(rect, false);
}

std::optional<gtsam::Point3> StereoRig::triangulate(const gtsam::StereoPoint2& z,
                                                    double min_disparity_px) const {
  if (z.uL() - z.uR() < min_disparity_px) return std::nullopt;
  // A StereoCamera at the origin of L' back-projects straight into L' coordinates.
  const gtsam::StereoCamera camera(gtsam::Pose3(), calibration_);
  return camera.backproject(z);
}

cv::Mat StereoRig::rectifyLeftImage(const cv::Mat& raw) const {
  cv::Mat out;
  cv::remap(raw, out, map_left_x_, map_left_y_, cv::INTER_LINEAR);
  return out;
}

cv::Mat StereoRig::rectifyRightImage(const cv::Mat& raw) const {
  cv::Mat out;
  cv::remap(raw, out, map_right_x_, map_right_y_, cv::INTER_LINEAR);
  return out;
}

}  // namespace kestrel
