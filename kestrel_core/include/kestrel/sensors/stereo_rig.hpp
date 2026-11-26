// SPDX-License-Identifier: Apache-2.0
//
// kestrel/sensors/stereo_rig.hpp — calibrated stereo camera: distortion, rectification and the
// rectified pinhole model used by the estimator.
//
// TUTORIAL NOTE
// -------------
// Theory: docs/theory/02_cameras_and_stereo.md.  References: [Hartley04], [Bouguet00].
//
// The two IR cameras of the Gemini 335L are not perfectly parallel and their lenses distort.
// *Rectification* computes a virtual pair of ideal pinhole cameras — same focal length, image rows
// aligned, right camera exactly `baseline` metres along +x — and a rotation for each real camera
// into its virtual one. After rectifying feature coordinates:
//
//   * a stereo match lies on the SAME ROW in both images (v_left == v_right), which makes
//     matching a 1-D search and gives a trivial outlier check, and
//   * depth follows from disparity alone:  z = f · b / (u_left − u_right).
//
// That is exactly the model of GTSAM's Cal3_S2Stereo / StereoCamera / stereo factors, which the
// estimator uses. Division of labour:
//
//   OpenCV  stereoRectify (or fisheye::stereoRectify)  computes R1, R2, P1, P2
//   OpenCV  undistortPoints / projectPoints             maps raw pixels ↔ rectified pixels
//   GTSAM   Cal3_S2Stereo + StereoCamera                 projection, triangulation, Jacobians
//   Kestrel (this file)                                  frame bookkeeping between them
//
// We rectify *feature points*, not whole images: tracking runs on the raw images (no resampling
// blur, no wasted pixels at the borders), and only a few hundred points per frame are converted.
// Whole-image rectification is still available for visualization and debugging.
//
// Frames:  B = body (IMU), L/R = raw left/right camera (optical), L'/R' = rectified cameras.
//          R1 rotates L into L':  p_L' = R1 · p_L.

#pragma once

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoPoint2.h>

#include <opencv2/core.hpp>
#include <optional>
#include <vector>

#include "kestrel/config.hpp"

namespace kestrel {

class StereoRig {
 public:
  /// Builds the rig from the camera section of the configuration (runs stereo rectification).
  explicit StereoRig(const CameraConfig& config);

  // --- rectified model --------------------------------------------------------------------------

  /// Rectified pinhole intrinsics + baseline, as used by GTSAM's stereo factors.
  const gtsam::Cal3_S2Stereo::shared_ptr& calibration() const { return calibration_; }
  double focalLength() const { return calibration_->fx(); }
  double baseline() const { return calibration_->baseline(); }

  /// Pose of the rectified left camera L' in the body (IMU) frame: T_B_L'.
  const gtsam::Pose3& bodyToRectifiedLeft() const { return T_B_Lrect_; }

  // --- raw ↔ rectified pixel conversion ---------------------------------------------------------

  /// Raw (distorted) left pixels → rectified left pixels.
  std::vector<cv::Point2f> rectifyLeft(const std::vector<cv::Point2f>& raw) const;
  /// Raw (distorted) right pixels → rectified right pixels.
  std::vector<cv::Point2f> rectifyRight(const std::vector<cv::Point2f>& raw) const;
  /// Rectified left pixels → raw left pixels (e.g. to predict where a point appears in the image).
  std::vector<cv::Point2f> unrectifyLeft(const std::vector<cv::Point2f>& rectified) const;
  /// Rectified right pixels → raw right pixels.
  std::vector<cv::Point2f> unrectifyRight(const std::vector<cv::Point2f>& rectified) const;

  // --- stereo geometry ---------------------------------------------------------------------------

  /// Triangulates a rectified stereo observation into the rectified left camera frame L'.
  /// Returns nullopt for disparities below `min_disparity_px` (too far / mismatched).
  std::optional<gtsam::Point3> triangulate(const gtsam::StereoPoint2& z,
                                           double min_disparity_px = 0.5) const;

  // --- whole images (visualization / debugging) --------------------------------------------------

  cv::Mat rectifyLeftImage(const cv::Mat& raw) const;
  cv::Mat rectifyRightImage(const cv::Mat& raw) const;

  /// Raw OpenCV calibration of one camera (K 3x3, D 1xN) — for the front-end.
  const cv::Mat& rawK(bool left) const { return left ? K_left_ : K_right_; }
  const cv::Mat& rawD(bool left) const { return left ? D_left_ : D_right_; }
  bool isFisheye() const { return fisheye_; }

 private:
  std::vector<cv::Point2f> undistortToRectified(const std::vector<cv::Point2f>& raw,
                                                bool left) const;
  std::vector<cv::Point2f> distortFromRectified(const std::vector<cv::Point2f>& rect,
                                                bool left) const;

  bool fisheye_ = false;
  cv::Size size_;
  cv::Mat K_left_, D_left_, K_right_, D_right_;  // raw calibration (CV_64F)
  cv::Mat R1_, R2_, P1_, P2_;                    // rectification (CV_64F)
  cv::Mat map_left_x_, map_left_y_, map_right_x_, map_right_y_;
  gtsam::Cal3_S2Stereo::shared_ptr calibration_;
  gtsam::Pose3 T_B_Lrect_;
};

}  // namespace kestrel
