// SPDX-License-Identifier: Apache-2.0
//
// kestrel/eval/dynamic_overlay.hpp — paints a moving object into real stereo images, for
// testing dynamic-object handling on datasets that have none (EuRoC, TUM-VI).
//
// TUTORIAL NOTE
// -------------
// Recording people and forklifts (roadmap D8/D9) gives the real test. Until then, and as a
// controlled, repeatable check, this helper overlays a large textured panel that crosses the
// view at a fixed distance. "Large object crossing the view" is the hard regime of theory ch. 07
// §1: while it covers a big part of the image, it competes with the static scene for the
// majority of the features.
//
// To be a fair test, the object must look physically real to the STEREO front-end, not only to
// the left camera. It is therefore drawn in RECTIFIED coordinates, where a point at depth Z
// appears in the right image shifted left by the disparity d = f·b/Z, with the same row. Each raw
// (distorted) pixel is then looked up through its rectified position, so lens distortion, which is
// strong on TUM-VI's fisheye lenses, is handled exactly. The tracker sees a planar object with
// correct stereo depth, moving sideways in front of the camera, inconsistent with the
// ego-motion that the IMU reports.
//
// The painted pixels are returned as a mask. Feeding that mask back as the external mask gives
// the "oracle segmentation" upper bound in the ablation (tools/dynamic_ablation.py).

#pragma once

#include <opencv2/core.hpp>

#include "kestrel/sensors/stereo_rig.hpp"

namespace kestrel {

struct DynamicOverlayOptions {
  double depth_m = 1.5;          ///< object distance from the camera (sets the disparity)
  double width_fraction = 0.4;   ///< object width as a fraction of the image width
  double height_fraction = 0.6;  ///< ... and height
  double start_s = 5.0;          ///< first crossing starts this long after the first frame
  double period_s = 10.0;        ///< a new crossing starts every period
  double crossing_s = 4.0;       ///< time to cross the full image width
  unsigned seed = 7;             ///< texture
};

class DynamicOverlay {
 public:
  DynamicOverlay(const StereoRig& rig, int width, int height,
                 const DynamicOverlayOptions& options = DynamicOverlayOptions());

  /// Paints the object into the raw left/right images (8-bit) at time t, in seconds since the
  /// first frame. Returns the raw-left mask of painted pixels (255), or an empty Mat if the
  /// object is not in view at t.
  cv::Mat apply(double t, cv::Mat* left, cv::Mat* right) const;

  /// Left edge of the object in rectified pixels at time t, if it is crossing.
  bool position(double t, double* x0) const;
  double disparityPx() const { return disparity_; }

 private:
  DynamicOverlayOptions options_;
  int width_, height_;
  double disparity_;
  cv::Mat texture_;           ///< object appearance, CV_8U, size in rectified pixels
  cv::Mat map_left_, map_right_;  ///< raw pixel → rectified position (CV_32FC2)
};

}  // namespace kestrel
