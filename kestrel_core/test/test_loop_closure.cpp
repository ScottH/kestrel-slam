// SPDX-License-Identifier: Apache-2.0
//
// Tests for place recognition, geometric loop verification and the pose graph.
//
// Images are synthesized with exact geometry: a textured plane seen by the nominal (ideal,
// rectified) stereo rig. A camera C seeing the plane n·X = d of a reference camera A images it
// through the plane-induced homography  H = K (R_CA + t_CA nᵀ / d) K⁻¹  (theory ch. 02), so the
// true relative pose of any rendered view is known exactly.

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

#include "kestrel/config.hpp"
#include "kestrel/loop/loop_closer.hpp"
#include "kestrel/loop/place_recognizer.hpp"

namespace kestrel {
namespace {

namespace fs = std::filesystem;

/// A texture with plenty of ORB corners: random filled rectangles and circles.
cv::Mat makeScene(int w, int h, unsigned seed) {
  cv::Mat img(h, w, CV_8UC1, cv::Scalar(128));
  cv::RNG rng(seed);
  for (int i = 0; i < 400; ++i) {
    const cv::Point c(rng.uniform(0, w), rng.uniform(0, h));
    const int s = rng.uniform(4, 30);
    const cv::Scalar col(rng.uniform(0, 255));
    if (i % 2) {
      cv::rectangle(img, c, c + cv::Point(s, rng.uniform(4, 30)), col, cv::FILLED);
    } else {
      cv::circle(img, c, s / 2, col, cv::FILLED);
    }
  }
  cv::GaussianBlur(img, img, cv::Size(3, 3), 0);
  return img;
}

/// Renders what camera C sees, given the texture as seen by camera A (plane z = depth in A).
cv::Mat render(const cv::Mat& texture_A, const StereoRig& rig, const gtsam::Pose3& T_A_C,
               double depth) {
  const auto& K = *rig.calibration();
  const cv::Matx33d Km(K.fx(), 0, K.px(), 0, K.fy(), K.py(), 0, 0, 1);
  const gtsam::Pose3 T_C_A = T_A_C.inverse();
  const gtsam::Matrix3 R = T_C_A.rotation().matrix();
  const gtsam::Vector3 t = T_C_A.translation();
  cv::Matx33d M;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) M(r, c) = R(r, c) + (c == 2 ? t(r) / depth : 0.0);
  }
  const cv::Matx33d H = Km * M * Km.inv();
  cv::Mat out;
  cv::warpPerspective(texture_A, out, cv::Mat(H), texture_A.size(), cv::INTER_LINEAR,
                      cv::BORDER_CONSTANT, cv::Scalar(128));
  return out;
}

struct StereoView {
  cv::Mat left, right;
};

/// Left/right images of the plane for a rig whose left camera sits at T_A_L.
StereoView view(const cv::Mat& texture, const StereoRig& rig, const gtsam::Pose3& T_A_L,
                double depth) {
  const gtsam::Pose3 T_L_R(gtsam::Rot3(), gtsam::Point3(rig.baseline(), 0, 0));
  return {render(texture, rig, T_A_L, depth), render(texture, rig, T_A_L * T_L_R, depth)};
}

class LoopClosureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rig_ = std::make_shared<StereoRig>(config_.camera);
    config_.loop_closure.synchronous = true;
    w_ = config_.camera.width;
    h_ = config_.camera.height;
  }
  Config config_;
  std::shared_ptr<StereoRig> rig_;
  int w_ = 0, h_ = 0;
};

TEST_F(LoopClosureTest, PlaceRecognizerFindsTheRightPlace) {
  // Vocabulary from a handful of scenes, database of 10 places, query = shifted view of place 3.
  auto orb = cv::ORB::create(500);
  std::vector<cv::Mat> training, place_desc;
  for (unsigned s = 0; s < 10; ++s) {
    std::vector<cv::KeyPoint> kp;
    cv::Mat d;
    orb->detectAndCompute(makeScene(w_, h_, 100 + s), cv::noArray(), kp, d);
    training.push_back(d);
    place_desc.push_back(d);
  }
  const fs::path voc = fs::temp_directory_path() / ("kestrel_voc_" + std::to_string(::getpid()) + ".fbow");
  trainFbowVocabulary(training, 8, 3, voc.string(), 2);
  auto recognizer = makeFbowPlaceRecognizer(voc.string());
  for (int i = 0; i < 10; ++i) recognizer->add(i, place_desc[i]);
  ASSERT_EQ(recognizer->size(), 10u);

  const cv::Mat shifted = render(makeScene(w_, h_, 103), *rig_,
                                 gtsam::Pose3(gtsam::Rot3::Ry(0.03), gtsam::Point3(0.1, 0, 0)), 3.0);
  std::vector<cv::KeyPoint> kp;
  cv::Mat q;
  orb->detectAndCompute(shifted, cv::noArray(), kp, q);
  const auto result = recognizer->query(q, 3, [](int id) { return id == 7; });
  ASSERT_FALSE(result.empty());
  EXPECT_EQ(result.front().id, 3);
  EXPECT_GT(result.front().score, 1.2 * result[1].score) << "the right place should stand out";
  for (const auto& c : result) EXPECT_NE(c.id, 7) << "excluded ids must not be returned";
  fs::remove(voc);
}

TEST_F(LoopClosureTest, VerificationRecoversRelativePose) {
  LoopCloser closer(config_.loop_closure, rig_, nullptr);
  const cv::Mat texture = makeScene(w_, h_, 7);
  const double depth = 3.0;
  const gtsam::Pose3 T_A_B(gtsam::Rot3::RzRyRx(0.01, -0.05, 0.02), gtsam::Point3(0.25, -0.05, 0.3));

  const StereoView a = view(texture, *rig_, gtsam::Pose3(), depth);
  const StereoView b = view(texture, *rig_, T_A_B, depth);
  const KeyframeFeatures fa = closer.describe(a.left, a.right);
  const KeyframeFeatures fb = closer.describe(b.left, b.right);
  ASSERT_GT(fa.with_depth, 100) << "stereo depth for ORB features";

  gtsam::Pose3 T_est;
  const int inliers = closer.verify(fa, fb, &T_est);
  ASSERT_GE(inliers, config_.loop_closure.min_inliers);
  const double t_err = (T_est.translation() - T_A_B.translation()).norm();
  const double r_err = gtsam::Rot3::Logmap(T_est.rotation().between(T_A_B.rotation())).norm();
  std::printf("[verify] %d inliers, translation error %.4f m, rotation error %.3f deg\n", inliers,
              t_err, r_err * 180 / M_PI);
  EXPECT_LT(t_err, 0.02);
  EXPECT_LT(r_err * 180 / M_PI, 0.5);

  // A different place must not verify.
  const StereoView other = view(makeScene(w_, h_, 99), *rig_, gtsam::Pose3(), depth);
  EXPECT_EQ(closer.verify(closer.describe(other.left, other.right), fb, &T_est), 0);
}

TEST_F(LoopClosureTest, PoseGraphRemovesInjectedDrift) {
  // Truth: 40 keyframes around a 4 m × 4 m square, ending where it started. Odometry: each step
  // has a small systematic yaw error (0.4°) — classic drift that makes the square fail to close.
  LoopCloser closer(config_.loop_closure, rig_, nullptr);
  std::vector<gtsam::Pose3> truth;
  for (int i = 0; i <= 40; ++i) {
    const int side = std::min(i / 10, 3), s = i - 10 * side;
    const double step = 0.4;
    gtsam::Point3 p;
    switch (side) {
      case 0: p = gtsam::Point3(s * step, 0, 0); break;
      case 1: p = gtsam::Point3(4, s * step, 0); break;
      case 2: p = gtsam::Point3(4 - s * step, 4, 0); break;
      default: p = gtsam::Point3(0, 4 - s * step, 0); break;
    }
    if (i == 40) p = gtsam::Point3(0, 0, 0);
    truth.emplace_back(gtsam::Rot3::Rz(M_PI / 2 * std::min(i / 10, 4)), p);
  }
  gtsam::Pose3 odom = truth[0];
  const gtsam::Pose3 drift(gtsam::Rot3::Rz(0.4 * M_PI / 180), gtsam::Point3(0.005, 0, 0));
  for (int i = 0; i <= 40; ++i) {
    if (i > 0) odom = odom * truth[i - 1].between(truth[i]) * drift;
    closer.addKeyframe({i, static_cast<double>(i), odom, cv::Mat(), cv::Mat()});
  }
  const double error_before = (odom.translation() - truth[40].translation()).norm();

  // The loop: keyframe 40 is back at keyframe 0's pose (relative pose = identity).
  closer.addLoopConstraint(0, 40, truth[0].between(truth[40]));
  const auto poses = closer.keyframePoses();
  const gtsam::Pose3 corrected = std::get<2>(poses.back());
  const double error_after = (corrected.translation() - truth[40].translation()).norm();
  std::printf("[pose graph] end-point error %.3f m → %.3f m\n", error_before, error_after);
  EXPECT_GT(error_before, 0.5);
  EXPECT_LT(error_after, 0.1 * error_before);
  // The correction is what the odometry frame needs: T_map_odom · T_odom_last = T_map_last.
  EXPECT_TRUE((closer.mapFromOdom() * odom).equals(corrected, 1e-6));
}

}  // namespace
}  // namespace kestrel
