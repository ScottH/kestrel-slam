// SPDX-License-Identifier: Apache-2.0
//
// Smoke test for the GTSAM dependency.
//
// Why: GTSAM and our code must agree on Eigen's version and SIMD alignment settings. A mismatch
// does not fail at link time — it crashes or corrupts memory at run time, typically when
// fixed-size Eigen members (inside Pose3, NavState, ...) are copied into containers. This test
// exercises exactly those paths (containers of GTSAM types, an incremental iSAM2 problem) so a
// broken toolchain is caught here instead of deep inside the estimator.

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <map>
#include <vector>

namespace {

using gtsam::symbol_shorthand::X;

TEST(GtsamSmoke, ContainersOfFixedSizeTypes) {
  std::vector<gtsam::Pose3> poses;
  std::map<int, gtsam::NavState> states;
  for (int i = 0; i < 1000; ++i) {
    poses.emplace_back(gtsam::Rot3::Rz(0.001 * i), gtsam::Point3(i, 0, 0));
    states[i] = gtsam::NavState(poses.back(), gtsam::Vector3(1, 2, 3));
  }
  EXPECT_NEAR(poses[999].translation().x(), 999.0, 1e-12);
  EXPECT_NEAR(states[500].v().z(), 3.0, 1e-12);
}

TEST(GtsamSmoke, IncrementalPoseGraphWithIsam2) {
  // A square loop of 4 poses, each odometry edge slightly wrong, plus a loop closure.
  gtsam::ISAM2 isam;
  const auto odo_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
  const gtsam::Pose3 step(gtsam::Rot3::Rz(M_PI / 2), gtsam::Point3(1.0, 0, 0));
  const gtsam::Pose3 noisy_step(gtsam::Rot3::Rz(M_PI / 2 + 0.05), gtsam::Point3(1.1, 0, 0));

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  graph.addPrior(X(0), gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
  initial.insert(X(0), gtsam::Pose3());
  isam.update(graph, initial);

  gtsam::Pose3 guess;
  for (int i = 1; i < 4; ++i) {
    graph = gtsam::NonlinearFactorGraph();
    initial.clear();
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), noisy_step, odo_noise);
    guess = guess * noisy_step;
    initial.insert(X(i), guess);
    isam.update(graph, initial);
  }
  // Loop closure X3 → X0 with the true relative motion.
  graph = gtsam::NonlinearFactorGraph();
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(3), X(0), step, odo_noise);
  isam.update(graph, gtsam::Values());
  isam.update();

  const gtsam::Values estimate = isam.calculateEstimate();
  ASSERT_EQ(estimate.size(), 4u);
  // After closing the loop, the composed loop must be (nearly) consistent again.
  const gtsam::Pose3 loop = estimate.at<gtsam::Pose3>(X(3)) * step;
  EXPECT_LT(gtsam::Pose3::Logmap(loop).norm(), 0.3);
}

}  // namespace
