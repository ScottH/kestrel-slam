// SPDX-License-Identifier: Apache-2.0
//
// Loop closure: description, proposal, verification, pose graph. See kestrel/loop/loop_closer.hpp.

#include "kestrel/loop/loop_closer.hpp"

#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include "kestrel/common/logging.hpp"

namespace kestrel {
namespace {

gtsam::Key K(int id) { return gtsam::Symbol('k', static_cast<std::uint64_t>(id)); }

/// Descriptors farther apart than this (Hamming bits of 256) are never considered a match.
constexpr int kMaxHamming = 64;
/// Only one accepted loop per this many keyframes: consecutive keyframes re-detect the same loop.
constexpr int kMinKeyframesBetweenLoops = 5;

}  // namespace

LoopCloser::LoopCloser(const LoopClosureConfig& config, std::shared_ptr<const StereoRig> rig,
                       std::unique_ptr<PlaceRecognizer> recognizer)
    : config_(config), rig_(std::move(rig)), recognizer_(std::move(recognizer)) {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam_ = std::make_unique<gtsam::ISAM2>(params);
  if (!recognizer_) KLOG_WARN("loop closure: no vocabulary — loop detection disabled");
  if (!config_.synchronous) worker_ = std::thread([this] { workerLoop(); });
}

LoopCloser::~LoopCloser() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  queue_cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

// ------------------------------------------------------------------------------------------------
// Threading
// ------------------------------------------------------------------------------------------------

void LoopCloser::addKeyframe(KeyframeInput keyframe) {
  if (config_.synchronous) {
    process(std::move(keyframe));
    return;
  }
  // The caller may reuse its image buffers; keep our own copies.
  keyframe.left = keyframe.left.clone();
  keyframe.right = keyframe.right.clone();
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(std::move(keyframe));
  }
  queue_cv_.notify_one();
}

void LoopCloser::workerLoop() {
  for (;;) {
    KeyframeInput kf;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
      if (stop_ && queue_.empty()) return;
      kf = std::move(queue_.front());
      queue_.pop_front();
      busy_ = true;
    }
    process(std::move(kf));
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      busy_ = false;
    }
    idle_cv_.notify_all();
  }
}

void LoopCloser::waitUntilIdle() {
  if (config_.synchronous) return;
  std::unique_lock<std::mutex> lock(queue_mutex_);
  idle_cv_.wait(lock, [this] { return queue_.empty() && !busy_; });
}

gtsam::Pose3 LoopCloser::mapFromOdom() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return T_map_odom_;
}

std::vector<LoopEvent> LoopCloser::loops() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return loops_;
}

std::vector<std::tuple<int, double, gtsam::Pose3>> LoopCloser::keyframePoses() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::vector<std::tuple<int, double, gtsam::Pose3>> out;
  out.reserve(optimized_.size());
  for (const auto& [id, pose] : optimized_) out.emplace_back(id, times_.at(id), pose);
  return out;
}

// ------------------------------------------------------------------------------------------------
// 1. Description: ORB + stereo depth
// ------------------------------------------------------------------------------------------------

KeyframeFeatures LoopCloser::describe(const cv::Mat& left, const cv::Mat& right) const {
  KeyframeFeatures out;
  if (left.empty()) return out;  // e.g. pose-graph-only use (tests)
  auto orb = cv::ORB::create(config_.orb_features);
  std::vector<cv::KeyPoint> kl, kr;
  cv::Mat dl, dr;
  orb->detectAndCompute(left, cv::noArray(), kl, dl);
  if (kl.empty()) return out;
  if (!right.empty()) orb->detectAndCompute(right, cv::noArray(), kr, dr);

  std::vector<cv::Point2f> pl, pr;
  for (const auto& k : kl) pl.push_back(k.pt);
  for (const auto& k : kr) pr.push_back(k.pt);
  // ORB keypoints sit on the pixel grid of their pyramid level. Sub-pixel refinement matters:
  // with only ~14 px of disparity at 3 m, half a pixel is ~4 % of depth (theory ch. 02).
  const cv::TermCriteria subpix(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.01);
  cv::cornerSubPix(left, pl, cv::Size(3, 3), cv::Size(-1, -1), subpix);
  out.rectified = rig_->rectifyLeft(pl);
  out.descriptors = dl;
  out.points.assign(kl.size(), cv::Point3f(std::numeric_limits<float>::quiet_NaN(), 0, 0));
  if (kr.empty()) return out;
  const auto rr = rig_->rectifyRight(pr);

  // Rectified stereo: a match lies on (almost) the same row. Bucket right keypoints by row.
  std::map<int, std::vector<int>> rows;
  for (std::size_t j = 0; j < rr.size(); ++j) rows[static_cast<int>(std::lround(rr[j].y))].push_back(static_cast<int>(j));
  const double max_disparity = rig_->focalLength() * rig_->baseline() / 0.2;
  std::vector<int> match_left;            // left keypoint index of each stereo match
  std::vector<cv::Point2f> match_right;   // matched right keypoint (raw pixels)
  for (std::size_t i = 0; i < out.rectified.size(); ++i) {
    const cv::Point2f& p = out.rectified[i];
    int best = -1, best_d = kMaxHamming + 1, second_d = 256;
    const int row = static_cast<int>(std::lround(p.y));
    for (int r = row - 2; r <= row + 2; ++r) {
      const auto it = rows.find(r);
      if (it == rows.end()) continue;
      for (const int j : it->second) {
        const double disparity = p.x - rr[j].x;
        if (disparity <= 0.5 || disparity > max_disparity || std::abs(rr[j].y - p.y) > 2.0) continue;
        const int d = static_cast<int>(cv::norm(dl.row(static_cast<int>(i)), dr.row(j), cv::NORM_HAMMING));
        if (d < best_d) {
          second_d = best_d;
          best_d = d;
          best = j;
        } else if (d < second_d) {
          second_d = d;
        }
      }
    }
    if (best < 0 || best_d > 50 || best_d > config_.ratio_test * second_d) continue;
    match_left.push_back(static_cast<int>(i));
    match_right.push_back(pr[best]);
  }
  if (match_left.empty()) return out;

  // Refine each stereo match to sub-pixel accuracy with Lucas–Kanade, starting from the matched
  // right keypoint. A refinement that moves more than 2 px means the descriptor match was wrong.
  std::vector<cv::Point2f> from, to = match_right;
  for (const int i : match_left) from.push_back(pl[i]);
  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(left, right, from, to, status, err, cv::Size(11, 11), 1, subpix,
                           cv::OPTFLOW_USE_INITIAL_FLOW);
  const auto to_rect = rig_->rectifyRight(to);
  for (std::size_t m = 0; m < match_left.size(); ++m) {
    if (!status[m] || cv::norm(to[m] - match_right[m]) > 2.0) continue;
    const cv::Point2f& p = out.rectified[match_left[m]];
    if (std::abs(to_rect[m].y - p.y) > 1.0) continue;  // must stay on the epipolar row
    const auto X = rig_->triangulate(gtsam::StereoPoint2(p.x, to_rect[m].x, 0.5 * (p.y + to_rect[m].y)));
    if (!X || X->z() < 0.2 || X->z() > 20.0) continue;
    out.points[match_left[m]] =
        cv::Point3f(static_cast<float>(X->x()), static_cast<float>(X->y()), static_cast<float>(X->z()));
    ++out.with_depth;
  }
  return out;
}

// ------------------------------------------------------------------------------------------------
// 4. Geometric verification: descriptor matching + PnP-RANSAC
// ------------------------------------------------------------------------------------------------

int LoopCloser::verify(const KeyframeFeatures& match, const KeyframeFeatures& query,
                       gtsam::Pose3* T_matchcam_querycam) const {
  if (match.with_depth < config_.min_inliers || query.descriptors.empty()) return 0;
  // Candidate side: only keypoints with a 3D position can anchor PnP.
  std::vector<int> index;
  cv::Mat train;
  for (std::size_t i = 0; i < match.points.size(); ++i) {
    if (std::isnan(match.points[i].x)) continue;
    index.push_back(static_cast<int>(i));
    train.push_back(match.descriptors.row(static_cast<int>(i)));
  }
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn;
  matcher.knnMatch(query.descriptors, train, knn, 2);
  std::vector<cv::Point3f> object;
  std::vector<cv::Point2f> image;
  for (const auto& m : knn) {
    if (m.empty() || m[0].distance > kMaxHamming) continue;
    if (m.size() > 1 && m[0].distance > config_.ratio_test * m[1].distance) continue;
    object.push_back(match.points[index[m[0].trainIdx]]);
    image.push_back(query.rectified[m[0].queryIdx]);
  }
  if (static_cast<int>(object.size()) < config_.min_inliers) return 0;

  const auto& Kc = *rig_->calibration();
  const cv::Mat Kmat = (cv::Mat_<double>(3, 3) << Kc.fx(), 0, Kc.px(), 0, Kc.fy(), Kc.py(), 0, 0, 1);
  cv::Mat rvec, tvec;
  std::vector<int> inliers;
  const bool ok = cv::solvePnPRansac(object, image, Kmat, cv::noArray(), rvec, tvec, false, 200,
                                     static_cast<float>(config_.pnp_reprojection_px), 0.99, inliers,
                                     cv::SOLVEPNP_EPNP);
  if (!ok || static_cast<int>(inliers.size()) < config_.min_inliers) return 0;
  // Refine on the inliers (Levenberg–Marquardt from the EPnP solution).
  std::vector<cv::Point3f> obj_in;
  std::vector<cv::Point2f> img_in;
  for (const int i : inliers) {
    obj_in.push_back(object[i]);
    img_in.push_back(image[i]);
  }
  cv::solvePnP(obj_in, img_in, Kmat, cv::noArray(), rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);

  // OpenCV returns the transform taking match-camera points into the query camera.
  cv::Mat Rcv;
  cv::Rodrigues(rvec, Rcv);
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(Rcv, R);
  cv::cv2eigen(tvec, t);
  const gtsam::Pose3 T_query_match{gtsam::Rot3(R), gtsam::Point3(t)};
  *T_matchcam_querycam = T_query_match.inverse();
  return static_cast<int>(inliers.size());
}

// ------------------------------------------------------------------------------------------------
// 5. Pose graph
// ------------------------------------------------------------------------------------------------

void LoopCloser::addOdometryNode(const Record& r) {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  if (last_id_ < 0) {
    // The first keyframe anchors the map frame (= odometry frame at the start).
    graph.addPrior(K(r.id), r.T_odom_body, gtsam::noiseModel::Isotropic::Sigma(6, 1e-6));
    values.insert(K(r.id), r.T_odom_body);
  } else {
    const Record& prev = records_.at(last_id_);
    const gtsam::Pose3 relative = prev.T_odom_body.between(r.T_odom_body);
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << gtsam::Vector3::Constant(config_.odometry_sigma_rot),
         gtsam::Vector3::Constant(config_.odometry_sigma_trans))
            .finished());
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(K(last_id_), K(r.id), relative, noise);
    // Initial guess: chain the relative motion onto the previous OPTIMIZED pose.
    gtsam::Pose3 prev_map;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      prev_map = optimized_.at(last_id_);
    }
    values.insert(K(r.id), prev_map * relative);
  }
  isam_->update(graph, values);
}

void LoopCloser::addLoopConstraint(int match_id, int query_id, const gtsam::Pose3& T_match_query) {
  gtsam::NonlinearFactorGraph graph;
  // Plain Gaussian on purpose. A robust kernel (Cauchy/Huber) here would treat a GENUINE loop
  // after a long drift — residual of many sigmas — as an outlier and ignore it: exactly the loops
  // that matter most (found by test_loop_closure: 0.77 m of drift stayed uncorrected). Protection
  // against false loops comes from geometric verification and temporal consistency instead.
  const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << gtsam::Vector3::Constant(config_.loop_sigma_rot),
       gtsam::Vector3::Constant(config_.loop_sigma_trans))
          .finished());
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(K(match_id), K(query_id), T_match_query,
                                                           noise);
  isam_->update(graph, gtsam::Values());
  optimize(5);  // a loop can move many poses: let iSAM2 relinearize a few times
}

void LoopCloser::optimize(int iterations) {
  for (int i = 0; i < iterations; ++i) isam_->update();
  const gtsam::Values estimate = isam_->calculateEstimate();
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto& [id, record] : records_) {
    if (estimate.exists(K(id))) optimized_[id] = estimate.at<gtsam::Pose3>(K(id));
  }
  if (last_id_ >= 0) {
    T_map_odom_ = optimized_.at(last_id_) * records_.at(last_id_).T_odom_body.inverse();
  }
}

// ------------------------------------------------------------------------------------------------
// Per-keyframe pipeline (steps 1–5)
// ------------------------------------------------------------------------------------------------

void LoopCloser::process(KeyframeInput kf) {
  Record record{kf.id, kf.t, kf.T_odom_body, describe(kf.left, kf.right)};
  addOdometryNode(record);
  records_.emplace(kf.id, record);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    times_[kf.id] = kf.t;
  }
  const int previous_id = last_id_;
  last_id_ = kf.id;
  optimize(0);

  if (!recognizer_ || record.features.descriptors.empty()) return;
  recognizer_->add(kf.id, record.features.descriptors);
  if (previous_id < 0) return;

  // 2. Propose: best non-recent keyframe, score normalized by the query's own neighbour.
  const double recent_cut = kf.t - config_.exclude_recent_s;
  const auto candidates = recognizer_->query(
      record.features.descriptors, 1,
      [this, recent_cut](int id) { return records_.at(id).t > recent_cut; });
  const double neighbour = recognizer_->score(kf.id, previous_id);
  if (candidates.empty() || neighbour < 0.01) {
    consistent_count_ = 0;
    return;
  }
  const PlaceCandidate& best = candidates.front();
  const double normalized = best.score / neighbour;
  if (normalized < config_.min_bow_score) {
    consistent_count_ = 0;
    last_candidate_ = -1;
    return;
  }
  // 3. Temporal consistency: the same place (± a few keyframes) for consecutive queries.
  consistent_count_ =
      (last_candidate_ >= 0 && std::abs(best.id - last_candidate_) <= 5) ? consistent_count_ + 1 : 1;
  last_candidate_ = best.id;
  if (consistent_count_ < config_.temporal_consistency) return;
  if (!loops_.empty() && kf.id - loops_.back().query_id < kMinKeyframesBetweenLoops) return;

  // 4. Verify geometrically.
  const Record& match = records_.at(best.id);
  gtsam::Pose3 T_mc_qc;
  const int inliers = verify(match.features, record.features, &T_mc_qc);
  if (inliers < config_.min_inliers) {
    KLOG_DEBUG("loop candidate " << best.id << " → " << kf.id << " rejected (" << inliers
                                 << " PnP inliers)");
    return;
  }
  // Camera-to-camera → body-to-body: T_mb_qb = T_B_C · T_mc_qc · T_C_B.
  const gtsam::Pose3& T_B_C = rig_->bodyToRectifiedLeft();
  const gtsam::Pose3 T_mb_qb = T_B_C * T_mc_qc * T_B_C.inverse();

  // 5. Pose graph.
  addLoopConstraint(best.id, kf.id, T_mb_qb);
  LoopEvent event{kf.id, best.id, kf.t, match.t, T_mb_qb, inliers, normalized};
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    loops_.push_back(event);
  }
  KLOG_INFO("loop closed: keyframe " << kf.id << " (t=" << kf.t << ") ↔ " << best.id << " (t="
                                     << match.t << "), " << inliers << " inliers, score "
                                     << normalized << ", correction "
                                     << T_map_odom_.translation().norm() << " m");
  consistent_count_ = 0;
}

}  // namespace kestrel
