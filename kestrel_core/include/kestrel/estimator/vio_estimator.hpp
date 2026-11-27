// SPDX-License-Identifier: Apache-2.0
//
// kestrel/estimator/vio_estimator.hpp — sliding-window visual-inertial estimation (via GTSAM).
//
// TUTORIAL NOTE
// -------------
// Theory: docs/theory/05_nonlinear_least_squares.md and 06_sliding_window_vio.md.
// References: [Forster17] (IMU factors), [Kaess12] (iSAM2), [Qin18], [Leutenegger15] (windowed
// VIO).
//
// The estimator answers: "given everything seen in the last few seconds, what is the body's
// pose, velocity and IMU bias right now?" It is a factor graph over KEYFRAMES (≈ 10 Hz):
//
//      prior ── X0,V0,B0 ══IMU══ X1,V1,B1 ══IMU══ X2,V2,B2 ══ ...      (X pose, V velocity, B bias)
//                 │  ╲             │   ╲            │
//                 L7  L8 ...       L8  L9 ...       L9 ...             (L landmarks = 3D points)
//
//   * CombinedImuFactor (GTSAM) links consecutive keyframes using the preintegrated IMU motion
//     and models the bias random walk.
//   * GenericStereoFactor (GTSAM) links a keyframe to a 3D landmark through a rectified stereo
//     observation (uL, uR, v). When only the left image saw the point, a mono
//     GenericProjectionFactor is used instead.
//   * Observation noise is wrapped in a Huber robust kernel, so a few bad matches cannot pull
//     the solution far (theory ch. 05, robust estimation).
//
// Solving: GTSAM's IncrementalFixedLagSmoother (iSAM2 underneath) keeps only variables whose
// timestamp is within `smoother_lag_s` of the newest keyframe and MARGINALIZES older ones —
// i.e. folds their information into a prior on the remaining variables instead of throwing it
// away. That is the Schur-complement marginalization VINS-style systems implement by hand;
// here it is done by the library. Kestrel's job is the policy:
//
//   * which frames become keyframes (time since the last one, or too few tracked landmarks),
//   * how landmarks enter the graph (only with a valid stereo depth) and when they leave
//     (their timestamp is refreshed on every observation, so a landmark is marginalized only
//     after it has not been seen for a full window),
//   * initialization (static start or an explicitly given state),
//   * outlier reporting back to the front-end, and recovery when the solver fails,
//   * dynamic objects (M4, theory ch. 07): new landmarks are confirmed only after enough time
//     for a mover to give itself away; a flagged landmark loses not only its future
//     observations but also the factors it contributed EARLIER in the window (GTSAM's
//     factorsToRemove); and a solve that would flag most of what is seen is distrusted.
//
// Between keyframes, the state is predicted with the IMU alone (cheap and smooth).

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "kestrel/common/gtsam_compat.hpp"  // IncrementalFixedLagSmoother
#include "kestrel/config.hpp"
#include "kestrel/frontend/feature_tracker.hpp"
#include "kestrel/imu/imu.hpp"
#include "kestrel/sensors/stereo_rig.hpp"

namespace kestrel {

class VioEstimator {
 public:
  /// Result for one processed frame.
  struct Output {
    double t = 0.0;
    bool initialized = false;  ///< false until the first keyframe has been created
    bool is_keyframe = false;  ///< this frame was added to the graph
    bool reset = false;        ///< the solver failed and the estimator restarted
    gtsam::NavState state;     ///< world ← body pose and world-frame velocity
    gtsam::imuBias::ConstantBias bias;
    int keyframe_index = -1;       ///< index of the latest keyframe
    int landmarks_in_window = 0;   ///< landmarks currently in the smoother
    int new_landmarks = 0;         ///< landmarks created at this keyframe
    int visual_factors = 0;        ///< stereo + mono factors added at this keyframe
    std::vector<int> outlier_ids;  ///< track ids the front-end should drop
    std::vector<int> new_landmark_ids;  ///< track ids that became landmarks at this keyframe
    int removed_factors = 0;  ///< factors of flagged landmarks removed from the window
    double solve_ms = 0.0;
  };

  VioEstimator(const Config& config, std::shared_ptr<const StereoRig> rig);

  /// Starts from a known state (simulation, tests, or re-initialization after a reset).
  void initialize(double t, const gtsam::NavState& state, const gtsam::imuBias::ConstantBias& bias);

  /// Starts assuming the body is (nearly) still: roll/pitch from the average accelerometer
  /// reading (it measures −gravity), gyro bias from the average gyro reading, yaw = 0, position
  /// = 0. Uses IMU samples in [t − init_static_window_s, t]. Returns false if there are too few.
  bool initializeStatic(double t, const std::vector<ImuSample>& imu);

  bool initialized() const { return pending_init_.has_value() || num_keyframes_ > 0; }
  bool hasKeyframe() const { return num_keyframes_ > 0; }

  /// Processes one tracked frame. `imu` must contain the samples covering the interval from the
  /// previous keyframe (or the initialization time) to frame.t. If the estimator has not been
  /// initialized yet, a static initialization is attempted first.
  Output process(const FeatureFrame& frame, const std::vector<ImuSample>& imu);

  /// Latest keyframe state and bias (valid once a keyframe exists).
  const gtsam::NavState& keyframeState() const { return kf_state_; }
  const gtsam::imuBias::ConstantBias& bias() const { return kf_bias_; }
  double keyframeTime() const { return kf_time_; }

  /// Current estimates of all landmarks in the window, as (track id, world position). Computes
  /// the full smoother estimate, so call it at keyframes and only when needed (visualization).
  std::vector<std::pair<int, gtsam::Point3>> landmarksInWindow() const;

  /// Snapshot of the factor graph in the window, for visualization: keyframe poses, landmarks,
  /// which landmarks the newest keyframe observes, and how many factors of each kind exist.
  /// Factors are classified by the variables they connect (no type casts): IMU factors link
  /// pose/velocity/bias of two keyframes, visual factors a pose and a landmark.
  struct WindowGraph {
    std::vector<std::pair<int, gtsam::Pose3>> keyframes;   ///< (keyframe index, world pose)
    std::vector<std::pair<int, gtsam::Point3>> landmarks;  ///< (track id, world position)
    std::vector<int> newest_observations;                  ///< landmark ids seen by the newest
    int imu_factors = 0, visual_factors = 0, prior_factors = 0, other_factors = 0;
  };
  WindowGraph windowGraph() const;

  /// Drops everything; the next process() call initializes again.
  void reset();

 private:
  void createFirstKeyframe(const FeatureFrame& frame, const std::vector<ImuSample>& imu,
                           Output* out);
  void addKeyframe(const FeatureFrame& frame, const gtsam::PreintegratedCombinedMeasurements& pim,
                   const gtsam::NavState& predicted, Output* out);
  /// Adds visual factors / new landmarks for keyframe `k`; returns the factors' landmark ids.
  void addVisualFactors(const FeatureFrame& frame, int k, const gtsam::Pose3& T_w_b,
                        gtsam::NonlinearFactorGraph* graph, gtsam::Values* values,
                        gtsam::FixedLagSmoother::KeyTimestampMap* stamps, Output* out);
  bool solve(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
             const gtsam::FixedLagSmoother::KeyTimestampMap& stamps, Output* out);
  /// Post-solve check of the latest keyframe's observations; returns how many were flagged.
  int collectOutliers(int k, Output* out);
  /// Marks a landmark as outlier: no further factors, reported to the front-end, and (M4) its
  /// factors already in the window are queued for removal with the next update.
  void flagLandmark(int id, Output* out);
  void readKeyframeState(int k);
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> makeSmoother() const;

  Config config_;
  std::shared_ptr<const StereoRig> rig_;
  ImuParamsPtr imu_params_;
  gtsam::Cal3_S2::shared_ptr mono_calibration_;
  gtsam::SharedNoiseModel stereo_noise_, mono_noise_, landmark_prior_noise_;

  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  struct PendingInit {
    double t;
    gtsam::NavState state;
    gtsam::imuBias::ConstantBias bias;
  };
  std::optional<PendingInit> pending_init_;

  int num_keyframes_ = 0;  ///< also the index of the next keyframe
  double kf_time_ = 0.0;
  gtsam::NavState kf_state_;
  gtsam::imuBias::ConstantBias kf_bias_;

  /// Observations added at the latest keyframe, kept for the post-solve outlier check.
  struct Observation {
    int id;
    bool stereo;
    gtsam::StereoPoint2 z;  ///< valid if stereo
    gtsam::Point2 uv;       ///< left pixel (mono observations)
  };
  std::vector<Observation> last_observations_;
  /// First stereo observation of a not-yet-confirmed landmark (see addVisualFactors()).
  struct Candidate {
    int keyframe;
    double t;  ///< time of that keyframe
    gtsam::StereoPoint2 z;
    gtsam::Point3 p_w;
  };
  std::unordered_map<int, Candidate> candidates_;
  /// Landmarks known to be outliers: never receive new factors.
  std::unordered_map<int, bool> rejected_;
  /// Where each landmark's visual factors live in the smoother (iSAM2 factor slots), so they
  /// can be removed if it turns out to be on a moving object. Slots are re-checked before use:
  /// marginalization frees slots, and iSAM2 reuses freed slots for new factors.
  std::unordered_map<int, std::vector<std::size_t>> landmark_factors_;
  gtsam::FactorIndices pending_removals_;
};

}  // namespace kestrel
