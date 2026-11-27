# Kestrel SLAM — Roadmap & Checklist

This is the **working checklist** for taking Kestrel from scaffolding to a solid, field-tested
SLAM system. [DEV_PLAN.md](DEV_PLAN.md) explains *why* each piece exists and how it is designed;
this file tracks *what is done* and *what is next*.

**How to use it**

* Tick boxes (`- [x]`) in the same PR that delivers the item, so the checklist always matches `main`.
* Every milestone has four lists:
  * **Build**: features to implement.
  * **Verify**: automated tests that must pass in `docker compose run --rm test`.
  * **Data**: recordings it needs.
  * **Exit**: the measurable bar for calling it done.
* Data IDs (**D1**, **D2**, …) refer to [§ Real-world data collection](#real-world-data-collection).
  Field-test IDs (**F1**, …) refer to [§ Field testing](#field-testing).
* Numbers in **Exit** criteria are starting targets. Adjust them in a PR, with the evidence, if
  data shows they are unrealistic.

## Status at a glance

| Milestone | Scope | Status |
|-----------|-------|--------|
| M0 | Scaffolding, Docker, capture tool, theory docs | ✅ merged (PR #2) |
| M1 | Sensor models on GTSAM + OpenCV (IMU, stereo rectification) | ✅ merged (PR #3) |
| M2 | Stereo front-end | ✅ merged (PR #4); real-data exit check pending |
| M3 | Sliding-window VIO (GTSAM fixed-lag smoother) | ✅ merged (PR #5) |
| M4 | Dynamic-object robustness | ⬜ |
| M5 | Loop closure (FBoW + PnP + iSAM2 pose graph) | ✅ merged (PR #7); precision follow-ups open |
| M6 | Multi-session maps & relocalization | ⬜ |
| M7 | Lifelong mapping (persistence filters) | ⬜ |
| M8 | Voxel map & change detection | ⬜ |
| M9 | ROS 2 node & Nav2 integration | ⬜ |
| M10 | Benchmarking, tuning, Jetson performance | ⬜ |
| M11 | Tutorial chapters per module | ⬜ (theory chapters done) |
| M12 | Long-term operation in changing spaces: height-band filter, exclusion zones | ⬜ |
| v1.0 | [Definition of "solid"](#definition-of-solid-v10-release-criteria) met | ⬜ |

**Recommended order**:
M1 → M3 (synthetic) → M2 → M3 (EuRoC) → M3 (your data) → M5 → M4 → M6 → M7 → M8 → M12 → M9 → M10.
Start data collection (**D1–D4**) now, in parallel with M1. Calibration is the long pole for
running on your own camera.

---

## M0 — Scaffolding, Docker, capture tool

**Build**
- [x] Multi-stage Dockerfile (base → orbbec driver → dev → builder → runtime)
- [x] `compose.yaml` services: build, test, shell, capture, foxglove
- [x] Entrypoint: USB access for Orbbec devices, drop to host uid
- [x] `kestrel_core`: typed YAML config + validation, logging facade
- [x] `kestrel_interfaces`: SaveMap / LoadMap / ResetMap
- [x] `kestrel_ros`: component node stub (config + log bridge)
- [x] `kestrel_capture`: rosbag recorder (MCAP/zstd) + live quality monitor
- [x] Theory chapters (`docs/theory/`) and `docs/REFERENCES.md`
- [x] CI workflow, devcontainer, clang-format, Apache-2.0

**Verify**
- [x] 11 C++ + 3 Python tests pass locally in Docker
- [x] Clean-clone reproduction of the CI command passes
- [x] CI green on GitHub (run 45876537420, after the `--no-build` fix)

**Exit**
- [x] PR #2 merged into `main`; PR #1 closed
- [ ] Delete the leftover branch `m0-scaffolding` on GitHub

**Loose ends (do opportunistically)**
- [ ] Build and smoke-test the `runtime` image target
- [ ] Build the arm64 image (Jetson) and run the tests on a Jetson
- [ ] Bump GitHub Actions to Node 24 versions (deprecation warning in CI)
- [ ] Capture tool: verify on the real camera, including `laser_mode:=interleave` and that
      per-frame `metadata` topics identify laser-on frames

---

## M1 — Sensor models on proven libraries

*Library policy (see DEV_PLAN §2): GTSAM provides poses, Lie-group math and IMU preintegration;
OpenCV provides lens models and stereo rectification. Kestrel writes only the adapters.*

**Build**
- [x] Docker image: GTSAM 4.2 (ROS-packaged) replaces Ceres
- [x] `makeImuParams()`: YAML noise model → GTSAM `PreintegrationCombinedParams` (z-up gravity)
- [x] `integrateImu()`: cut the IMU stream at frame times, interpolating boundary samples
- [x] `StereoRig`: OpenCV stereo rectification (radtan + fisheye) of feature *points*,
      rectified GTSAM `Cal3_S2Stereo`, body → rectified-left extrinsics, triangulation,
      whole-image rectification for debugging
- [x] Theory chapters 01–03: "In Kestrel" sections point at the GTSAM/OpenCV classes used

**Verify**
- [x] GTSAM toolchain smoke test (containers of fixed-size types, incremental iSAM2)
- [x] IMU parameters: gravity direction and noise units
- [x] IMU prediction matches GTSAM's analytic `AcceleratingScenario` (off-grid frame times)
- [x] Midpoint integration is second order (error ÷ ~4 when the rate doubles)
- [x] Splitting an interval in two gives the same motion as integrating it whole
- [x] Predicted IMU covariance matches Monte-Carlo within ±25 % (validates noise units)
- [x] Rectification on the EuRoC calibration: matches row-aligned (< 5e-3 px), raw ↔ rectified
      round trip, triangulated points equal ground truth in the body frame
- [x] Same for a fisheye (equidistant) variant; nominal Gemini config gives 95 mm baseline

**Data**: none (synthetic + EuRoC calibration). Ideally **D1–D3** are recorded meanwhile.

**Exit**
- [x] All of the above green in CI

---

## M2 — Stereo front-end

*OpenCV does the image processing (Shi–Tomasi, pyramidal KLT, fundamental-matrix RANSAC,
CLAHE); Kestrel's `FeatureTracker` is the policy around it.*

**Build**
- [x] Shi-Tomasi detection with min-distance spacing; existing tracks thinned longest-first
- [x] Pyramidal KLT tracking + forward–backward check
- [x] Gyro-aided prediction (pure-rotation homography as KLT's initial guess)
- [x] Fundamental-matrix RANSAC outlier rejection on rectified coordinates
- [x] Stereo matching left → right (KLT + rectified row check + positive disparity)
- [x] CLAHE option; external mask input; `dropTracks()` estimator feedback (for M4)
- [x] Debug visualization (`drawDebug`) and `run_frontend --video` on EuRoC-format data
- [x] EuRoC (ASL format) dataset reader (also used by M3)
- [ ] Affine KLT for patches stretched near the wide-FOV image edge (optional; see the gyro test)
- Moved to M9: laser-frame filter (needs the driver's per-frame metadata)

**Verify**
- [x] Synthetic: sub-pixel tracking accuracy (median < 0.05 px), stereo disparity and row
      error < 0.05 px
- [x] Gyro aid: an 11.5° whip-pan keeps ≥ 65 % of still-visible tracks (measured 71 %); without
      the gyro, 0
- [x] RANSAC rejects an independently rotating patch. Planar scenes / pure translation of a
      patch are degenerate for F (documented in the test); M4's IMU gating covers them
- [x] Masks, dropped tracks never resurrected, EuRoC reader on a synthetic dataset
- [ ] Runtime budget: ≤ 8 ms/frame at 848×480 on x86 (measure with `run_frontend` on real data)

**Data**: TUM-VI room1 (EuRoC optional — ETH's servers are very slow);
**D4** (`static_start`); **D5** (`loop`).

**Exit**
- [ ] ≥ 150 tracked features/frame on EuRoC MH_01 and on D5
- [ ] Median track length ≥ 15 frames on D5

---

## M3 — Sliding-window VIO

*GTSAM provides the optimization (fixed-lag iSAM2 smoother, IMU and stereo factors, robust
noise); Kestrel's `VioEstimator` is the policy, and `VisualInertialOdometry` the wiring.*

**Build**
- [x] GTSAM `IncrementalFixedLagSmoother` over keyframes with `CombinedImuFactor`s, explicit
      landmarks with stereo (and mono fallback) factors, Huber robust noise
- [x] Landmark policy: created only after a second keyframe confirms the first stereo
      observation; refreshed timestamps keep seen landmarks in the window; weak (5 m) prior
      against cheirality-induced indeterminate systems
- [x] Prediction gating: observations far from the IMU-predicted projection are rejected and
      the track is dropped in the front-end
- [x] Keyframe selection: time interval or too few tracked landmarks
- [x] Initialization: static start (gravity → roll/pitch, gyro bias if really still, noise-aware
      motion check) or an explicit state
- [x] Failure recovery: reset from the IMU prediction (same world frame)
- [x] IMU-rate prediction (`VisualInertialOdometry::predict`)
- [x] `run_vio` offline runner (EuRoC/TUM-VI ASL format) → TUM trajectories
- [x] `tools/evaluate.py` (evo): SE(3)-aligned ATE, RPE per metre, plots, `metrics.json`
- [ ] `run_bag` offline runner for our MCAP recordings (needs rosbag2; lands with M9 / first
      recordings)
- [ ] Performance: keyframe solve is 28 ms median on TUM-VI, ~60 ms in the denser simulation;
      tune lag / keyframe rate / landmark count (M10)

**Verify**
- [x] Synthetic end-to-end (`test_vio_estimator`, 20 s / 20 m circle): final error 0.5 cm
      (0.02 % of path), yaw 0.2°, no resets
- [x] Static start: tilt error 0.05°, drift 0.5 cm in 3 s
- [x] 3 % gross outliers: 0 resets, final error 0.5 cm, 317 mismatches reported
- [x] Deterministic: two runs on the same data give identical trajectories
- [ ] Fixed-lag smoother vs. full batch solve on a short sequence (nice-to-have)

**Data**: TUM-VI room1–6 (downloaded; ETH's EuRoC servers proved too slow — EuRoC is optional);
**D1–D3** (calibration); **D4–D7**.

**Exit**
- [x] TUM-VI room1 runs to completion with no resets: ATE 7.2 cm over 146 m (0.05 %),
      RPE 1.9 cm per metre, 23 ms/frame median (see `docs/benchmarks.md`)
- [x] TUM-VI room1–6 all complete with no resets: mean ATE 9.1 cm, RPE 1.9 cm per metre
      (table in `docs/benchmarks.md`)
- [x] EuRoC MH_01 / MH_02 complete with no resets: ATE 6.5 / 4.3 cm
- [ ] Compare against published VIO-only results on the same sequences (cite the papers' tables)
- [ ] Real time: ≥ 30 Hz on x86 at 848×480 (p95 latency logged)
- [ ] D5 (your loop): end-point drift < 1 % of path length, VIO only
- [ ] D7 (fast motion): no divergence

---

## Regression testing — cross-cutting

- [x] `tools/regression_report.py`: runs every sequence in `tools/benchmarks.yaml`, writes a
      summary table (mean/median/max) + `summary.json`, compares with
      `tools/regression_baseline.json`, exits 1 on regressions (`docs/benchmarks.md`)
- [x] Initial baseline committed for TUM-VI rooms 1–6 and EuRoC MH_01/02
- [x] **Quick CI regression test** (~1.5 min of a CI job, plus a one-off 167 MB download that is
      cached afterwards): runs the real pipeline end-to-end on a short, fixed clip and fails the
      build if accuracy regresses
  - [x] Clip: first 40 s of TUM-VI room1 (initialization, tracking, ~20 loop closures), cut by
        `tools/make_ci_clip.py` into a reproducible archive (8-bit, bit-identical to what the
        pipeline reads; TUM-VI is CC BY 4.0, attribution in the clip README)
  - [x] Clip + vocabulary hosted as release asset `ci-data-v1/kestrel_ci_data.tar.gz`; SHA-256
        pinned in `ci.yml`, cached with `actions/cache` keyed by that checksum
  - [x] `ci_sequences` / `ci_tolerances` in `tools/benchmarks.yaml`; `regression_report.py --ci`
        with its own baseline `tools/regression_baseline_ci.json`; timing is not checked; a
        minimum loop count catches a silently disabled loop closer
  - [x] CI step after the unit tests; `report.md` + `summary.json` uploaded as a workflow artifact
  - [x] Determinism check: two runs must give byte-identical trajectories and loops
- [ ] Cross-check the CI baseline against the GitHub runner's numbers. Re-baseline from a CI run
      if the CPU difference is larger than the tolerance
- [ ] Add our own Gemini 335Lg recordings to the registry as they arrive (D4–D18)

---

## Visualization (Foxglove) — cross-cutting

- [x] Foxglove SDK in the image (pinned, checksummed, x86-64 + aarch64)
- [x] `kestrel_viz`: live WebSocket + MCAP recording of images, tracks, poses, TF, landmarks,
      frustum, per-frame stats and log messages (`docs/visualization.md`)
- [x] `run_vio --foxglove / --mcap / --rate`; layout `tools/foxglove/kestrel_layout.json`
- [x] Loop-closure edges, pose graph, window factor graph and the map-frame trajectory (M5)
- [ ] Voxel map and change regions (with M8)
- [ ] Same topics from the ROS node via foxglove_bridge (M9)

---

## M4 — Dynamic-object robustness

**Build**
- [ ] Post-solve reprojection gating → flag and remove dynamic tracks
- [ ] Dynamic "heat" grid with decay; blocks new detections in hot cells
- [ ] Back-end → front-end feedback (drop flagged track ids)
- [ ] External semantic-mask input (topic / API), optional
- [ ] Optional ONNX segmentation plug-in (YOLO-seg class) behind a CMake flag

**Verify**
- [ ] Synthetic: a moving landmark cluster (30 % of features) leaves ATE within 10 % of the
      static case, and ≥ 90 % of its tracks are flagged
- [ ] Ablation script: on/off comparison table produced automatically

**Data**: **D8** (people walking), **D9** (large moving object filling the view).

**Exit**
- [ ] D8: ATE / end-point drift within 1.5× of the comparable static recording
- [ ] D9: no divergence while a large object crosses the view

---

## M5 — Loop closure

*FBoW proposes places, OpenCV verifies them (ORB, PnP-RANSAC), GTSAM optimizes the pose graph
(iSAM2). Kestrel's `LoopCloser` / `SlamSystem` are the policy and the glue.*

**Build**
- [x] Keyframe ORB extraction + stereo 3D points (sub-pixel: cornerSubPix + LK on the matched row)
- [x] FBoW in the image (pinned commit; ARM portability patch), behind a pluggable
      `PlaceRecognizer` interface with our inverted index
- [x] `train_vocabulary`; recipe: EuRoC MH_01+MH_02 → `data/vocabulary/kestrel_orb_k10L5.fbow`
      (13 s), evaluated on TUM-VI (different places)
- [x] Normalized BoW score, recent-past exclusion, temporal-consistency check
- [x] Geometric verification: ratio-tested matching + EPnP-RANSAC + LM refinement, ≥ 80 inliers
- [x] GTSAM iSAM2 pose graph (Gaussian loop edges — see theory ch. 08) in its own thread
      (inline offline); drift correction `T_map_odom`; `SlamSystem` (REP-105 frames)
- [x] `run_vio` writes map-frame trajectories and `loops.txt`; `tools/analyze_loops.py` audits
      every loop against ground truth; `tools/regression_report.py` compares VIO vs. SLAM and checks for regressions
- [x] Foxglove: pose graph `/slam/graph` (odometry edges, magenta loop closures), corrected path
      `/slam/path`, window factor graph `/vio/factors` (IMU + visual factors), factor counts

**Verify**
- [x] BoW: the right place wins (unit test)
- [x] Verification recovers a known relative pose to 2 mm / 0.04° and rejects another place
- [x] Pose graph removes injected drift on a synthetic loop (0.77 m → 0.05 m)
- [x] Precision: 0 wrong loops on TUM-VI room2 (152 loops audited vs. ground truth)
- [x] Post-optimization consistency check (chi²) with a stiff pose graph removes imprecise loops
- [ ] Precision on all benchmark sequences: a few imprecise loops (0.2–0.4 m, right place) still
      pass on room1/room4/room6/MH_01. Next ideas: weight loop edges by the PnP inlier geometry
      (covariance from the inlier spread / depth), reject loops dominated by distant points,
      bidirectional verification
- [ ] Tune loop-edge sigma to the measured loop accuracy (median ~2 cm / 0.6° vs. the current
      5 cm / 1.1°) so single loops pull harder against the stiff odometry chain; re-baseline
- [ ] Tracking latency while the pose graph runs (live, `--async`): p95 logged

**Data**: TUM-VI rooms, EuRoC MH_01/02; later **D5**, **D6** (figure-8), **D10** (perceptual
aliasing: repetitive corridors/aisles).

**Exit**
- [x] TUM-VI room2: ATE 18.3 cm (VIO) → 2.0 cm (SLAM, final pose graph)
- [ ] Full-system ATE ≤ VIO-only ATE on every benchmark sequence
- [ ] D5 / D6: end-point error after loop closure < 0.2 % of path length
- [ ] D10: no false loops accepted (precision = 100 %)

---

## M6 — Multi-session maps & relocalization

**Build**
- [ ] Map serialization (keyframes, descriptors, 3D points, pose graph, persistence state), versioned
- [ ] Load map as a fixed prior; relocalize; publish `map → odom` only once relocalized
- [ ] Sequence handling: reset/tracking loss → new sequence → merge on cross-sequence loop
- [ ] `save_map` / `load_map` / `reset` services

**Verify**
- [ ] Save → load round-trip is bit-exact
- [ ] Synthetic: two sessions with an unknown offset merge into one consistent frame

**Data**: EuRoC MH_01 → MH_02–05; **D11** (two sessions, same place, different days);
**D12** (kidnapped robot: cover the lens mid-run).

**Exit**
- [ ] Relocalization succeeds within 5 s on MH_02–05 against an MH_01 map
- [ ] D11: second session relocalizes and merges; consistency error < 5 cm at revisited markers
- [ ] D12: recovers from the lens cover and re-merges without manual action

---

## M7 — Lifelong mapping (persistence filters)

**Build**
- [ ] Persistence filter (log-space recursion, exponential survival prior)
- [ ] Visibility test: projection + depth-based occlusion vs. disappearance
- [ ] Pruning of low-persistence points; stale keyframes removed from place recognition
- [ ] New keyframe insertion over changed areas
- [ ] Map statistics over sessions (points added / pruned / survived)

**Verify**
- [ ] Filter posterior equals the closed form (theory ch. 09) for random observation sequences
- [ ] Synthetic: removed object's points pruned within ≤ 3 revisits; static points survive ≥ 99 %
- [ ] Occlusion (a person standing in front) does not prune the occluded points

**Data**: **D13** (before/after pairs with documented changes); **D14** (multi-week series).

**Exit**
- [ ] D13: ≥ 90 % of points on removed objects pruned after the after-change session; < 2 %
      of static points pruned
- [ ] D14: relocalization success ≥ 95 % across all sessions; map size stays bounded

---

## M8 — Voxel map & change detection

**Build**
- [ ] OctoMap (ROS-packaged) occupancy with clamped log-odds, fed with keyframe depth
- [ ] Hysteresis state machine → appeared / disappeared events
- [ ] 2D occupancy grid projection (height band) for Nav2
- [ ] Rebuild from stored keyframe clouds after large loop corrections
- [ ] Depth source selection: laser-on depth in interleave mode, passive otherwise

**Verify**
- [ ] Synthetic: inserted then removed box → appeared then disappeared events; grid matches voxels
- [ ] OctoMap clamping lets a voxel flip back within N observations (configurable)
- [ ] Integration ≤ 15 ms per keyframe on x86

**Data**: **D13**, **D15** (objects moved while the camera watches).

**Exit**
- [ ] D13: ≥ 80 % of documented changes are detected as change regions, with ≤ 1 false change
      region per 100 m²
- [ ] 2D grid accepted by Nav2 (map server / costmap) without errors

---

## M12 — Long-term operation in changing spaces (people, pallets)

**Scenario.** The robot localizes for weeks against a **pre-stored map** (M6) in a space with
many moving people and with piles of objects (pallets, boxes, carts) that appear and disappear
day to day. Localization must rest on what is permanent: walls, columns, racking structure,
ceilings, high signage. Two features, both configurable per site and both cutting map points
out *before* they can produce wrong matches. They sit alongside the M7 persistence filter, which
only learns that something is gone after it has already been seen missing.

Builds on: M4 (dynamic tracks), M6 (stored map, relocalization), M7 (persistence), M8 (voxel
map and change regions). Ships after M8; feature A's front-end part can land with M4.

### Feature A — Ignore moving objects and everything below a height band

Default band: **keep only points ≥ 1.83 m (6 ft) above the floor**; configurable, e.g.
`map_filter.min_height_m`. People, carts, forklifts and pallet stacks live below it.

**Build**
- [ ] Floor reference in the map frame: gravity from the VIO (z is already gravity-aligned) plus
      the floor height, from the robot's `base_link` (M9: camera mount height) or from a floor-plane
      fit (RANSAC on low stereo points) when running hand-held
- [ ] Height filter on **map points**, used for the stored map, relocalization and loop
      closure: points below the band are not stored as map landmarks, and their ORB features are
      left out of the BoW descriptor and PnP verification
- [ ] Separate, softer setting for **odometry** (VIO). Short-term tracking still benefits from
      static low features, so by default these are only down-weighted, not removed. Removing
      them all would starve a forward-looking camera mounted below 1.8 m (see the risk below)
- [ ] **Moving objects** at any height: M4 machinery (post-solve reprojection/epipolar gating,
      dynamic heat grid, optional person/forklift segmentation masks) also keeps flagged tracks
      out of the map
- [ ] Optional height-band and semantic class lists: `ignore_classes: [person, forklift, pallet]`
- [ ] Foxglove: map points coloured kept / below band / dynamic; the band drawn as a plane

**Verify**
- [ ] Synthetic: random points with known heights → exact split at the threshold, under
      arbitrary camera tilt (gravity from the estimator, not from the camera axis)
- [ ] Floor-plane fit: < 2 cm height error on synthetic stereo with 30 % clutter
- [ ] Ablation on D19: relocalization and ATE with the filter on vs. off (auto-generated table)

**Risk / camera mounting.** A camera at ~1 m looking straight ahead sees few points above 1.8 m
in a low-ceiling room. Check the keep ratio per keyframe on D19 early. If it is too low, tilt the
camera up 10–20° or lower the band. Relocalization reports "too few permanent features" instead
of guessing.

### Feature B — Exclusion zones: ignore parts of the map with large object changes

**Build**
- [ ] Zone file per map (`zones.yaml`, stored and versioned with the map): named 2D polygons in
      the map frame with an optional height range, e.g. `pallet_staging`, `loading_dock`.
      Points inside a zone are never used for localization or loop closure and never enter the
      persistent map; change detection there is reported as "expected" rather than as an anomaly
- [ ] **Finding the zones (data-driven):** M7/M8 statistics over sessions → a per-cell
      *change-frequency* heat map (appeared/disappeared events and pruned points per m² per
      session). Cells above a threshold are proposed as zones; a person approves or edits them
- [ ] Zone editing without code: draw in Foxglove (map + heat map shown) or edit the YAML; a
      `zones` CLI `propose` / `apply` / `show`
- [ ] ROS 2: zones loaded with the map, reloadable at runtime (service), published as markers
- [ ] Interaction with A: zones are absolute (all heights) unless a height range is given;
      above-band points inside a zone, e.g. ceiling over a dock, can stay usable

**Verify**
- [ ] Unit: point-in-polygon with height range, incl. concave polygons and zone edges
- [ ] Synthetic multi-session: a region whose boxes are shuffled every session is proposed as a
      zone after ≤ 3 sessions; static regions are never proposed
- [ ] Relocalization in D20 with zones on vs. off: fewer wrong or rejected matches, same or
      better success rate

**Data**: **D19**, **D20** (below).

**Exit (A + B together)**
- [ ] D19/D20: relocalization success ≥ 95 % across all sessions, **zero** wrong relocalizations
      or loops, ATE vs. a static-day session within 1.5×
- [ ] Map points kept in zones or below the band: 0 (by construction; checked by a test)
- [ ] Proposed zones cover ≥ 80 % of the documented pallet areas in D20 with ≤ 20 % false area

---

## M9 — ROS 2 node & Nav2

**Build**
- [ ] Synced IR pair, IMU, optional depth and mask subscriptions
- [ ] Laser-frame filtering from driver metadata (skip laser-on frames in interleave mode;
      moved here from M2)
- [ ] Visual check: zero tracks on laser dots in an interleave recording
- [ ] Odometry (IMU rate), TF `map → odom → base_link`, path, landmarks, voxels, changes,
      grid, loop markers
- [ ] Calibration from `camera_info` + TF, or YAML (switch)
- [ ] `base_link → camera` from the robot's URDF/TF
- [ ] Launch: `gemini_335lg.launch.py` (driver + Kestrel), `bag.launch.py` (replay)
- [ ] Foxglove layout + RViz config
- [ ] `slam` compose service (camera → SLAM live)

**Verify**
- [ ] Launch test: node starts, consumes a short bag, publishes all topics
- [ ] TF tree check (no conflicts with the driver's TFs)
- [ ] Message rates and latency measured under load

**Data**: any D-series bag replayed; then live (**F1**, **F2**).

**Exit**
- [ ] Bag replay output identical to the offline runner
- [ ] Nav2 navigates a goal sequence on the robot using Kestrel's map and TF (**F2**)

---

## M10 — Benchmarking, tuning, performance

**Build**
- [ ] One-command benchmark: `docker compose run --rm bench` → `docs/benchmarks.md` table
- [ ] Parameter sweeps (features, window size, noise inflation) with recorded results
- [ ] Profiling (perf / tracing) on x86 and Jetson Orin
- [ ] Regression guard in CI: see "Regression testing" (quick CI regression test)

**Verify / Exit**
- [ ] Benchmarks table: EuRoC, TUM-VI (subset), your D-series, next to published baselines
- [ ] Real time on Jetson Orin at 848×480 @ 30 Hz (≤ 33 ms p95 end-to-end)
- [ ] CPU and memory budget documented for a 1-hour run

---

## M11 — Tutorial chapters (per module)

- [x] Theory chapters 01–11 + references
- [ ] Per-module tutorial chapters linking theory → code → experiments
- [ ] "Reproduce our results" walkthrough (EuRoC + one D-series bag)
- [ ] Exercises with solutions branch

---

## Real-world data collection

All recordings use `docker compose run --rm capture session:=<place> scenario:=<id>`. See
[hardware/capture_guide.md](hardware/capture_guide.md) for how. Keep every bag's
`.capture.yaml`; it should show no warnings, `stereo.unmatched: 0` and `static_start_complete: true`.

**Ground truth without a motion-capture system**: put tape **markers** on the floor at the
start/end point and a few waypoints, and measure distances between them with a tape or laser
measure. Returning precisely to a marker gives a free end-point error. Optionally, print a few
large AprilTags at surveyed positions; they give point checks along the path.

### Calibration (needed before M3 on your camera)
- [ ] **D1** `calib_camera`: AprilGrid, stereo intrinsics/extrinsics → Kalibr *(print a 6×6 grid)*
- [ ] **D2** `calib_imu`: AprilGrid with full 6-axis excitation at `imu_rate:=500hz` → Kalibr
      camera–IMU extrinsics and time offset
- [ ] **D3** `imu_static`: 2–3 h untouched, `fps:=5 record_depth:=false` → Allan variance
- [ ] Run Kalibr in Docker; commit the results as `kestrel_ros/config/<serial>.yaml`
- [ ] Compare with factory calibration (driver `camera_info` / TF) and document the differences

### Odometry & loop closure (M2–M5)
- [ ] **D4** `static_start`: 3 s still, short walk (sanity, start-up behaviour)
- [ ] **D5** `loop`: 30–100 m closed loop, end on the start marker, 2–3 laps
- [ ] **D6** `figure8`: two loops sharing a corridor (loop detection from different headings)
- [ ] **D7** `fast`: aggressive motion and quick turns (robustness)
- [ ] **D10** `aliasing`: repetitive corridors/aisles/shelving (false-loop stress test)
- [ ] Repeat D5 in **interleave** laser mode (front-end + depth source checks)
- [ ] Low light / mixed lighting variant of D5

### Dynamic scenes (M4)
- [ ] **D8** `dynamic`: people walking through view, same route as D5
- [ ] **D9** `dynamic_large`: a large object (cart, forklift, door) crossing or filling the view

### Multi-session & lifelong (M6–M8) — the core of this project
- [ ] **D11** `session_a` / `session_b`: same route, different days, no deliberate changes
- [ ] **D12** `kidnap`: cover the lens for ~3 s mid-run, keep walking
- [ ] **D13** `before_change` / `after_change` pairs:
  - [ ] Pair 1: move 2–3 large items (shelves, boxes). Write down exactly what moved, with photos
  - [ ] Pair 2: remove items and add new ones
  - [ ] Pair 3: the same changes at a different time of day (lighting)
- [ ] **D14** multi-week series: the same route weekly for ≥ 4 weeks while the space naturally changes
- [ ] **D15** `change_live`: the camera watches while someone moves objects in view

### Long-term operation in changing spaces (M12)
- [ ] **D19** `busy_floor`: the same route on ≥ 5 days with people walking through and carts
      moving; note the camera height and tilt. Record one quiet (static) session as reference
- [ ] **D20** `pallet_days`: a space where pallets or box piles change daily (warehouse,
      loading area, storeroom) over ≥ 2 weeks. Photograph and sketch the pallet areas each day
      (ground truth for the proposed zones)

### Outdoor (secondary target)
- [ ] **D16** `outdoor`: shade → sun transition (IR saturation check), 100–300 m loop
- [ ] **D17** `outdoor_dusk`: low light

### On the robot (M9+)
- [ ] Measure and record the camera mount (photo + `base_link → camera` measurements)
- [ ] **D18**: drive the D5/D13 routes on the robot (realistic motion and vibration)

---

## Field testing

Live tests on the real system. Record a bag of every field test (the capture tool can run
alongside) so failures can be replayed offline.

- [ ] **F1 — Handheld live**: run Kestrel live with Foxglove; walk D5 and D6 routes. Pass: no
      resets, loop closes visibly, end-point error < 0.5 % of path length.
- [ ] **F2 — Robot + Nav2**: navigate a sequence of 10 goals in a mapped space. Pass: ≥ 9/10
      reached, no localization jumps > 10 cm.
- [ ] **F3 — Relocalization**: start the robot at 10 random poses in a saved map. Pass: ≥ 9/10
      relocalize within 5 s.
- [ ] **F4 — Changed environment**: rearrange large objects, then repeat F2 and F3. Pass: same bars;
      the change regions appear in `/changes`.
- [ ] **F5 — Dynamic**: F2 with people walking around. Pass: same bars as F2.
- [ ] **F6 — Endurance**: 2 h continuous operation. Pass: no crash, no memory growth beyond the
      documented budget, real-time maintained.
- [ ] **F7 — Outdoor**: D16 route live. Pass: no divergence; drift documented.
- [ ] **F8 — Jetson**: F1, F2 and F6 on the Jetson Orin.

---

## Definition of "solid" (v1.0 release criteria)

Kestrel v1.0 is declared when **all** of these hold, with the evidence linked from
`docs/benchmarks.md`:

- [ ] **Accuracy**: EuRoC full-system ATE within 1.2× of the best published open-source
      stereo-inertial result on each sequence (ORB-SLAM3 / VINS-Fusion / Basalt)
- [ ] **Loop closure precision**: zero false loops across all benchmark and D-series data
- [ ] **Lifelong**: D13 and D14 targets met (pruning, relocalization ≥ 95 %, bounded map)
- [ ] **Dynamic**: D8 and D9 targets met
- [ ] **Changing spaces**: M12 exit targets met on D19 and D20 (height band + exclusion zones)
- [ ] **Field**: F1–F6 passed (F7 and F8 documented)
- [ ] **Real time**: 30 Hz on x86 and Jetson Orin
- [ ] **Robustness**: no crash in 10 h cumulative field operation
- [ ] **Teaching**: every module has its tutorial chapter; the "reproduce our results"
      walkthrough works on a clean machine with only Docker
