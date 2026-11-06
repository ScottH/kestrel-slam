# Kestrel SLAM — Development Plan

> **Status:** draft for review (2026-11-06). Nothing below is implemented yet.
> Phases are ordered so that every milestone ends with something that builds in Docker,
> has tests, and can be evaluated on data.

---

## 1. Goals

| # | Goal | How we will know |
|---|------|------------------|
| G1 | Accurate, robust 6-DoF localization from an **Orbbec Gemini 335Lg** (global-shutter stereo IR + internal IMU) | ATE / RPE on public stereo-inertial benchmarks within the range of published SOTA systems (VINS-Fusion, ORB-SLAM3, Basalt, OpenVINS) |
| G2 | **Loop closure** — detect revisits, correct accumulated drift, keep a globally consistent map | Drift reduction on loop-rich sequences; zero accepted false-positive loops on the benchmark set |
| G3 | **Lifelong / changing environments** — large objects may appear, disappear or move | Maps stay usable across sessions; removed objects disappear from the map; relocalization still succeeds after rearrangement |
| G4 | **Dynamic objects** in view (people, forklifts, doors) do not corrupt the estimate | Bounded accuracy loss on dynamic sequences vs. static ones |
| G5 | **ROS 2 native** — drop-in for Nav2 (`map → odom → base_link` TF, `nav_msgs/Odometry`, `OccupancyGrid`) | Launch with the camera, drive a robot with Nav2 on the output |
| G6 | **Zero local installs** — everything builds, tests and runs in Docker | `docker compose run test` on a clean laptop with only Docker installed |
| G7 | **Tutorial-quality code** — modular, heavily commented, one documentation chapter per module | A reader can follow `docs/tutorial/` from the sensor model to the full system |

Indoor is the primary target; outdoor (sunlight, longer depth range) is a supported secondary case.

---

## 2. Architecture

```
               ┌──────────────────────── kestrel_ros (ROS 2 node) ─────────────────────────┐
 Orbbec driver │  IR left/right ─┐                                                         │
 (OrbbecSDK_   │  IMU (gyro+acc) ├─► kestrel::System  ──► /odom, TF, /path, /map (grid),   │
  ROS2)        │  depth (opt.)   │                         /voxels, /changes, /loops       │
               │  semantic mask ─┘   (optional external dynamic-object mask)               │
               └───────────────────────────────────────────────────────────────────────────┘

 kestrel_core (pure C++17, no ROS dependency)
 ┌──────────────┐  FeatureFrame  ┌──────────────────────────┐  Keyframes  ┌───────────────────────────┐
 │  Front-end   │ ─────────────► │  Sliding-window VIO      │ ──────────► │  Map manager (own thread) │
 │ stereo KLT,  │                │  IMU preintegration,     │             │  • BoW place recognition  │
 │ gyro-aided   │ ◄───────────── │  Ceres BA, Schur         │ ◄────────── │  • PnP loop verification  │
 │ prediction   │  dynamic ids / │  marginalization,        │  drift corr │  • 4-DoF pose graph       │
 └──────────────┘  mask grid     │  dynamic-feature filter  │  T_map_odom │  • persistence filters    │
                                 └──────────────────────────┘             │  • voxel map + changes    │
                                                                          │  • multi-session save/load│
                                                                          └───────────────────────────┘
```

**Design rules**

* `kestrel_core` never includes ROS headers. The same code runs from the ROS node, offline dataset
  runners and unit tests.
* Each module has one header, one source file, one test file and one tutorial chapter.
* Every algorithm cites its source paper, with equation references in comments where useful.
* Configuration lives in a single YAML file read by both the ROS node and the offline tools.

**Dependencies:** Eigen 3.4, OpenCV ≥ 4.5, Ceres ≥ 2.1 (Manifold API), GoogleTest, and ROS 2 **Jazzy**.
All of these are **inside the Docker image**, never on the host. No GTSAM, no Pangolin, no
deep-learning runtime is required. Learned components are optional plug-ins.

---

## 3. Algorithm choices (and why)

| Subsystem | Choice | Rationale / reference |
|-----------|--------|-----------------------|
| Features | Shi-Tomasi corners + pyramidal KLT, forward–backward check, gyro-rotation-compensated prediction | Fast and accurate on a global-shutter sensor (VINS-Fusion / OpenVINS style) |
| Stereo matching | KLT left→right with an epipolar check against calibrated extrinsics | Works without rectified images |
| IMU | On-manifold preintegration, midpoint integration, first-order bias correction | Forster et al., TRO 2017 |
| Estimator | Fixed-lag sliding window (10 keyframes), Ceres, inverse-depth landmarks, Cauchy loss | Qin et al. (VINS); Leutenegger et al. (OKVIS) |
| Marginalization | Schur complement → linear prior; keyframe vs. non-keyframe policy | Keeps information from dropped states |
| Initialization | Stereo-inertial: gravity from the accelerometer, poses from stereo PnP, gyro-bias alignment, then full BA | Stereo gives metric scale, so start-up is fast and robust |
| Dynamic objects (short-term) | (1) Robust loss + χ² reprojection gating after each solve; (2) a decaying image-grid "dynamic heat" map that blocks new features in moving regions; (3) an optional external semantic-mask topic | Geometric by default; semantic segmentation (e.g., YOLO-seg) is a plug-in, not a dependency |
| **Loop closure** | ORB descriptors on keyframes → hierarchical bag of words (our own trainer, DBoW2-style tf-idf) → temporal-consistency check → descriptor matching + **PnP-RANSAC** against stereo-triangulated 3D points → **4-DoF pose graph** (x, y, z, yaw; roll and pitch are observable from gravity), in a dedicated thread | Gálvez-López & Tardós (DBoW2); Qin et al. (VINS 4-DoF pose graph) |
| Multi-session / relocalization | Each tracking run is a *sequence*. A loaded prior map is a fixed sequence. Live sequences merge on a verified cross-sequence loop. Tracking loss starts a new sequence instead of losing the map. | Same approach as VINS-Fusion map merging and the ORB-SLAM3 Atlas |
| Changing environment (long-term) | A **persistence filter** per map point (Rosen, Mason & Leonard, ICRA 2016): a Bayesian survival estimate from detections and misses, using depth to tell "occluded" from "gone". Low-persistence points are pruned; stale keyframes leave place recognition. | Principled removal of objects that moved or disappeared |
| Dense map | Hashed voxel occupancy with clamped log-odds (so it can change back). Depth from keyframes is ray-cast, appeared/disappeared voxel events are recorded, a 2D `OccupancyGrid` is projected for Nav2, and the map is rebuilt from stored keyframe clouds after large loop corrections. | OctoMap-style clamping keeps the map responsive to change |

---

## 4. Docker-first development environment

The host needs nothing but Docker (plus the NVIDIA Container Toolkit only if you want GPU features
on a Jetson or a desktop GPU).

```
docker/
├── Dockerfile            multi-stage: base (ROS 2 Jazzy + Eigen/OpenCV/Ceres/GTest)
│                           → dev (build tools, evo, gdb, clang-format)
│                           → runtime (slim image with the built workspace, for the robot)
│                           → orbbec (adds the OrbbecSDK_ROS2 driver + udev rules)
├── compose.yaml          services below
└── entrypoint.sh         sources ROS, the workspace overlay, and runs the command
```

| `docker compose …` | What it does |
|--------------------|--------------|
| `run --rm build` | colcon build of the repo (source bind-mounted, build cache in a named volume) |
| `run --rm test` | colcon test + ctest + GoogleTest results |
| `run --rm euroc MH_01_easy` | downloads (once, into a `datasets` volume), runs and evaluates a EuRoC sequence, writes plots to `./results` |
| `up slam` | camera driver + Kestrel node on real hardware (`--privileged`, `/dev/bus/usb`, host network for DDS) |
| `up rviz` | RViz in a container, using X11/Wayland forwarding (or Foxglove Studio via the browser, which needs no display plumbing) |
| `run --rm shell` | interactive dev shell with everything installed |

* Images are **multi-arch** (amd64 for your laptop, arm64 for Jetson). On Jetson the base switches
  to an L4T/JetPack-compatible ROS 2 Jazzy image.
* The source tree is bind-mounted, so you edit on the laptop with any IDE. A
  `.devcontainer/devcontainer.json` lets VS Code open straight into the container with IntelliSense.
* CI (GitHub Actions) uses the same `test` service, so "works in CI" equals "works on your laptop".

---

## 5. Repository layout

```
kestrel_slam/
├── README.md                      quick start (docker compose), results table
├── docker/  .devcontainer/        see §4
├── docs/
│   ├── DEV_PLAN.md                (this file)
│   ├── architecture.md
│   ├── hardware/gemini_335lg.md   driver setup, laser/interleave, Kalibr calibration (in Docker)
│   ├── benchmarks.md              procedure + results
│   └── tutorial/                  00_overview … 12_lifelong_mapping (one chapter per module)
├── kestrel_core/                  ament_cmake package; also builds with plain CMake
│   ├── include/kestrel/{common,sensors,imu,frontend,estimator,loop,mapping,io}/
│   ├── src/  apps/ (run_euroc, run_tumvi, train_vocabulary)  test/
├── kestrel_interfaces/            SaveMap.srv, LoadMap.srv, ResetMap.srv
├── kestrel_ros/                   node, launch (gemini_335lg, bag replay), config, rviz
└── tools/                         evaluate.py (evo wrapper), plot scripts
```

---

## 6. Milestones

Each milestone ends in a commit whose tests pass in `docker compose run --rm test`.
✅ marks the acceptance criteria.

### M0 — Scaffolding + Docker
Dockerfile (base/dev/runtime stages), compose services, devcontainer, CMake/ament packages, config
loader, logging, CI workflow, `.clang-format`, license.
✅ `docker compose run --rm build` and `… test` succeed on a clean machine with only Docker installed.

### M1 — Math & sensor models
`SE3`/SO(3) utilities, pinhole-radtan and equidistant camera models, IMU noise model, and
**IMU preintegration** with covariance and bias Jacobians.
✅ Exp/Log round-trip tests; numeric-vs-analytic Jacobians; preintegration matches ground truth on a
simulated trajectory; bias correction is accurate to first order.

### M2 — Front-end
Stereo KLT tracker, gyro-aided prediction, F-matrix RANSAC, grid-uniform detection, masks.
✅ Runs on EuRoC images; track-length and stereo-match statistics are logged; ≥ 150 features per frame on MH_01.

### M3 — Sliding-window VIO (the core)
Feature manager, triangulation, Ceres factors (IMU, mono and stereo reprojection), custom pose
manifold, marginalization, initializer, keyframe policy, failure detection and reset.
✅ **Synthetic end-to-end test** (simulated landmarks + IMU, no images): ATE < 2 cm.
✅ All factor Jacobians verified numerically.
✅ EuRoC MH_01–05, V1, V2 run to completion, with VIO-only ATE within ~1.5× of published VINS-Fusion stereo-inertial results.

### M4 — Dynamic-object robustness
χ² gating, dynamic heat grid, front-end feedback, external mask input.
✅ Synthetic test with a moving landmark cluster: the estimate is unaffected and the cluster is flagged.
✅ Measurable gain on OpenLORIS and on your own dynamic recordings.

### M5 — Loop closure ⭐
ORB extraction + stereo 3D points on keyframes, vocabulary trainer, BoW database, temporal
consistency, PnP-RANSAC verification, 4-DoF pose-graph thread, drift-correction output (`T_map_odom`).
✅ Unit tests: BoW scoring; the pose graph recovers injected drift.
✅ EuRoC: full-system ATE ≤ VIO-only ATE on every sequence, with **zero false loops** across the set.
✅ Pose-graph optimization never blocks tracking (separate thread, latency logged).

### M6 — Multi-session maps & relocalization
Binary map serialization (keyframes, descriptors, 3D points, pose graph, persistence state),
loading a map as a fixed prior, relocalization, and sequence merging after tracking loss.
✅ Save → restart → load → relocalize on EuRoC MH_01 → MH_02 (same building, different run).

### M7 — Lifelong mapping (changing environments)
Persistence filters (visibility from projection + depth occlusion test), pruning, stale-keyframe
handling, and new keyframe insertion over changed areas.
✅ The filter posterior matches the paper's closed form.
✅ Synthetic "object removed" scenario: affected points are pruned within N revisits, while static points survive.

### M8 — Dense voxel map & change detection
Voxel hashing, DDA ray-casting, a hysteresis state machine, change events, 2D grid projection, and
rebuild after loop correction.
✅ An object inserted and then removed produces appeared/disappeared events; the 2D grid matches the voxels.

### M9 — ROS 2 integration
`kestrel_ros` node with:
* time-synced IR pair, IMU, and optional depth and mask inputs;
* high-rate IMU-propagated odometry and the full TF tree;
* publishers for path, landmarks, voxels, changes, grid and loop markers;
* save/load/reset services;
* a Gemini 335Lg launch file that includes the Orbbec driver with the correct IR/IMU/laser settings;
* an RViz config, plus a Foxglove layout;
* extrinsics taken from the driver's TF tree or from YAML.

The node runs in the `orbbec` image.
✅ `docker compose up slam` works with the camera; rosbag replay works; frames are Nav2-compatible.

### M10 — Benchmarking & tuning
Dockerized `tools/evaluate.py` (evo: ATE/RPE, plots) and scripts for EuRoC, TUM-VI and OpenLORIS.
The README gets a results table with honest measured numbers next to published baselines.
Profiling targets: real-time at 30 Hz on x86 and on Jetson Orin.

### M11 — Tutorial documentation
Chapters 00–12, one per module. Each chapter covers the theory (with equations), points to the code,
and lists experiments to try. A hardware chapter covers Gemini 335Lg setup and Kalibr camera–IMU
calibration (Kalibr also runs in Docker).

---

## 7. Testing strategy

* **Unit tests** (GoogleTest) for every math module, with numeric-differentiation checks for every factor.
* **Synthetic integration tests**: a simulator generates a trajectory, noisy and biased IMU readings,
  and landmark observations, including moving and disappearing objects. This lets us test the
  back-end, loop closure and lifelong logic deterministically, without datasets.
* **Dataset regression**: EuRoC (accuracy), TUM-VI (robustness), and OpenLORIS-Scene (lifelong and
  dynamic scenes, recorded with similar stereo-IR + IMU hardware). Results are tracked in
  `docs/benchmarks.md`.
* **Hardware**: your own Gemini 335Lg rosbags. Ideally include an A/B pair of the same space with
  furniture moved between the two recordings.
* All of the above runs through Docker, both locally and in CI.

---

## 8. Risks & mitigations

| Risk | Mitigation |
|------|------------|
| **IR laser projector dot pattern** moves with the camera and creates fake static features | Default to laser off for tracking, and document the driver's interleaved laser on/off mode so depth (laser on) and tracking (laser off) can share one stream. The front end can drop laser-on frames. |
| Factory camera–IMU extrinsics or time offset are imprecise | A Kalibr calibration guide; optional online extrinsic estimation as a stretch goal |
| A BoW vocabulary must be trained (ORB-SLAM's vocabulary is GPL) | Ship our own trainer and a documented training recipe. Loop closure is disabled with a warning if no vocabulary is present. |
| "SOTA" claims | Report only measured numbers, side by side with published baselines on identical sequences |
| USB/GMSL camera access from inside Docker | Compose `slam` service runs privileged with `/dev` and udev rules mounted; GMSL on Jetson needs the host's JetPack camera drivers, which the hardware doc covers |
| GUI (RViz) from a container on a laptop | Foxglove bridge as the default (browser-based); X11/Wayland forwarding documented as an alternative |

---

## 9. Open questions for you

1. **Compute target** — Jetson Orin (likely, given the GMSL2 335Lg), x86, or both? This sets the
   real-time budget and decides whether arm64 images and optional GPU features are first-class.
2. **ROS 2 distro** — Jazzy is assumed. Do you also need Humble?
3. **License** — Apache-2.0 is assumed (this means no GPL dependencies).
4. **Learned components** — classical by default with optional ONNX plug-ins (SuperPoint/LightGlue
   features, YOLO-seg dynamic masks), or skip them entirely?
5. **Data** — do you have, or can you record, Gemini 335Lg bags? Ideally the same space before and
   after moving large objects.
6. **Hosting** — should this be pushed to a GitHub remote (enables CI)? Which org or user?

## 10. Order of work after approval

M0 (Docker first) → M1 → M3 synthetic test (back-end validated without images) → M2 → M3 on
EuRoC → M5 loop closure → M4 → M6 → M7 → M8 → M9 → M10 → M11. Tutorial chapters are written alongside
each module, not at the end.
