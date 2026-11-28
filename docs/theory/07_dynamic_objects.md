# 07 — Dynamic Objects

> **Why this chapter exists.** The theory so far assumed a **static world**: every landmark stays
> put. Warehouses, offices and streets break that assumption constantly. People walk through the
> view, forklifts drive past, doors swing. This chapter covers *short-term* dynamics, meaning
> things moving **while we are looking at them**. [Ch. 09](09_lifelong_mapping.md) covers the
> long-term case, where something was moved between visits.

Primary references: **[Saputra18]** (survey), **[Bescos18]** (DynaSLAM), **[Yu18]** (DS-SLAM),
**[Scona18]** (StaticFusion), **[Huber64]**, **[Zhang97]** (robust statistics),
**[Hartley04]** (RANSAC and model selection).

---

## 1. Why moving objects break SLAM

A feature on a moving object produces reprojection residuals that are not noise. They are
*systematic*: consistent with a different rigid motion. Three regimes:

1. **A few isolated movers** (a pedestrian far away). These are classic outliers. Robust kernels and
   RANSAC handle them.
2. **A large mover occupying a big part of the view** (a truck passing close by). Now the
   *majority* of features may move together. RANSAC ([Ch. 04](04_feature_tracking.md) §5) picks the
   largest consistent set, so it may conclude that **the truck is the world and the camera is
   moving the other way**.
3. **Slow movers** (a door closing over 3 s). Each frame-to-frame residual is small, below any
   per-frame threshold, but the landmark slowly corrupts the estimate.

### 1.1 The breakdown point

The **breakdown point** of an estimator is the smallest fraction of arbitrary outliers that can
move the estimate arbitrarily far.

* Least squares: **0%**. A single bad measurement can move the solution anywhere.
* M-estimators (Huber, Cauchy): bounded influence per residual, but still a low breakdown point
  once outliers are numerous and structured. With a bad initial guess, Cauchy may converge to the
  outliers' solution.
* RANSAC: can tolerate a majority of *unstructured* outliers, but fails when outliers are
  *structured* (one coherent wrong motion that out-votes the static scene).

The conclusion: no purely statistical, single-frame method resolves regime 2. You need
**additional information** that tells the two motions apart. The IMU provides exactly that.

---

## 2. The IMU as an independent witness

The IMU measures ego-motion independently of what is in front of the camera. In the
sliding-window estimator ([Ch. 06](06_sliding_window_vio.md)), the IMU factors constrain the
relative motion of consecutive keyframes. A coherent set of moving features must fight those
factors *and* the static features. The tight coupling turns an ambiguous visual problem into a
well-posed one. For the same reason, Kestrel does **not** let front-end RANSAC have the final say:
F-matrix RANSAC removes gross mismatches, but per-landmark decisions are made after the
visual–inertial optimization, using its residuals.

---

## 3. χ² gating of landmarks

After each optimization, compute every observation's whitened residual. If the noise model is
correct, the squared whitened residual of a static landmark observation,
$e = \mathbf{r}^\top\boldsymbol\Sigma^{-1}\mathbf{r}$, follows a **χ² distribution** with as many
degrees of freedom (dof) as the residual has components:

| Residual | dof | 95% threshold | 99% threshold |
|----------|-----|---------------|---------------|
| Mono reprojection $(u, v)$ | 2 | 5.991 | 9.210 |
| Stereo reprojection $(u_L, v_L, u_R)$ | 3 | 7.815 | 11.345 |

If an observation's $e$ exceeds the threshold, it is unlikely to be a static point under the
current estimate. Kestrel uses the same idea in pixels, which is easier to reason about. Two
gates are applied, both per observation:

* **Before the solve**, the landmark is projected with the **IMU-predicted** pose. An error above
  `gate_reprojection_px` (12 px) means the track jumped, so the observation is not added.
* **After the solve**, an error above `outlier_reprojection_px` (4 px) flags the landmark.

Flagged landmarks get no further factors, and their feature IDs are reported back to the
front-end so it stops tracking them.

Classifying *after* the solve and then re-estimating without the flagged points is a simple
version of **alternating estimation**. In Kestrel the re-estimate happens with the next keyframe
(§3.2).

### 3.1 Give a mover time to give itself away

New landmarks need a **second keyframe** to confirm their stereo position (theory ch. 06). If that
second keyframe comes only one frame later (33–50 ms), an object moving at walking speed has
moved about 2 cm, which is well inside every gate. In simulation, a box that travels with the camera
and is briefly still at its turning points got 38 of 45 points confirmed during such a moment.
The next solve followed the close, precise box, flagged **76 static points** instead, and the
estimator ended 6 m off.

Kestrel therefore requires the confirming keyframe to be at least `dynamic.min_confirmation_s`
(0.09 s, about one keyframe interval) after the first sighting. That short wait costs nothing on
static scenes, and in that simulation no box point entered the graph.

### 3.2 Remove what a flagged landmark has already contributed

Flagging stops *future* observations, but the factors a mover added in the last few keyframes stay
in the window until they are marginalized, and keep pulling on the solution. GTSAM's fixed-lag
smoother can delete factors (`factorsToRemove`), so Kestrel records where each landmark's factors
live (iSAM2 factor slots) and removes them when the landmark is flagged, in the next keyframe's
update. A slot is checked before it is removed, because marginalization frees slots and iSAM2
reuses them.

Removing outliers also helps on *clean* data, since mismatches are outliers too. On the first
40 s of TUM-VI room1, VIO ATE drops from 1.86 cm to **1.41 cm**.

*Negative results:*

* Re-solving immediately when many landmarks were flagged, as an extra iSAM2 update, sounds better
  but measured worse (2.95 cm). Doing the removal in the next regular update is both cheaper and
  more accurate.
* Removing factors only for *batches* of K ≥ 5 or 10 flags at one keyframe, on the theory that
  movers come in groups, gave no gain on clean data. It also made movers worse, because their
  flags arrive spread over several keyframes.

**How this was decided: averaged runs.** Over a 70–150 m sequence, odometry drift is *chaotic*: a
harmless change reshuffles single-run VIO ATE by ±50 % per sequence (MH_02 ranged from 4.0 to
8.5 cm across variants). Comparing single runs per sequence was misleading twice during M4. The
fair comparison runs each sequence from several start frames and averages
(`tools/regression_report.py --starts 0 4 8`):

| Mean of 8 sequences, 3 starts each | M4 off (M3) | M4, no removal | M4, K = 5 | **M4 (K = 1)** |
|---|---|---|---|---|
| VIO ATE | 8.9 cm | 8.2 cm | 8.8 cm | **8.1 cm** |
| SLAM final ATE | 2.5 cm | 2.5 cm | 2.6 cm | **2.4 cm** |

M4 is better than M3 on average even on clean data. Per sequence it is not uniformly better:
TUM-VI room1 VIO is consistently worse (11.0 vs. 6.9 cm averaged), while its SLAM result is
unchanged (2.2 cm).

### 3.3 The avalanche guard

If one solve would flag more than half of a keyframe's observations (`dynamic.max_flag_fraction`),
the *solution* is the suspect, not the world: a large mover has pulled it. Flagging the majority
would hand the estimator to the mover (regime 2 of §1), so nothing is flagged in that case.

### 3.4 The limit: slow, coherent movers

A box carrying 30 % of the features, 2 m away, **drifting at 0.1 m/s**, moves 1 cm per keyframe.
That passes every gate. Over the window, the estimate partly *follows* the box, and the velocity
and accelerometer bias absorb the small mismatch the IMU would otherwise show. Measured residuals
after the solve: median 0.83 px for box points against 0.73 px for static ones. Even the
pre-solve, IMU-predicted residuals differ only by about 2× (2.0 vs. 1.6 px), because the predicted
velocity has already absorbed the drift.

An accumulated, IMU-referenced consistency score per landmark was tried and **measured no effect**
for exactly that reason.

| Drift speed of the box | 0.05 | 0.1 | 0.2 | 0.3 | 0.5 | 1.0 m/s |
|---|---|---|---|---|---|---|
| RMSE with M4 / oracle | 3.8× | 7.2× | 16× | 27× | 1.03× | 1.00× |

(Simulation, 30 % of features on the moving box. *Oracle* means its features removed.) From walking
speed upward the movers are handled. Below it, geometry and IMU cannot tell "the box moves" from
"we move", and **prior knowledge** is needed: a semantic mask (§5).

---

## 4. The dynamic heat grid (spatial memory of motion)

A moving object rarely contributes one bad feature. It contributes a cluster, and it will keep
doing so in the next frames. Removing landmarks one at a time after they have already been used
is reactive. Kestrel adds a cheap **spatial memory**: a coarse grid over the image
(`grid_rows × grid_cols`, default 6×8) with a "heat" $h_c\in[0,1]$ per cell:

$$
h_c \leftarrow \alpha\,h_c + (1-\alpha)\,\frac{n^{\text{outlier}}_c}{n^{\text{features}}_c + 1},
\qquad \alpha = \texttt{heat\_decay}.
$$

* Outliers concentrated in a cell heat it up quickly. Isolated outliers spread across the image
  barely register.
* Cells with $h_c > $ `heat_mask_threshold` are **masked**. The front-end detects no *new* features
  there, so it stops feeding the estimator points that are likely to be on the moving object.
* Heat decays geometrically (with $\alpha = 0.8$, a cell cools to ~10% in about 10 frames), so a
  region becomes usable again shortly after the object leaves.

This is a heuristic, not a probabilistic model. It is deliberately simple, cheap (48 numbers),
and easy to visualize for debugging. Because the grid lives in image coordinates, it naturally
tracks objects that stay in roughly the same part of the image over a few frames. For an
object sweeping fast across the view, the per-landmark gating of §3 does the work.

---

## 5. Semantic masks (optional)

Learned segmentation (e.g. instance segmentation of *person*, *car*, *forklift*) provides
**prior knowledge** that a pixel is likely to move, *before* it does:

* **Pros:** it catches slow movers and objects that are momentarily still but will move. It
  protects the map from people standing next to a shelf.
* **Cons:** a parked car is static and full of good features, and masking it throws information
  away. Unknown movable classes (carts, pallets) are missed. It costs GPU time.

Kestrel treats semantics as **optional input**: any node can publish a binary mask aligned with
the left IR image, and the front-end excludes masked pixels from detection and tracking (DEV_PLAN
M4). This keeps the core free of a deep-learning runtime while allowing YOLO-seg or similar on a
Jetson.

---

## 6. Literature overview

* **DynaSLAM** [Bescos18] combines Mask R-CNN segmentation with multi-view geometric checks on
  ORB-SLAM2, and inpaints the static background.
* **DS-SLAM** [Yu18] combines semantic segmentation with a moving-consistency check based on
  epipolar distances.
* **StaticFusion** [Scona18] jointly estimates camera motion and a per-cluster static/dynamic
  segmentation in dense RGB-D SLAM.
* **Survey:** [Saputra18] classifies approaches into robust estimation, motion segmentation,
  semantic filtering and dynamic-object tracking.

Kestrel sits in the "robust estimation + geometric consistency with IMU" family, with
semantics as a plug-in. That choice favours generality (unknown object classes) and compute
budget (Jetson-class hardware).

---

## 7. Relation to the rest of the system

| Timescale | Example | Mechanism | Chapter |
|-----------|---------|-----------|---------|
| Within a window (seconds) | person walking by | robust kernel, χ² gating, heat grid, masks | this chapter |
| Between visits (minutes–weeks) | pallet moved, door left open | persistence filter on map points | [09](09_lifelong_mapping.md) |
| Dense geometry | box removed from a shelf | occupancy with clamping + change events | [10](10_volumetric_mapping.md) |

---

## 8. In Kestrel

| Concept | Where |
|---------|-------|
| Parameters | `dynamic:` section of the config (`enable: false` = plain M3 gates, for ablations) |
| IMU-predicted gate, post-solve gate, feedback of track ids | `VioEstimator::addVisualFactors`, `collectOutliers` (`kestrel/estimator/vio_estimator.hpp`) |
| Confirmation delay, factor removal, avalanche guard (§3.1–3.3) | `VioEstimator` (`flagLandmark`, `dynamic.min_confirmation_s`, `max_flag_fraction`) |
| Heat grid (§4) and detection-only mask | `kestrel/frontend/dynamic_heat_grid.hpp`; `FeatureTracker::track(..., detection_mask)`; wired in `VisualInertialOdometry` |
| External semantic masks (§5) | `VisualInertialOdometry::addStereo(t, left, right, mask)`; offline `run_vio --masks DIR` (one PNG per frame); ROS topic with the node (M9) |
| Synthetic moving objects | `test/sim_world.hpp` (`dynamic_*` options), `test/test_dynamic.cpp` |
| Moving object painted into real images | `kestrel/eval/dynamic_overlay.hpp`, `run_vio --inject-dynamic [--oracle-mask]` |
| On/off ablation table | `tools/dynamic_ablation.py` (`docs/benchmarks.md`) |

**Measured** (simulation, 30 % of features on the mover; RMSE against an oracle without them):
a box travelling with the camera, 1.00× (M3 without these measures: 6.2 m, diverged);
boxes walking at 0.5 m/s, 1.08× (M3: 31×); 100 % of their tracks neutralized. Slow coherent
movers are the known limit (§3.4). On real images with a large injected object, see
`docs/benchmarks.md`.

**Observation:** on the injected-object test the heat grid never heats up. The front-end's
F-matrix RANSAC and the estimator's gates drop the object's tracks before enough of them are
flagged in one cell. Recordings with people walking (roadmap D8) will show whether it matters
in practice.

---

## 9. Pitfalls

* **Gating with a wrong noise model.** If $\sigma_{px}$ is too small, half the static world is
  "dynamic". Calibrate the threshold on a static sequence first.
* **Masking too aggressively** in feature-poor environments leaves the estimator starving.
  Monitor the feature count and the fraction of the image masked.
* **Removing landmarks during the initializer.** Before the window has converged, residuals are
  large for everyone. Only gate once initialized.
* **Treating every outlier as motion.** Mismatches from repetitive texture also cluster
  spatially (e.g. on a grid of identical shelves) and will heat the grid. That is harmless, but
  it is not evidence of motion.

---

## 10. Exercises

1. A 3-dof stereo residual has whitened squared error $e = 9$. Is it an outlier at 95%? At 99%?
   What is the probability that a *static* point produces $e > 9$?
2. With $\alpha = 0.8$ and a constant outlier ratio of 0.6 in a cell, what is the steady-state
   heat? How many frames does it take to cross a threshold of 0.5 starting from 0?
3. Construct a 2-D toy (40 static points, 60 points on one rigid mover) for which F-matrix
   RANSAC returns the mover's motion. Add a gyro rotation prior: how does the outcome change?
4. Argue why a semantic mask is *complementary* to persistence filtering ([Ch. 09](09_lifelong_mapping.md))
   rather than a replacement for it.
