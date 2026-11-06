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
current estimate. Kestrel uses a simpler, slightly more conservative statistic in pixels, the
**mean reprojection error** over the landmark's observations in the window
(`outlier_reprojection_px`, default 3 px), and flags landmarks above it as outliers. Flagged
landmarks are removed from the estimator, and their feature IDs are reported back to the
front-end so it stops tracking them.

Gating *after* the solve, and re-solving if many landmarks were removed, is a simple version of
**alternating estimation**: estimate motion with robust weights, classify points, then
re-estimate with the inliers only.

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

| Concept | Where (planned) |
|---------|-----------------|
| Parameters | `dynamic:` section of the config (available now) |
| Landmark gating + feedback of dynamic feature IDs | `kestrel/estimator/dynamic_filter.hpp` — **planned (M4)** |
| Heat grid and front-end mask | `kestrel/estimator/dynamic_filter.hpp`, `kestrel/frontend/feature_tracker.hpp` — **planned (M4)** |
| External semantic mask input (ROS topic) | `kestrel_ros` — **planned (M4/M9)** |
| Synthetic moving-cluster test | `test/test_dynamic.cpp` — **planned (M4)** |

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
