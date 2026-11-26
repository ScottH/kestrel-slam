# Kestrel SLAM — Theory Notes

This folder is the **textbook** that goes with the code. Kestrel is meant to be read as well as run,
and every module in `kestrel_core` has a chapter here that explains *why* it works before
`docs/tutorial/` explains *how the code does it*.

Each chapter follows the same structure:

1. **Intuition** — the idea in plain words, usually with a picture in your head.
2. **Derivation** — the mathematics, with consistent notation (see below).
3. **In Kestrel** — which module and milestone implements it (see [`../DEV_PLAN.md`](../DEV_PLAN.md)).
   Most code is still *planned*. Those sections describe the design the code will follow.
4. **Pitfalls** — the mistakes that cost real SLAM developers days.
5. **Exercises** — small things to derive, compute or try. Several can be done with the unit tests
   and synthetic simulator once they exist.

Citations like **[Forster17]** refer to [`../REFERENCES.md`](../REFERENCES.md).

---

## Chapters

| # | Chapter | Covers | Used by milestone |
|---|---------|--------|-------------------|
| 01 | [Lie groups for rotations and poses](01_lie_groups.md) | SO(3), SE(3), Exp/Log, Jacobians, quaternions, optimization on manifolds | M1, everything after |
| 02 | [Cameras and stereo geometry](02_cameras_and_stereo.md) | Pinhole + distortion, epipolar geometry, triangulation, depth uncertainty, Gemini 335L specifics | M1, M2 |
| 03 | [IMU modelling and preintegration](03_imu_and_preintegration.md) | Noise model, Allan variance, on-manifold preintegration, bias correction, IMU residual | M1, M3 |
| 04 | [Feature detection and tracking](04_feature_tracking.md) | Shi–Tomasi, pyramidal KLT, forward–backward check, gyro-aided prediction, RANSAC | M2 |
| 05 | [Nonlinear least squares](05_nonlinear_least_squares.md) | MAP estimation, Gauss–Newton, Levenberg–Marquardt, robust kernels, factor graphs, Schur complement | M3 |
| 06 | [Sliding-window visual–inertial odometry](06_sliding_window_vio.md) | State, factors, keyframes, marginalization, consistency/observability, initialization | M3 |
| 07 | [Dynamic objects](07_dynamic_objects.md) | Robust statistics limits, χ² gating, dynamic heat grid, semantic masks | M4 |
| 08 | [Place recognition and loop closure](08_place_recognition_and_loop_closure.md) | Bag of words, vocabulary trees, geometric verification, 4-DoF pose graph, map merging | M5, M6 |
| 09 | [Lifelong mapping in changing environments](09_lifelong_mapping.md) | Semi-static worlds, the persistence filter, occlusion vs disappearance, pruning | M7 |
| 10 | [Volumetric mapping and change detection](10_volumetric_mapping.md) | Log-odds occupancy, clamping, ray casting, voxel hashing, change events, 2D grids | M8 |
| 11 | [Evaluating SLAM honestly](11_evaluation.md) | ATE/RPE, Umeyama alignment, datasets, timing, reporting | M10 |

### Suggested reading order

* **"I want to understand the whole system"** — read 01 → 02 → 03 → 05 → 04 → 06 → 08 → 07 → 09 → 10 → 11.
  Chapter 05 (optimization) comes before 04 because the tracking chapter is easier once you have
  seen Gauss–Newton.
* **"I only care about mapping changing environments"** — skim 01, then 08 → 09 → 10.
* **"I need to tune it on my robot"** — 02 (camera and laser issues) → 03 (IMU noise) → 07 → 11.

Prerequisites: linear algebra (eigen-decomposition, SVD, least squares), basic probability
(Gaussians, Bayes' rule) and multivariable calculus. No prior robotics knowledge is assumed.

---

## Notation and conventions (used in every chapter and in the code)

These match `kestrel_core/include/kestrel/config.hpp`. When a formula from a paper uses a
different convention, the chapter converts it and says so.

| Symbol / rule | Meaning |
|---------------|---------|
| $W$ | **World** frame: fixed, gravity-aligned, $z$ axis **up**. Origin and yaw are arbitrary (set at start-up). |
| $B$ | **Body** frame = the **IMU** frame. The estimator tracks the body. |
| $C$, $C_L$, $C_R$ | **Camera** frame (left/right), *optical* convention: $x$ right, $y$ down, $z$ forward. |
| $\mathbf{T}_{AB} \in SE(3)$ | Transform that maps **points expressed in $B$ into $A$**: $\mathbf{p}_A = \mathbf{T}_{AB}\,\mathbf{p}_B$. In code: `T_a_b`. Composition chains naturally: $\mathbf{T}_{AC} = \mathbf{T}_{AB}\mathbf{T}_{BC}$. |
| $\mathbf{R}_{AB}, \mathbf{t}_{AB}$ | Rotation and translation of $\mathbf{T}_{AB}$; $\mathbf{t}_{AB}$ is the origin of $B$ expressed in $A$. |
| $\mathbf{T}_{WB_k}$ or $(\mathbf{R}_k, \mathbf{p}_k)$ | Pose of the body at time/keyframe $k$. $\mathbf{v}_k$ is its velocity in $W$. |
| $\mathbf{T}_{BC}$ | Camera–IMU extrinsics (`T_imu_left`, `T_imu_right` in the config). |
| $\mathbf{g}_W = (0, 0, -g)^\top$ | Gravity **acceleration** in $W$, $g \approx 9.81\ \mathrm{m/s^2}$. A resting accelerometer therefore reads $+g$ along the body's "up" axis. |
| $(\cdot)^\wedge$, $[\cdot]_\times$ | Skew-symmetric (hat) matrix of a 3-vector: $\mathbf{a}^\wedge\mathbf{b} = \mathbf{a}\times\mathbf{b}$. $(\cdot)^\vee$ is its inverse. |
| $\mathrm{Exp}, \mathrm{Log}$ | Maps between the vector space $\mathbb{R}^3$ and $SO(3)$ (capitalized = vector arguments, see [Ch. 01](01_lie_groups.md)). |
| **Right perturbation** | Rotation errors are applied *locally*: $\mathbf{R} \boxplus \delta\boldsymbol{\theta} = \mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol{\theta})$, and for quaternions $\mathbf{q}\otimes\mathrm{Exp}_q(\delta\boldsymbol\theta)$. Positions are perturbed additively: $\mathbf{p} + \delta\mathbf{p}$. |
| Quaternions | Hamilton convention, $\mathbf{q} = (q_w, \mathbf{q}_v)$, $\mathbf{q}$ and $-\mathbf{q}$ are the same rotation. Eigen stores coefficients as $(x, y, z, w)$. |
| $\mathbf{b}_g, \mathbf{b}_a$ | Gyroscope and accelerometer biases. |
| $\pi(\cdot)$ | Camera projection: 3-D point in the camera frame → pixel. |
| $\bar{\mathbf{x}} = (x, y, 1)^\top$ | **Normalized** image coordinates: undistorted, divided by depth, independent of focal length. |
| $\lambda$ | Inverse depth of a landmark in its anchor camera. |
| $\|\mathbf{r}\|^2_{\boldsymbol\Sigma}$ | Squared Mahalanobis norm $\mathbf{r}^\top\boldsymbol\Sigma^{-1}\mathbf{r}$. |
| $\hat{(\cdot)}$, $\tilde{(\cdot)}$ | Estimate / measurement. A bar $\bar{(\cdot)}$ marks a *linearization point*. |
| Units | SI everywhere (m, s, rad, m/s², rad/s), pixels for image quantities. |

### A worked example of the transform convention

The left camera's optical centre in the world frame is
$\mathbf{T}_{WC_L} = \mathbf{T}_{WB}\,\mathbf{T}_{BC_L}$. A point $\mathbf{p}_W$ seen by that camera has
camera coordinates
$\mathbf{p}_{C} = \mathbf{T}_{WC_L}^{-1}\mathbf{p}_W = \mathbf{R}_{BC}^\top\left(\mathbf{R}_{WB}^\top(\mathbf{p}_W - \mathbf{p}_{WB}) - \mathbf{t}_{BC}\right)$.
If the subscripts "cancel" from the inside out ($W\!B \cdot B\!C = W\!C$), the expression is
consistent. Reading subscripts this way catches most frame bugs before they reach the code.
