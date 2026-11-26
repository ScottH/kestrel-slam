# 06 — Sliding-Window Visual–Inertial Odometry

> **Why this chapter exists.** This is the heart of Kestrel. It combines the camera model
> ([Ch. 02](02_cameras_and_stereo.md)), IMU preintegration ([Ch. 03](03_imu_and_preintegration.md))
> and nonlinear least squares ([Ch. 05](05_nonlinear_least_squares.md)) into an estimator that runs
> in real time forever, by optimizing only a **window** of recent keyframes and summarizing
> everything older in a **prior**.

Primary references: **[Leutenegger15]** (OKVIS, keyframe-based VIO with marginalization),
**[Qin18]** (VINS-Mono, whose structure Kestrel follows most closely), **[Sibley10]**,
**[DongSi11]** (fixed-lag smoothing and its consistency), **[Huang10]**, **[Hesch14]** (FEJ,
observability), **[Kelly11]** (observability of VIO), **[Strasdat12]** (why optimization beats
filtering at equal cost), **[Mourikis07]** (MSCKF, the filtering alternative).

---

## 1. Intuition

Full batch optimization over all frames ever seen is the most accurate option, but its cost grows
forever. A Kalman filter keeps only the current state and linearizes each measurement once, which
is cheap but loses accuracy. A **sliding window** is the compromise: re-linearize and optimize the
last $N$ keyframes jointly (accurate), and when a keyframe leaves the window, convert its
information into a Gaussian **prior** on the keyframes that remain. This step is
**marginalization**. Strasdat et al. [Strasdat12] showed that, for a given compute budget,
optimizing over more frames beats filtering with more features. Most current
state-of-the-art VIO is optimization-based. OpenVINS [Geneva20] is a strong filter-based exception.

---

## 2. The state

For each keyframe $k = 0, \dots, N$ in the window ($N$ = `window_size`, default 10):

$$
\mathbf{x}_k = \big(\mathbf{p}_k,\ \mathbf{q}_k,\ \mathbf{v}_k,\ \mathbf{b}_{a,k},\ \mathbf{b}_{g,k}\big)
\quad\in\ \mathbb{R}^3\times SO(3)\times\mathbb{R}^9 ,
$$

which is 15 tangent-space dimensions. In code this is stored as two parameter blocks: a 7-number
pose with the manifold of [Ch. 01](01_lie_groups.md) §6, and a 9-number "speed-bias" block. In
addition there is one **inverse depth** $\lambda_l$ per landmark, relative to its *anchor*
keyframe (the first keyframe that observed it). Optionally the camera–IMU extrinsics
$\mathbf{T}_{BC}$ can be estimated too (`estimate_extrinsics`).

### 2.1 Why inverse depth?

A landmark is stored as its normalized observation $\bar{\mathbf{x}}_a$ in the anchor camera,
which is a fixed measurement, plus one number $\lambda = 1/Z$. Three reasons:

* **One parameter instead of three.** The direction is already well known from the measurement,
  so the Schur complement ([Ch. 05](05_nonlinear_least_squares.md) §6) is cheaper.
* **Far points are fine.** A point at infinity has $\lambda = 0$, a perfectly valid finite value.
  It still constrains rotation.
* **Better linearity.** Reprojection error is closer to linear in $\lambda$ than in $Z$, especially
  for distant points, so Gaussian assumptions hold better.

---

## 3. The cost function

$$
\min_{\mathcal{X}}\ \underbrace{\|\mathbf{r}_{\text{prior}}(\mathcal{X})\|^2}_{\text{marginalization}}
+ \sum_{k}\underbrace{\|\mathbf{r}_{\mathcal{I}_{k,k+1}}\|^2_{\boldsymbol\Sigma_{k,k+1}}}_{\text{IMU, Ch. 03}}
+ \sum_{(l,j)}\rho\Big(\underbrace{\|\mathbf{r}_{\mathcal{C}_{lj}}\|^2_{\boldsymbol\Sigma_{\mathcal{C}}}}_{\text{reprojection}}\Big).
$$

### 3.1 Reprojection factor

Landmark $l$ is anchored in keyframe $a$, with left-camera observation $\bar{\mathbf{x}}_{a}$ and
inverse depth $\lambda$. It is observed in keyframe $j$, in camera $c$ (left or right), at
$\bar{\mathbf{z}}_{j}$. Chain the transforms, reading subscripts right to left:

$$
\begin{aligned}
\mathbf{P}_{C_a} &= \tfrac1\lambda\,\bar{\mathbf{x}}_{a} &&\text{(point in anchor camera)}\\
\mathbf{P}_{W} &= \mathbf{R}_a\big(\mathbf{R}_{BC_L}\mathbf{P}_{C_a} + \mathbf{t}_{BC_L}\big) + \mathbf{p}_a &&\text{(to world)}\\
\mathbf{P}_{C_j} &= \mathbf{R}_{BC_c}^\top\Big(\mathbf{R}_j^\top(\mathbf{P}_W - \mathbf{p}_j) - \mathbf{t}_{BC_c}\Big) &&\text{(into camera } c \text{ of keyframe } j)\\
\mathbf{r}_{\mathcal{C}} &= \frac{f}{\sigma_{px}}\left(\begin{pmatrix}P_x/P_z\\P_y/P_z\end{pmatrix}_{C_j} - \bar{\mathbf{z}}_j\right) &&\text{(whitened)}
\end{aligned}
$$

There are three variants:

* **anchor-left → other keyframe, left or right camera:** depends on $(\mathbf{T}_a, \mathbf{T}_j, \lambda)$;
* **anchor-left → anchor-right** (the stereo pair of the anchor frame): depends **only on $\lambda$**.
  This is the stereo constraint that fixes metric depth immediately.

The Jacobians follow from the chain rule and the two derivative examples in
[Ch. 01](01_lie_groups.md) §3.2. Kestrel will use Ceres automatic differentiation and verify the
result numerically.

### 3.2 IMU factor

This is exactly the residual of [Ch. 03](03_imu_and_preintegration.md) §7, linking
$(\mathbf{x}_k, \mathbf{x}_{k+1})$, weighted by the preintegrated covariance.

---

## 4. Keyframes and the two kinds of marginalization

Each new frame first enters the window as the newest state. Then one state must leave so the
window stays at $N+1$. Which one depends on whether the **second-newest** frame turned out to be a
keyframe:

* **Second-newest is a keyframe** (enough parallax or new features, see below): marginalize the
  **oldest** keyframe. Its information becomes part of the prior, its landmarks are marginalized
  with it, and the landmarks still observed later are re-anchored in the next keyframe.
* **Second-newest is not a keyframe** (camera almost static or barely moving): **drop** it. Its
  visual measurements are discarded (they added little), and its IMU data is concatenated into the
  preintegration between its neighbours, so no inertial information is lost.

**Keyframe criterion** (VINS-style): the frame is a keyframe if the mean parallax of tracked
features relative to the previous keyframe exceeds `keyframe_parallax_px` (rotation-compensated),
or if tracking quality dropped: few tracked features, or many new ones. This keeps the window
spanning real motion. When the robot stands still, the window doesn't fill up with nearly
identical frames that would collapse the baseline.

---

## 5. Marginalization by Schur complement

Linearize every factor connected to the states being removed, $\mathbf{x}_m$, around the current
estimate. Let $\mathbf{x}_r$ be the *remaining* states those factors also touch. The local
quadratic model is

$$
E(\boldsymbol\delta) = \tfrac12\boldsymbol\delta^\top\mathbf{H}\boldsymbol\delta + \mathbf{g}^\top\boldsymbol\delta + c,
\qquad
\mathbf{H} = \begin{pmatrix}\mathbf{H}_{mm} & \mathbf{H}_{mr}\\ \mathbf{H}_{rm} & \mathbf{H}_{rr}\end{pmatrix},\ 
\mathbf{g} = \begin{pmatrix}\mathbf{g}_m\\ \mathbf{g}_r\end{pmatrix},
$$

with $\mathbf{H} = \sum\mathbf{J}_i^\top\mathbf{J}_i$ and $\mathbf{g} = \sum\mathbf{J}_i^\top\mathbf{r}_i$
over the whitened (and robustified) factors. Minimizing over $\boldsymbol\delta_m$ (equivalently,
integrating it out of the Gaussian) leaves

$$
\mathbf{H}^\ast = \mathbf{H}_{rr} - \mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{H}_{mr},\qquad
\mathbf{g}^\ast = \mathbf{g}_r - \mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{g}_m .
$$

**Turning it back into a residual.** Least-squares solvers want residuals and Jacobians, not
$\mathbf{H}^\ast$ and $\mathbf{g}^\ast$. Find $\mathbf{J}', \mathbf{r}'$ with
$\mathbf{J}'^\top\mathbf{J}' = \mathbf{H}^\ast$ and $\mathbf{J}'^\top\mathbf{r}' = \mathbf{g}^\ast$.
Using the eigen-decomposition $\mathbf{H}^\ast = \mathbf{V}\mathbf{S}\mathbf{V}^\top$ and keeping
only eigenvalues above a small threshold (the matrix is rank-deficient along unobservable
directions):

$$
\mathbf{J}' = \mathbf{S}^{1/2}\mathbf{V}^\top,\qquad \mathbf{r}' = \mathbf{S}^{-1/2}\mathbf{V}^\top\mathbf{g}^\ast .
$$

Check: $\mathbf{J}'^\top\mathbf{r}' = \mathbf{V}\mathbf{S}^{1/2}\mathbf{S}^{-1/2}\mathbf{V}^\top\mathbf{g}^\ast = \mathbf{g}^\ast$. ✓

The **prior factor** in later optimizations is then

$$
\mathbf{r}_{\text{prior}}(\mathbf{x}_r) = \mathbf{r}' + \mathbf{J}'\,(\mathbf{x}_r \boxminus \bar{\mathbf{x}}_r),
$$

where $\bar{\mathbf{x}}_r$ is the linearization point at the time of marginalization, and
$\boxminus$ is the manifold minus of [Ch. 01](01_lie_groups.md) §6.

**Fill-in.** Marginalizing a keyframe connects all the remaining states it shared factors with,
so $\mathbf{H}^\ast$ is dense over them. If we marginalized every landmark too, the prior would
become denser and denser. Instead, landmarks anchored in the marginalized keyframe are included,
and landmarks observed only in the dropped non-keyframe are simply discarded, a deliberate small
loss of information that keeps the problem sparse (the same choice as [Leutenegger15] and
[Qin18]).

---

## 6. Consistency, observability and FEJ

**Observability.** With a monocular or stereo camera and an IMU, four directions of the state
can never be determined from the data: **global position (3)** and **rotation about gravity
(yaw, 1)** [Kelly11, Hesch14]. Roll and pitch *are* observable because the accelerometer senses
gravity. Scale is observable thanks to the IMU, and directly from stereo.

**Inconsistency.** The prior was linearized at $\bar{\mathbf{x}}_r$. Later, the other factors are
linearized at newer estimates. Jacobians evaluated at *different* points can make an unobservable
direction appear observable. The estimator then "learns" yaw from nothing, becomes overconfident,
and its estimate drifts in a way its covariance does not admit. This is the classic EKF-SLAM
inconsistency [Huang10].

**First-Estimates Jacobians (FEJ).** Evaluate every Jacobian involving a marginalized-connected
state at its *first* estimate, so all linearizations agree on the null space [Huang10, Hesch14].
Many optimization-based systems, VINS-Mono included, skip strict FEJ and accept slight
inconsistency because re-linearization buys accuracy. Kestrel will start the same way, and the
benchmark milestone (M10) includes an FEJ experiment.

**Gauge in practice.** The window has no factor fixing absolute position or yaw, so they are
anchored by the prior, and at start-up by fixing the first pose's position and yaw. The loop
closure pose graph ([Ch. 08](08_place_recognition_and_loop_closure.md)) works in exactly these
four unobservable dimensions, which is why its corrections never fight the VIO.

---

## 7. Stereo-inertial initialization

Monocular VIO must recover scale, gravity and velocity from motion, which is delicate. Stereo
gives metric depth immediately, so Kestrel's initializer is simpler:

1. **Gravity / attitude.** Average the accelerometer over the first frames, while the capture
   procedure asks for a still start. The mean $\bar{\mathbf{a}}$ points "up" in the body frame.
   Choose $\mathbf{R}_0$ that rotates it onto $(0,0,1)$, with yaw = 0. Biases start at 0 and
   velocity at 0.
2. **Structure and poses.** Triangulate landmarks from the stereo pair of the first frame. For
   each subsequent frame until the window is full, estimate the pose by **PnP** against these
   landmarks, with the IMU prediction as the initial guess, and triangulate new landmarks.
3. **Gyroscope bias.** Compare visual rotations with preintegrated ones. For consecutive frames,
   $\mathrm{Log}\big((\Delta\hat{\mathbf{R}}_{ij}\mathrm{Exp}(\mathbf{J}^R_{ij}\delta\mathbf{b}_g))^\top\mathbf{R}_i^\top\mathbf{R}_j\big)\approx\mathbf{0}$
   is *linear* in $\delta\mathbf{b}_g$ to first order:
   $\mathbf{J}^R_{ij}\,\delta\mathbf{b}_g \approx \mathrm{Log}\big(\Delta\hat{\mathbf{R}}_{ij}^\top\mathbf{R}_i^\top\mathbf{R}_j\big)$.
   Solve the stacked least-squares problem and re-propagate all preintegrations.
4. **Full optimization.** Run the window optimization of §3. Velocities and the accelerometer
   bias converge from the IMU factors.

If the start is not still, step 1 can instead use the visual–inertial alignment of VINS-Mono
[Qin18]. A moving start is planned as a fallback.

---

## 8. Failure detection and recovery

Declare failure, and restart as a new *sequence* ([Ch. 08](08_place_recognition_and_loop_closure.md) §7),
if any of these happen: too few tracked landmarks, implausible bias magnitudes (e.g.
$\|\mathbf{b}_a\| > 2.5$ m/s², $\|\mathbf{b}_g\| > 1$ rad/s), or a per-frame jump in position or
attitude far beyond physical motion. Restarting quickly and re-attaching through place recognition
is far better than letting a diverged estimate corrupt the map.

---

## 9. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| `FeatureManager` (tracks, anchors, triangulation, re-anchoring) | `kestrel/estimator/feature_manager.hpp` — **planned (M3)** |
| Factors: IMU, reprojection (mono/stereo, same-frame stereo) | `kestrel/estimator/factors.hpp` — **planned (M3)** |
| `MarginalizationInfo` + prior factor | `kestrel/estimator/marginalization.hpp` — **planned (M3)** |
| `Estimator` (window, keyframe policy, initializer, failure detection) | `kestrel/estimator/estimator.hpp` — **planned (M3)** |
| Synthetic end-to-end test (ATE < 2 cm) | `test/test_estimator_synthetic.cpp` — **planned (M3)** |

---

## 10. Pitfalls

* **Re-anchoring** a landmark when its anchor is marginalized must transform the *depth*, not
  just relabel the anchor.
* **Marginalizing with outliers still in the window** bakes them permanently into the prior. Run
  outlier rejection *before* marginalization.
* **Robust kernels during marginalization** must be applied consistently: use the same
  re-weighting as in the solve, otherwise the prior is over- or under-confident.
* **Numerical rank.** Threshold small eigenvalues of $\mathbf{H}^\ast$. Inverting noise creates
  absurd priors.
* **A static robot** produces no parallax. Without the non-keyframe policy, the window collapses
  onto one viewpoint and depth becomes unobservable from motion. Stereo saves us here.

---

## 11. Exercises

1. Write out the Jacobian of the anchor-left → anchor-right stereo residual with respect to
   $\lambda$. When is it zero, and what does that mean physically?
2. A 1-D toy: states $x_0, x_1, x_2$ with factors $x_0\approx0$ ($\sigma=1$),
   $x_1 - x_0 \approx 1$, $x_2 - x_1\approx1$ ($\sigma = 0.1$). Marginalize $x_0$ by hand and
   check that the resulting prior on $x_1$ has mean 1 and variance $1 + 0.01$.
3. Show that the VIO cost of §3 is invariant to $\mathbf{p}_k\to\mathbf{p}_k + \mathbf{t}$ and to a
   rotation about the world $z$-axis applied to all poses, velocities and landmarks, but *not* to
   rotations about $x$ or $y$. Which term breaks the invariance?
4. In the gyro-bias step of §7, how many keyframe pairs are needed for $\delta\mathbf{b}_g$ to be
   determined? What motion makes the system ill-conditioned?
