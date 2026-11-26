# 04 — Feature Detection and Tracking

> **Why this chapter exists.** The front-end turns 30 images per second into a few hundred
> *feature tracks*: the same physical point followed from frame to frame and matched across the
> stereo pair. Everything downstream is only as good as these tracks. The front-end must also be
> fast, because it runs on every frame, unlike the optimizer.

Primary references: **[Shi94]** (good features to track), **[Lucas81]** (Lucas–Kanade),
**[Bouguet00]** (pyramidal LK, the OpenCV implementation), **[Kalal10]** (forward–backward
error), **[Fischler81]** (RANSAC), **[Hartley04]** ch. 4 and 11 (robust estimation, F matrix).

---

## 1. Intuition

Two broad designs exist:

* **Detect-and-match:** find keypoints and descriptors (ORB, SIFT, SuperPoint) in every frame, then
  match descriptors. This is robust to large motion, but the descriptors are expensive and
  repeatability limits accuracy.
* **Detect-and-track:** detect corners occasionally, then *track* them with optical flow, which
  follows the actual image patch frame to frame with sub-pixel accuracy. This is very fast and very
  accurate when motion between frames is small, which it is at 30 Hz with a global-shutter camera.

Kestrel, like VINS-Mono, VINS-Fusion and OpenVINS, **tracks** for odometry. It computes ORB
descriptors only on keyframes, for loop closure ([Ch. 08](08_place_recognition_and_loop_closure.md)).

---

## 2. Which points are trackable? Shi–Tomasi corners

Consider the sum of squared differences when a patch $\mathcal{W}$ is shifted by $\mathbf{d}$:

$$
E(\mathbf{d}) = \sum_{\mathbf{x}\in\mathcal{W}}\big(I(\mathbf{x}+\mathbf{d}) - I(\mathbf{x})\big)^2
\approx \mathbf{d}^\top\mathbf{M}\,\mathbf{d},\qquad
\mathbf{M} = \sum_{\mathbf{x}\in\mathcal{W}}\begin{pmatrix}I_x^2 & I_xI_y\\ I_xI_y & I_y^2\end{pmatrix}.
$$

$\mathbf{M}$ is the **structure tensor**. Its eigenvalues $\lambda_1\ge\lambda_2$ describe the patch:

* both small → flat region; a shift changes nothing, so the point cannot be tracked;
* one large → edge; the patch can slide along the edge (the *aperture problem*);
* **both large → corner**, trackable in every direction.

Shi and Tomasi showed that the tracker of §3 is well-conditioned exactly when $\lambda_2$ is large,
so they score points by $\min(\lambda_1,\lambda_2)$ [Shi94]. Harris uses
$\det\mathbf{M} - k\,\mathrm{tr}^2\mathbf{M}$ as a cheaper proxy. OpenCV's `goodFeaturesToTrack`
implements both.

---

## 3. Lucas–Kanade optical flow

**Brightness constancy:** the patch looks the same in the next image $J$, displaced by $\mathbf{d}$:
$J(\mathbf{x}+\mathbf{d}) \approx I(\mathbf{x})$. Minimize

$$
E(\mathbf{d}) = \sum_{\mathbf{x}\in\mathcal{W}}\big(J(\mathbf{x}+\mathbf{d}) - I(\mathbf{x})\big)^2 .
$$

Linearize $J(\mathbf{x}+\mathbf{d}+\Delta) \approx J(\mathbf{x}+\mathbf{d}) + \nabla J^\top\Delta$ and
set the derivative to zero. This is one Gauss–Newton step ([Ch. 05](05_nonlinear_least_squares.md)):

$$
\underbrace{\Big(\sum\nabla J\,\nabla J^\top\Big)}_{\mathbf{M}}\Delta = \sum\nabla J\,\big(I(\mathbf{x}) - J(\mathbf{x}+\mathbf{d})\big),
\qquad \mathbf{d}\leftarrow\mathbf{d}+\Delta .
$$

Iterate until $\|\Delta\|$ is tiny (a few hundredths of a pixel). The matrix being inverted is the
structure tensor of §2, which is why "good features to track" and "tracker convergence" are the
same condition.

### 3.1 Pyramids

The linearization is only valid for displacements smaller than about the window radius. To track
larger motion, build an image **pyramid** in which each level halves the resolution. Solve at the
coarsest level $L$, where a 40-pixel motion is only 5 pixels. Then propagate the estimate down,
$\mathbf{d}^{L-1}_{\text{init}} = 2\,\mathbf{d}^{L}$, and refine at each finer level [Bouguet00].
Kestrel defaults to 3 levels with a 21×21 window (`klt_pyramid_levels`, `klt_window_px`).

### 3.2 Forward–backward check

LK always returns *some* answer, even when the patch was occluded or the match is wrong. A cheap
and very effective test [Kalal10]: track $I\to J$ to get $\mathbf{x}'$, track back $J\to I$ from
$\mathbf{x}'$ to get $\mathbf{x}''$, and reject the track if
$\|\mathbf{x}-\mathbf{x}''\| > \tau_{fb}$ (`klt_fb_threshold_px`, default 0.5 px).
A correct track is reversible; most failures are not.

---

## 4. Gyro-aided prediction

At fast rotations, image motion can exceed what even a pyramid handles. A camera panning at
3 rad/s with $f = 430$ px moves the image by ~43 px per frame at 30 Hz. The gyroscope already tells
us the rotation between frames, so we can **predict** where each feature will be and start LK
from there.

The rotation between consecutive camera frames is
$\mathbf{R}_{C_1C_2} = \mathbf{R}_{BC}^\top\,\mathbf{R}_{B_1B_2}\,\mathbf{R}_{BC}$, with
$\mathbf{R}_{B_1B_2} = \prod_k\mathrm{Exp}((\tilde{\boldsymbol\omega}_k - \hat{\mathbf{b}}_g)\Delta t)$.
If we ignore translation, which is small compared with depth over 33 ms, a point at direction
$\bar{\mathbf{x}}_1$ in frame 1 appears in frame 2 at

$$
\bar{\mathbf{x}}_2 \simeq \mathbf{R}_{C_1C_2}^\top\,\bar{\mathbf{x}}_1
\quad(\text{then divide by the third coordinate}),
\qquad
\mathbf{u}_2 \simeq \mathbf{K}\,\mathbf{R}_{C_2C_1}\mathbf{K}^{-1}\mathbf{u}_1 .
$$

$\mathbf{H}_\infty = \mathbf{K}\mathbf{R}_{C_2C_1}\mathbf{K}^{-1}$ is the **infinite homography**. It
is exact for points at infinity or for pure rotation. In practice: undistort the pixel, rotate,
re-distort, and pass the result to LK as the initial guess (`OPTFLOW_USE_INITIAL_FLOW`). Most of
the motion is now explained, and LK only needs to correct the small translation-induced
residual.

---

## 5. Outlier rejection with RANSAC

Even after the forward–backward check, some tracks are wrong (repetitive texture, specular
highlights) or belong to **moving objects**. Static points in two frames must satisfy the epipolar
constraint $\mathbf{x}_2^\top\mathbf{F}\mathbf{x}_1 = 0$ ([Ch. 02](02_cameras_and_stereo.md)) with a
*single* $\mathbf{F}$. **RANSAC** [Fischler81] finds the $\mathbf{F}$ that the most tracks agree with:

1. Sample a minimal set ($s = 7$ or $8$ correspondences for $\mathbf{F}$).
2. Fit the model and count *inliers*: tracks within $\tau$ pixels of their epipolar line
   (`ransac_threshold_px`).
3. Repeat, keep the best model, and refit on all its inliers.

**How many iterations?** If a fraction $w$ of tracks are inliers, one sample is all-inlier with
probability $w^s$. To find at least one clean sample with confidence $p$:

$$
N = \frac{\log(1-p)}{\log(1-w^s)} .
$$

| Model | $s$ | $w$ | $p$ | $N$ |
|-------|-----|-----|-----|-----|
| Fundamental (8-pt) | 8 | 0.7 | 0.99 | 78 |
| Fundamental (7-pt) | 7 | 0.7 | 0.99 | 54 |
| P3P (loop closure) | 3 | 0.5 | 0.99 | 35 |
| P4P-type | 4 | 0.5 | 0.99 | 72 |

$N$ grows explosively with $s$ and with the outlier ratio. That is why minimal solvers matter,
and why, once IMU rotation is known, some systems use 2-point translation-only RANSAC.

**Limitation (important for [Ch. 07](07_dynamic_objects.md)):** RANSAC picks the motion supported by
the *largest consistent set*. If a bus fills 70% of the view, the "dominant motion" is the bus.
The IMU-aided back-end, not RANSAC, is the defence against that case.

---

## 6. Keeping features well distributed

Pose estimation is best conditioned when features cover the whole image and a range of depths.
A cluster of features in one corner constrains rotation about that corner poorly. Two simple
mechanisms handle this:

* **Minimum distance mask.** When topping up features, mask a disk of radius
  `min_feature_distance_px` around every existing track so that new corners appear in empty
  regions. Process existing tracks **in order of track length** and drop any that fall inside an
  older track's disk. Long tracks are the most valuable because they give the most parallax.
* **Grid quotas** (optional). Split the image into cells and cap the number of features per cell.

Kestrel also masks out regions flagged as dynamic ([Ch. 07](07_dynamic_objects.md)) and the
image border.

**Contrast.** IR images from the Gemini can be dark and low-contrast. **CLAHE** (contrast-limited
adaptive histogram equalization, `use_clahe`) boosts local contrast before detection and tracking.
It applies the same mapping to both stereo images, which preserves brightness constancy between
them.

---

## 7. Stereo matching by tracking

For each left feature, Kestrel runs LK from the left image into the **right** image, initialized
at the left position shifted by a typical disparity. Then it applies:

1. the forward–backward check (right → left),
2. the epipolar check against the calibrated stereo geometry ([Ch. 02](02_cameras_and_stereo.md) §3),
3. a positive-depth check after triangulation.

The surviving right observations give each new landmark an immediate metric depth. This is why
stereo-inertial initialization is fast and has no scale ambiguity.

---

## 8. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| `StereoFeatureTracker`: CLAHE, pyramids, LK with gyro prediction, FB check, F-RANSAC, masks, stereo LK | `kestrel/frontend/feature_tracker.hpp` — **planned (M2)** |
| Output `FeatureFrame` (id → normalized left/right coordinates) | `kestrel/frontend/feature_frame.hpp` — **planned (M2)** |
| Parameters | `frontend:` section of the config (available now) |
| Statistics logged per frame (tracks, new, stereo matches, ms) | **planned (M2)** acceptance: ≥ 150 features/frame on EuRoC MH_01 |

---

## 9. Pitfalls

* **Laser dots** ([Ch. 02](02_cameras_and_stereo.md) §5) are perfect corners and perfectly
  wrong. Disable the projector or use interleaved frames.
* **Auto-exposure jumps** break brightness constancy for one frame. LK fails in bulk, and the
  IMU must carry the estimate. Prefer slow AE or fixed exposure indoors.
* **Too few RANSAC iterations** under a high outlier ratio can return a model fitted to the
  outliers.
* **Tracking on rectified-and-cropped images** while using the raw intrinsics.
* **A feature count that is too high** costs time in the back-end, which scales with landmarks,
  and gains little. 150–250 well-spread tracks is the sweet spot for VIO.

---

## 10. Exercises

1. Show that the LK normal equations are singular for a patch containing a single straight edge.
   Which direction of $\mathbf{d}$ is unobservable?
2. A pyramid with 3 levels (scale factors 1, 1/2, 1/4) and a 21×21 window: roughly what is the
   largest displacement LK can recover without a prediction? With gyro prediction at 3 rad/s?
3. Compute $N$ from the RANSAC table for $w = 0.4$ (a large moving object in view) and $s = 8$.
   What does this suggest about the time budget?
4. Derive the infinite homography $\mathbf{H}_\infty$ from the projection equations, and show it
   maps vanishing points correctly for arbitrary translation.
