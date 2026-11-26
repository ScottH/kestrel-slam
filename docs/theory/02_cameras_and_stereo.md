# 02 — Cameras and Stereo Geometry

> **Why this chapter exists.** Every visual measurement Kestrel uses is a pixel, and turning
> pixels into geometry needs a model of the lens (projection and distortion) and of the two-camera
> rig (epipolar geometry, triangulation). This chapter also covers the properties of the Orbbec
> Gemini 335L(g) that change how a SLAM system must use it.

Primary references: **[Hartley04]** (the standard text), **[Brown66]** and **[Kannala06]**
(distortion models), **[HartleySturm97]** (triangulation), **[Furgale13]** (calibration with Kalibr).

---

## 1. The pinhole model

A camera in the optical frame $C$ ($x$ right, $y$ down, $z$ forward) maps a point
$\mathbf{P}_C = (X, Y, Z)^\top$ with $Z>0$ to **normalized coordinates** and then to **pixels**:

$$
\bar{\mathbf{x}} = \begin{pmatrix}x\\y\\1\end{pmatrix} = \frac{1}{Z}\begin{pmatrix}X\\Y\\Z\end{pmatrix},
\qquad
\begin{pmatrix}u\\v\\1\end{pmatrix} = \mathbf{K}\bar{\mathbf{x}},
\qquad
\mathbf{K} = \begin{pmatrix}f_x & 0 & c_x\\ 0 & f_y & c_y\\ 0&0&1\end{pmatrix}.
$$

$f_x, f_y$ are focal lengths in pixels and $(c_x, c_y)$ is the principal point. Two facts to remember:

* **Depth is lost.** Every point on the ray $s\,\bar{\mathbf{x}}$, $s>0$, projects to the same pixel.
  One camera measures *directions*, not distances.
* **Normalized coordinates are camera-independent.** Kestrel's estimator works with
  $\bar{\mathbf{x}}$, so the back-end never needs to know about lenses. A measurement noise of
  $\sigma_{px}$ pixels becomes $\sigma_{px}/f$ in normalized units. That is the scale factor in
  the reprojection residual of [Ch. 06](06_sliding_window_vio.md).

---

## 2. Lens distortion

Real lenses bend straight lines. The models below act **in normalized coordinates**, between
$\bar{\mathbf{x}}$ and $\mathbf{K}$.

### 2.1 Radial–tangential ("radtan", Brown–Conrady) — `pinhole-radtan`

With $r^2 = x^2+y^2$,

$$
\begin{aligned}
x_d &= x\,(1 + k_1r^2 + k_2r^4 + k_3r^6) + 2p_1xy + p_2(r^2 + 2x^2),\\
y_d &= y\,(1 + k_1r^2 + k_2r^4 + k_3r^6) + p_1(r^2 + 2y^2) + 2p_2xy.
\end{aligned}
$$

The $k_i$ terms model radial distortion (barrel or pincushion). The $p_i$ terms model decentering,
where the lens elements are not perfectly aligned with the sensor. This is OpenCV's default model,
and Kalibr calls it `radtan`. It suits the Gemini 335L's IR lenses (moderate field of view).

### 2.2 Equidistant (Kannala–Brandt) — `pinhole-equidistant`

For wide-angle and fisheye lenses, distortion is better expressed in the angle
$\theta = \arctan r$ from the optical axis:

$$
\theta_d = \theta\,(1 + k_1\theta^2 + k_2\theta^4 + k_3\theta^6 + k_4\theta^8),\qquad
(x_d, y_d) = \frac{\theta_d}{r}(x, y).
$$

This is OpenCV's `fisheye` model and Kalibr's `equidistant` [Kannala06]. TUM-VI
([Ch. 11](11_evaluation.md)) uses it.

### 2.3 Undistortion

Projection (distortion) has a closed form, but its inverse does not. Undistorting a pixel means
solving $d(\bar{\mathbf{x}}) = \bar{\mathbf{x}}_d$ for $\bar{\mathbf{x}}$, either by fixed-point
iteration or by a few Gauss–Newton steps starting from $\bar{\mathbf{x}} = \bar{\mathbf{x}}_d$.
OpenCV's `cv::undistortPoints` and `cv::fisheye::undistortPoints` do this. Kestrel undistorts
**feature points only**, never whole images: it is cheaper, and it avoids resampling the image,
which would blur the corners we track.

---

## 3. Two views: epipolar geometry

Two cameras observe the same point $\mathbf{P}$. Let $(\mathbf{R}, \mathbf{t}) = \mathbf{T}_{C_2C_1}$,
so that $\mathbf{P}_2 = \mathbf{R}\mathbf{P}_1 + \mathbf{t}$. The vectors $\mathbf{P}_2$, $\mathbf{t}$
and $\mathbf{R}\mathbf{P}_1$ are coplanar, so
$\mathbf{P}_2\cdot(\mathbf{t}\times\mathbf{R}\mathbf{P}_1) = 0$. Dividing by the depths gives the
**epipolar constraint**:

$$
\bar{\mathbf{x}}_2^\top\,\mathbf{E}\,\bar{\mathbf{x}}_1 = 0,\qquad \mathbf{E} = \mathbf{t}^\wedge\mathbf{R}\quad\text{(essential matrix)}.
$$

In pixels, $\mathbf{x}_2^\top\mathbf{F}\mathbf{x}_1 = 0$ with $\mathbf{F} = \mathbf{K}_2^{-\top}\mathbf{E}\mathbf{K}_1^{-1}$
(the **fundamental matrix**). Geometrically, a point in image 1 must appear somewhere on a *line*
in image 2: $\boldsymbol\ell_2 = \mathbf{E}\bar{\mathbf{x}}_1$.

Kestrel uses the constraint in two ways:

1. **Stereo match check.** For the calibrated stereo pair, $\mathbf{E}$ is known exactly from
   $\mathbf{T}_{C_RC_L} = \mathbf{T}_{BC_R}^{-1}\mathbf{T}_{BC_L}$. A left–right match is rejected if
   the right point lies more than a threshold from its epipolar line:
   $$d = \frac{|\bar{\mathbf{x}}_R^\top\mathbf{E}\bar{\mathbf{x}}_L|}{\sqrt{\ell_1^2 + \ell_2^2}}\cdot f
   \ \ > \ \tau_{\text{epi}}\ (\text{config: } \texttt{stereo\_epipolar\_threshold\_px}).$$
   This works on **unrectified** images, so Kestrel never warps the IR frames.
2. **Temporal outlier rejection.** Between consecutive frames the motion is unknown, so $\mathbf{F}$
   is estimated robustly with RANSAC ([Ch. 04](04_feature_tracking.md)). Features inconsistent with
   the dominant motion are discarded. Many of those lie on independently moving objects
   ([Ch. 07](07_dynamic_objects.md)).

---

## 4. Triangulation

### 4.1 Rectified stereo: depth from disparity

For an ideal rectified pair, with parallel optical axes and baseline $b$ along $x$, a point at
depth $Z$ has **disparity** $d = u_L - u_R = f b / Z$, so

$$
Z = \frac{f\,b}{d}.
$$

**Depth uncertainty grows quadratically.** Propagating a disparity error $\sigma_d$ gives
$\sigma_Z = |\partial Z/\partial d|\,\sigma_d = \frac{fb}{d^2}\sigma_d$, that is

$$
\boxed{\ \sigma_Z = \frac{Z^2}{f\,b}\,\sigma_d\ }
$$

For the Gemini 335L at 848×480 ($f\approx430$ px nominal, $b = 0.095$ m, so $fb\approx 40.9$ px·m)
and a sub-pixel matching accuracy of $\sigma_d = 0.25$ px:

| Depth $Z$ | 1 m | 2 m | 5 m | 10 m | 20 m |
|-----------|-----|-----|-----|------|------|
| $\sigma_Z$ | 0.6 cm | 2.4 cm | 15 cm | 61 cm | 2.4 m |

The consequence matters for design. Stereo gives excellent *metric scale* and *near* geometry,
but far points constrain depth weakly. Far points still constrain **rotation** well. This is why
Kestrel keeps far points but parameterizes them by inverse depth ([Ch. 06](06_sliding_window_vio.md)),
and why the dense map caps the integration range at ~5 m ([Ch. 10](10_volumetric_mapping.md)).

### 4.2 General two-view (and multi-view) triangulation: linear DLT

For views with camera-from-world matrices $\mathbf{P}_k = [\mathbf{R}_k\ |\ \mathbf{t}_k]$ (using
normalized coordinates, so no $\mathbf{K}$), each observation $\bar{\mathbf{x}}_k = (x_k, y_k, 1)$
satisfies $\bar{\mathbf{x}}_k \times (\mathbf{P}_k\tilde{\mathbf{X}}) = \mathbf{0}$ for the homogeneous
point $\tilde{\mathbf{X}}$. Two independent rows per view:

$$
\begin{pmatrix}
x_k\,\mathbf{P}_k^{(3)} - \mathbf{P}_k^{(1)}\\
y_k\,\mathbf{P}_k^{(3)} - \mathbf{P}_k^{(2)}
\end{pmatrix}\tilde{\mathbf{X}} = \mathbf{0},
$$

where $\mathbf{P}^{(i)}$ is the $i$-th row. Stack the rows of all views into $\mathbf{A}$, and take
$\tilde{\mathbf{X}}$ as the right singular vector of $\mathbf{A}$ with the smallest singular value.
Then check:

* **Cheirality.** The depth must be positive in every view.
* **Parallax.** The angle between the viewing rays must be large enough. Below ~1° the result is
  numerically meaningless.
* **Reprojection error.** It must be below a few pixels.

DLT minimizes an algebraic error, not the geometric one. That is fine as an *initial value*,
because the subsequent bundle adjustment minimizes the true reprojection error.
[HartleySturm97] gives the optimal two-view method if you ever need it.

---

## 5. The Orbbec Gemini 335L / 335Lg as a SLAM sensor

The 335Lg is the GMSL2 (automotive serializer/FAKRA connector) variant of the USB 335L. The
optical part is the same. Properties that matter for SLAM (check the current Orbbec datasheet for
exact figures):

* **Two global-shutter IR cameras, ~95 mm apart.** Global shutter means every pixel is exposed at
  the same instant, so fast motion does not shear the image as it does with a rolling shutter.
  This is a big advantage for visual–inertial odometry. The wide baseline (compared with ~50 mm on
  many RGB-D cameras) improves depth accuracy by the $\sigma_Z\propto 1/b$ law above.
* **Hardware-synchronized IR pair.** Left and right frames carry the same timestamp, so stereo
  matching needs no temporal interpolation. The capture monitor (`kestrel_capture`) verifies this.
* **Internal IMU** (accelerometer + gyroscope), time-stamped by the same device clock as the
  images. The camera–IMU time offset should be near zero. It still needs calibrating
  ([Ch. 03](03_imu_and_preintegration.md), [Furgale13]).
* **An IR dot-pattern projector ("laser").** It adds texture so that the on-board stereo depth
  engine works on blank walls. **For tracking it is harmful.** The dots are rigidly attached to the
  *camera*, so features on them move with the camera and look perfectly static in the image. A
  tracker that follows them estimates zero motion. Kestrel's options (see the capture launch file):
  * projector **off**: clean images at full rate, with passive-stereo depth only;
  * **interleaved** mode: alternate frames with the projector on and off. Track on the "off"
    frames and map with the depth from the "on" frames, at half the effective tracking rate.
* **Outdoors**, sunlight contains strong near-IR. The projector becomes irrelevant, images can
  saturate, and auto-exposure matters. Passive stereo usually works well on outdoor texture.

---

## 6. Calibration in one paragraph

Intrinsics, distortion, stereo extrinsics, camera–IMU extrinsics $\mathbf{T}_{BC}$ and the time
offset are best estimated *jointly* with Kalibr [Furgale13]. Kalibr models the trajectory as a
continuous-time B-spline and minimizes reprojection and IMU errors while you wave the camera in
front of an AprilGrid target. Factory calibration from the driver's `camera_info` and TF is a
usable starting point, but a joint calibration typically improves VIO accuracy noticeably,
especially $\mathbf{T}_{BC}$. The hardware guide (`docs/hardware/`) gives the recording recipe.

---

## 7. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| `pinhole-radtan` and `pinhole-equidistant` camera models (project, undistort, Jacobians) | `kestrel/sensors/camera.hpp` — **planned (M1)** |
| Stereo KLT matching + epipolar check | `kestrel/frontend/feature_tracker.hpp` — **planned (M2)** |
| DLT triangulation with cheirality, parallax and reprojection checks | `kestrel/estimator/feature_manager.hpp` — **planned (M3)** |
| Camera parameters and extrinsics | `camera:` section of the YAML config (`config.hpp`, available now) |
| Laser modes / capture settings | `kestrel_capture/launch/capture.launch.py` (M0) |

---

## 8. Pitfalls

* **Wrong extrinsic direction.** `T_imu_left` maps *camera* points into the *IMU* frame. Kalibr
  reports `T_cam_imu`, the inverse. Swapping them produces an estimator that diverges under
  rotation.
* **Distortion model mismatch.** Feeding equidistant coefficients to a radtan model "almost works"
  near the image centre and fails at the edges, where the useful wide-baseline features are.
* **Projector on while tracking.** You get a near-zero velocity estimate that slowly fights the
  IMU. Check the IR images for dots first.
* **Rectifying the images** before tracking interpolates pixels, and the result has a different
  $\mathbf{K}$. If you do rectify, use the rectified intrinsics consistently.
* **Assuming depth is reliable far away.** At 20 m the depth noise can exceed 2 m (§4.1). Treat
  far points as bearing-only.

---

## 9. Exercises

1. Derive $\sigma_Z$ for a non-rectified pair by linearizing the DLT solution. Under what
   condition does it reduce to $Z^2\sigma_d/(fb)$?
2. A feature at the image corner of an 848×480 camera has $r\approx 1.1$ in normalized units.
   With $k_1 = -0.05$, $k_2 = 0.01$, how many pixels does ignoring distortion cost at
   $f = 430$ px?
3. Write the essential matrix of the nominal Gemini rig from the default config
   ($\mathbf{T}_{BC_L} = \mathbf{I}$, $\mathbf{t}_{BC_R} = (0.095, 0, 0)$). What do the epipolar lines look like?
4. Explain, using the epipolar constraint, why features on the projector's dots satisfy
   $\bar{\mathbf{x}}_{t+1} = \bar{\mathbf{x}}_t$ regardless of camera motion. What does a RANSAC
   fundamental-matrix estimator conclude when most features are such dots?
