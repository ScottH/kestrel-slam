# 11 — Evaluating SLAM Honestly

> **Why this chapter exists.** "State of the art" is a claim about numbers, and SLAM numbers are
> easy to get subtly wrong: different alignment, different sequences, a lucky run, tuning on the
> test set. This chapter defines the metrics Kestrel reports, how they are computed, and the rules
> we follow so that our numbers can be trusted, by us first of all.

Primary references: **[Sturm12]** (ATE/RPE as standard metrics), **[Zhang18]** (trajectory
evaluation for VIO), **[Umeyama91]** (alignment), **[Grupp17]** (evo, the tool Kestrel uses),
**[Burri16]**, **[Schubert18]**, **[Shi20]** (datasets).

---

## 1. Two questions, two metrics

* **Is the whole trajectory globally correct?** → **Absolute Trajectory Error (ATE)**: align the
  estimate to ground truth once, then measure the position error at every time.
* **How fast does the estimate drift?** → **Relative Pose Error (RPE)**: compare the estimated
  and true *relative* motion over fixed intervals (e.g. every 10 m), independent of where errors
  happened earlier.

A loop-closing SLAM system should be judged mainly on ATE, and odometry (VIO without loops) mainly
on RPE/drift. Kestrel reports both, for VIO-only and full-system runs.

---

## 2. Alignment

The estimate lives in its own world frame (origin and yaw set at start-up,
[Ch. 06](06_sliding_window_vio.md) §6). Before comparing, find the transform that best aligns it
with ground truth. Given associated positions $\{\mathbf{x}_i\}$ (estimate) and $\{\mathbf{y}_i\}$
(ground truth):

$$
\min_{s,\mathbf{R},\mathbf{t}}\ \sum_i\|\mathbf{y}_i - (s\mathbf{R}\mathbf{x}_i + \mathbf{t})\|^2 .
$$

**Umeyama's closed form** [Umeyama91]. With means $\boldsymbol\mu_x, \boldsymbol\mu_y$, variance
$\sigma_x^2 = \frac1n\sum\|\mathbf{x}_i - \boldsymbol\mu_x\|^2$ and cross-covariance
$\boldsymbol\Sigma_{xy} = \frac1n\sum(\mathbf{y}_i - \boldsymbol\mu_y)(\mathbf{x}_i - \boldsymbol\mu_x)^\top = \mathbf{U}\mathbf{D}\mathbf{V}^\top$ (SVD):

$$
\mathbf{S} = \mathrm{diag}\big(1, 1, \det(\mathbf{U}\mathbf{V}^\top)\big),\quad
\mathbf{R} = \mathbf{U}\mathbf{S}\mathbf{V}^\top,\quad
s = \frac{\mathrm{tr}(\mathbf{D}\mathbf{S})}{\sigma_x^2},\quad
\mathbf{t} = \boldsymbol\mu_y - s\mathbf{R}\boldsymbol\mu_x .
$$

The $\mathbf{S}$ matrix prevents a reflection from being returned as a "rotation".

**Which degrees of freedom to align** depends on what the system can observe [Zhang18]:

| Sensor setup | Unobservable | Align with |
|--------------|-------------|------------|
| Monocular visual | 7 (pose + scale) | Sim(3), $s$ free |
| Stereo visual | 6 | SE(3), $s = 1$ |
| Visual–inertial (mono or stereo) | **4** (position + yaw) | **4-DoF** (position + yaw); SE(3) is also common |

Aligning with more freedom than the system has *hides* real errors. Sim(3) alignment of a stereo
system would hide a scale error, for example. Kestrel reports **SE(3)** ATE (so we are comparable
with most published stereo-inertial tables) and states the alignment used in every table.

---

## 3. Definitions

With aligned estimated poses $\hat{\mathbf{T}}_i$ and ground truth $\mathbf{T}_i$:

$$
\mathrm{ATE}_{\text{rmse}} = \sqrt{\frac1N\sum_{i=1}^{N}\big\|\mathrm{trans}\big(\mathbf{T}_i^{-1}\hat{\mathbf{T}}_i\big)\big\|^2}.
$$

For a sub-trajectory length $\Delta$ (distance or time) and pairs $(i, j)$ separated by $\Delta$:

$$
\mathbf{E}_{ij} = \big(\mathbf{T}_i^{-1}\mathbf{T}_j\big)^{-1}\big(\hat{\mathbf{T}}_i^{-1}\hat{\mathbf{T}}_j\big),
\qquad \mathrm{RPE}_{\text{trans}}(\Delta) = \mathrm{RMSE}_{(i,j)}\ \|\mathrm{trans}(\mathbf{E}_{ij})\|.
$$

The rotational RPE uses $\|\mathrm{Log}(\mathrm{rot}(\mathbf{E}_{ij}))\|$. Reporting RPE at several
$\Delta$ (e.g. 7, 14, …, 35 m, as in [Zhang18]) gives a **drift curve**. Drift in percent is
RPE$_{\text{trans}}/\Delta$.

**Time association.** Estimates and ground truth are sampled at different times. Associate
nearest timestamps within a tolerance (e.g. 10 ms), or interpolate ground truth. Check for clock
offsets: a 20 ms offset at 1 m/s is 2 cm of fake error.

---

## 4. Datasets used by Kestrel

| Dataset | Sensors | Ground truth | Why it's in Kestrel's suite |
|---------|---------|--------------|-------------------------------|
| **EuRoC MAV** [Burri16] | global-shutter stereo 20 Hz (752×480), IMU 200 Hz, on a micro aerial vehicle | Vicon / laser tracker | the standard stereo-inertial accuracy benchmark (Machine Hall + Vicon Room sequences) |
| **TUM-VI** [Schubert18] | stereo fisheye 20 Hz (512×512), IMU 200 Hz, handheld | motion capture (full for room sequences, start/end only for long ones) | robustness: long sequences, indoor/outdoor, lighting changes, equidistant camera model |
| **OpenLORIS-Scene** [Shi20] | RealSense cameras (RGB-D, stereo fisheye) with IMUs, odometry, on a ground robot | per-sequence ground truth (mocap or lidar SLAM, depending on scene) | **lifelong**: the same scenes revisited with changes, people and lighting variation, which is exactly Kestrel's target |
| **Own Gemini 335Lg recordings** | the actual sensor | none or partial (loop-closure consistency, start = end) | the real target hardware; before/after-change pairs (`kestrel_capture`) |

(Check each dataset's documentation for exact stream specifications and the official evaluation
protocol before publishing numbers.) OpenLORIS defines metrics beyond ATE, notably a *correct
rate of tracking* that accounts for tracking failures and relocalization. When we report on it, we
follow its protocol.

---

## 5. Measuring time

A SLAM system that is accurate at 2× real-time is not a real-time SLAM system. Report:

* **per-frame front-end and back-end latency:** median, 95th percentile, maximum;
* **keyframe-rate costs** (loop detection, pose graph, voxel integration), which run in other
  threads but must keep up;
* **real-time factor:** processing time divided by sequence duration;
* **hardware:** CPU model, cores used, and whether it was measured on the **Jetson** target.

Latency spikes matter more than averages: one 300 ms stall at the wrong moment loses tracking.

---

## 6. Rules for honest reporting (Kestrel's policy)

1. **Same alignment and same sequences** as the baselines we compare to. Prefer re-running the
   baselines ourselves with their published configurations, and say so when numbers are copied
   from papers.
2. **Multiple runs.** Multithreaded SLAM is not deterministic. Run each sequence ≥ 5 times and
   report the median, plus the spread (or all runs).
3. **Report failures.** A run that loses tracking counts as a failure in the table. It is never
   silently dropped.
4. **No tuning on the test set.** Tune on a few sequences and report on all. Every result links to
   the exact config YAML used (`kestrel_info` prints it, and runs store it).
5. **Separate VIO and SLAM.** Report VIO-only and with-loop-closure numbers; loop closure can hide
   a weak odometry.
6. **Publish the scripts.** `tools/evaluate.py` (planned, M10) wraps `evo` [Grupp17] so anyone can
   reproduce the numbers in Docker.

---

## 7. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| Dataset runners (EuRoC, TUM-VI) writing TUM-format trajectories | `kestrel_core/apps/` — **planned (M3/M10)** |
| `tools/evaluate.py` (evo wrapper: ATE/RPE, plots, tables) | **planned (M10)** |
| `docker compose run --rm euroc <sequence>` | **planned (M10)** |
| Results table and procedure | `docs/benchmarks.md` — **planned (M10)** |

---

## 8. Pitfalls

* **Aligning with Sim(3)** a system that should know scale.
* **Evaluating only the keyframes** instead of every frame. That hides the jitter of non-keyframe
  poses that a controller actually sees.
* **Comparing ATE with loop closure against a baseline without it** (or vice versa).
* **Body frame mismatch:** ground truth is often given for a marker or the IMU frame. Express the
  estimate in the same frame (apply $\mathbf{T}_{BC}$ etc.), or the lever arm shows up as error.
* **Cherry-picking runs**, even unintentionally ("that one was a bad run").

---

## 9. Exercises

1. Prove that the $\mathbf{R}$ of §2 is the best rotation by writing the cost as
   $\text{const} - 2s\,\mathrm{tr}(\mathbf{R}^\top\boldsymbol\Sigma_{xy})$ and maximizing the trace.
2. Implement 4-DoF (yaw + translation) alignment. Hint: it reduces to a 2-D Umeyama on the $xy$
   components plus a mean offset in $z$.
3. Take a ground-truth trajectory, add a constant 1% scale error, and compute ATE with SE(3) and
   with Sim(3) alignment. Which hides the error?
4. A system has ATE 3 cm on MH_01 in 4 of 5 runs and loses tracking in the 5th. Write the table row
   you would publish.
