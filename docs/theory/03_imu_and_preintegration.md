# 03 — IMU Modelling and Preintegration

> **Why this chapter exists.** The IMU makes Kestrel robust. Gravity makes roll and pitch
> observable, and the IMU bridges motion blur, textureless walls, and moments when a large moving
> object fills the view. It only helps if we model it honestly (noise, biases) and fold hundreds of
> samples per second into an optimizer that runs at keyframe rate. **Preintegration** does that
> folding.

Primary references: **[Forster17]** (on-manifold preintegration), **[Lupton12]** (the original
preintegration idea), **[Qin18]** (VINS-Mono's midpoint discretization), **[Woodman07]**
(inertial navigation basics), **[IEEE952]** and **[Allan66]** (Allan variance).

---

## 1. Intuition

An accelerometer measures *specific force*, meaning acceleration minus gravity, in its own
rotating frame. A gyroscope measures angular velocity, also in its own frame. Integrate the gyro
once and you get orientation. Rotate the accelerometer reading into the world frame, add gravity
back, and integrate twice to get position. Each integration amplifies errors. An uncorrected bias
of only 0.05 m/s² becomes 2.5 m of position error after 10 s ($\tfrac12 b t^2$). Pure inertial
navigation with a MEMS IMU fails within seconds. **Vision bounds the drift; the IMU fills the gaps
and fixes gravity's direction.**

---

## 2. Measurement model

With $\mathbf{R} = \mathbf{R}_{WB}$, true body angular velocity $\boldsymbol\omega$ and world
acceleration $\mathbf{a}_W$:

$$
\tilde{\boldsymbol\omega} = \boldsymbol\omega + \mathbf{b}_g + \mathbf{n}_g,\qquad
\tilde{\mathbf{a}} = \mathbf{R}^\top(\mathbf{a}_W - \mathbf{g}_W) + \mathbf{b}_a + \mathbf{n}_a,
$$

$$
\dot{\mathbf{b}}_g = \mathbf{n}_{bg},\qquad \dot{\mathbf{b}}_a = \mathbf{n}_{ba}.
$$

**Sanity check of the sign.** With $\mathbf{g}_W = (0,0,-g)$ and the IMU at rest and level
($\mathbf{a}_W = 0$, $\mathbf{R} = \mathbf{I}$), $\tilde{\mathbf{a}} = (0, 0, +g)$. A resting
accelerometer reads **+9.81 m/s² upward**. That reading is the reaction force of the table, not
gravity itself. The `kestrel_capture` monitor checks $\|\tilde{\mathbf{a}}\|\approx g$ for this
reason.

### 2.1 Noise: densities and random walks

$\mathbf{n}_g, \mathbf{n}_a$ are modelled as continuous-time white noise with **noise densities**
$\sigma_g$ [rad/s/√Hz] and $\sigma_a$ [m/s²/√Hz]. The biases follow random walks driven by white
noise with densities $\sigma_{bg}$ [rad/s²/√Hz] and $\sigma_{ba}$ [m/s³/√Hz]. These four numbers
are the `imu:` section of the Kestrel config, in the same units as Kalibr's `imu.yaml`.

**Discretization at sample period $\Delta t$** (rate $1/\Delta t$):

$$
\sigma_{g,d} = \frac{\sigma_g}{\sqrt{\Delta t}},\qquad
\sigma_{bg,d} = \sigma_{bg}\sqrt{\Delta t}.
$$

The same holds for the accelerometer. Why do they scale in opposite directions? White noise
averaged over a shorter window is noisier, so the per-sample standard deviation grows as
$\Delta t$ shrinks. A random walk accumulates less in a shorter step. Mixing these up is the
single most common IMU-configuration bug.

### 2.2 Allan variance in a nutshell

To estimate the parameters, record the IMU completely still for a few hours. Then, for
each averaging time $\tau$, average the signal over consecutive windows of length $\tau$ and
compute half the mean squared difference between successive averages. That quantity is the Allan
variance $\sigma^2(\tau)$. On a log–log plot of the **Allan deviation** $\sigma(\tau)$:

* a slope of **−1/2** is white noise. Read the noise density at $\tau = 1$ s: $\sigma(1)\approx\sigma_g$;
* a flat bottom is *bias instability*, the best achievable bias stability;
* a slope of **+1/2** is the rate random walk. Read $\sigma_{bg}$ at $\tau = 3$ s on that line.

Datasheet values are optimistic for a sensor mounted on a vibrating robot. Practitioners
commonly inflate both densities by a factor of 2–10 for the estimator. The Kestrel defaults are
already conservative.

---

## 3. Kinematics and why naive integration is a problem

$$
\dot{\mathbf{R}} = \mathbf{R}\,\boldsymbol\omega^\wedge,\qquad \dot{\mathbf{v}} = \mathbf{a}_W,\qquad \dot{\mathbf{p}} = \mathbf{v}.
$$

Integrating from keyframe $i$ to keyframe $j$ over samples $k = i, \dots, j-1$ (assuming
measurements are constant over each $\Delta t$; §5 refines this):

$$
\begin{aligned}
\mathbf{R}_j &= \mathbf{R}_i\prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k - \mathbf{b}_g)\Delta t\big),\\
\mathbf{v}_j &= \mathbf{v}_i + \mathbf{g}_W\Delta t_{ij} + \sum_{k}\mathbf{R}_k(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t,\\
\mathbf{p}_j &= \mathbf{p}_i + \sum_{k}\Big[\mathbf{v}_k\Delta t + \tfrac12\mathbf{g}_W\Delta t^2 + \tfrac12\mathbf{R}_k(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\Big],
\end{aligned}
$$

where $\mathbf{R}_k$ and $\mathbf{v}_k$ are the intermediate states obtained along the way (noise
terms omitted). The problem: the result depends on $\mathbf{R}_i$ and $\mathbf{v}_i$. Every time the
optimizer changes the estimate of keyframe $i$, which it does every iteration, all ~7–20
IMU samples between the keyframes (at 200 Hz) would need re-integrating.

---

## 4. Preintegration

**Trick:** move everything that depends on the state at $i$ to the left-hand side. Define the
**preintegrated measurements**, which depend *only* on IMU data and the bias estimate:

$$
\begin{aligned}
\Delta\mathbf{R}_{ij} &= \prod_{k=i}^{j-1}\mathrm{Exp}\big((\tilde{\boldsymbol\omega}_k - \mathbf{b}_g)\Delta t\big),\\
\Delta\mathbf{v}_{ij} &= \sum_{k=i}^{j-1}\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t,\\
\Delta\mathbf{p}_{ij} &= \sum_{k=i}^{j-1}\Big[\Delta\mathbf{v}_{ik}\Delta t + \tfrac12\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}_a)\Delta t^2\Big].
\end{aligned}
$$

They relate to the states by

$$
\boxed{
\begin{aligned}
\Delta\mathbf{R}_{ij} &= \mathbf{R}_i^\top\mathbf{R}_j,\\
\Delta\mathbf{v}_{ij} &= \mathbf{R}_i^\top(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}_W\Delta t_{ij}),\\
\Delta\mathbf{p}_{ij} &= \mathbf{R}_i^\top(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\Delta t_{ij} - \tfrac12\mathbf{g}_W\Delta t_{ij}^2).
\end{aligned}}
$$

These are motion increments expressed in the frame of body $i$. They do not change when the
optimizer moves $\mathbf{R}_i$ or $\mathbf{p}_i$. Integrate once, and use the result for every
iteration.

---

## 5. Uncertainty propagation

Substitute $\mathbf{R} = \hat{\mathbf{R}}\,\mathrm{Exp}(\delta\boldsymbol\phi)$,
$\mathbf{v} = \hat{\mathbf{v}} + \delta\mathbf{v}$, $\mathbf{p} = \hat{\mathbf{p}} + \delta\mathbf{p}$
for the preintegrated quantities and keep first-order terms
(using [Ch. 01](01_lie_groups.md) §2.3). This gives a linear recursion for
$\boldsymbol\eta_{ik} = (\delta\boldsymbol\phi_{ik}, \delta\mathbf{v}_{ik}, \delta\mathbf{p}_{ik})$:

$$
\begin{aligned}
\delta\boldsymbol\phi_{i,k+1} &= \Delta\mathbf{R}_{k,k+1}^\top\,\delta\boldsymbol\phi_{ik} + \mathbf{J}_r^{k}\,\mathbf{n}_{g,d}\,\Delta t,\\
\delta\mathbf{v}_{i,k+1} &= \delta\mathbf{v}_{ik} - \Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}_a)^\wedge\delta\boldsymbol\phi_{ik}\Delta t + \Delta\mathbf{R}_{ik}\mathbf{n}_{a,d}\Delta t,\\
\delta\mathbf{p}_{i,k+1} &= \delta\mathbf{p}_{ik} + \delta\mathbf{v}_{ik}\Delta t - \tfrac12\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \mathbf{b}_a)^\wedge\delta\boldsymbol\phi_{ik}\Delta t^2 + \tfrac12\Delta\mathbf{R}_{ik}\mathbf{n}_{a,d}\Delta t^2,
\end{aligned}
$$

where $\mathbf{J}_r^{k} = \mathbf{J}_r\big((\tilde{\boldsymbol\omega}_k - \mathbf{b}_g)\Delta t\big)$.
In matrix form, $\boldsymbol\eta_{i,k+1} = \mathbf{A}_k\boldsymbol\eta_{ik} + \mathbf{B}_k\mathbf{n}_k$,
so the covariance propagates as

$$
\boldsymbol\Sigma_{i,k+1} = \mathbf{A}_k\boldsymbol\Sigma_{ik}\mathbf{A}_k^\top + \mathbf{B}_k\,\boldsymbol\Sigma_{n}\,\mathbf{B}_k^\top,
\qquad \boldsymbol\Sigma_{ii} = \mathbf{0}.
$$

Kestrel (like VINS-Mono) propagates a **15-dimensional** error state that also includes the two
bias errors, so the bias random walk enters the same covariance. It also uses **midpoint
integration**: the average of samples $k$ and $k+1$, rotated by the average of the start and end
orientations, instead of the zero-order hold above. That reduces discretization error at the
same IMU rate. The structure of the recursion is unchanged; the per-step matrices simply contain
both endpoints. [Qin18] and its open-source code give the explicit midpoint $\mathbf{A}_k$ and
$\mathbf{B}_k$.

---

## 6. Biases: first-order correction

The preintegrated values were computed with a bias estimate $\bar{\mathbf{b}}$. When the optimizer
changes the bias by $\delta\mathbf{b}$, re-integrating would defeat the purpose. Instead,
Forster et al. expand to first order:

$$
\begin{aligned}
\Delta\mathbf{R}_{ij}(\mathbf{b}) &\approx \Delta\mathbf{R}_{ij}(\bar{\mathbf{b}})\,\mathrm{Exp}\!\Big(\tfrac{\partial\Delta\mathbf{R}}{\partial\mathbf{b}_g}\delta\mathbf{b}_g\Big),\\
\Delta\mathbf{v}_{ij}(\mathbf{b}) &\approx \Delta\mathbf{v}_{ij}(\bar{\mathbf{b}}) + \tfrac{\partial\Delta\mathbf{v}}{\partial\mathbf{b}_g}\delta\mathbf{b}_g + \tfrac{\partial\Delta\mathbf{v}}{\partial\mathbf{b}_a}\delta\mathbf{b}_a,\\
\Delta\mathbf{p}_{ij}(\mathbf{b}) &\approx \Delta\mathbf{p}_{ij}(\bar{\mathbf{b}}) + \tfrac{\partial\Delta\mathbf{p}}{\partial\mathbf{b}_g}\delta\mathbf{b}_g + \tfrac{\partial\Delta\mathbf{p}}{\partial\mathbf{b}_a}\delta\mathbf{b}_a.
\end{aligned}
$$

The five Jacobians are accumulated alongside the integration (zero-order-hold form):

$$
\begin{aligned}
\tfrac{\partial\Delta\mathbf{R}_{i,k+1}}{\partial\mathbf{b}_g} &= \Delta\mathbf{R}_{k,k+1}^\top\tfrac{\partial\Delta\mathbf{R}_{ik}}{\partial\mathbf{b}_g} - \mathbf{J}_r^k\Delta t,\\
\tfrac{\partial\Delta\mathbf{v}_{i,k+1}}{\partial\mathbf{b}_a} &= \tfrac{\partial\Delta\mathbf{v}_{ik}}{\partial\mathbf{b}_a} - \Delta\mathbf{R}_{ik}\Delta t,\\
\tfrac{\partial\Delta\mathbf{v}_{i,k+1}}{\partial\mathbf{b}_g} &= \tfrac{\partial\Delta\mathbf{v}_{ik}}{\partial\mathbf{b}_g} - \Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \bar{\mathbf{b}}_a)^\wedge\tfrac{\partial\Delta\mathbf{R}_{ik}}{\partial\mathbf{b}_g}\Delta t,\\
\tfrac{\partial\Delta\mathbf{p}_{i,k+1}}{\partial\mathbf{b}_a} &= \tfrac{\partial\Delta\mathbf{p}_{ik}}{\partial\mathbf{b}_a} + \tfrac{\partial\Delta\mathbf{v}_{ik}}{\partial\mathbf{b}_a}\Delta t - \tfrac12\Delta\mathbf{R}_{ik}\Delta t^2,\\
\tfrac{\partial\Delta\mathbf{p}_{i,k+1}}{\partial\mathbf{b}_g} &= \tfrac{\partial\Delta\mathbf{p}_{ik}}{\partial\mathbf{b}_g} + \tfrac{\partial\Delta\mathbf{v}_{ik}}{\partial\mathbf{b}_g}\Delta t - \tfrac12\Delta\mathbf{R}_{ik}(\tilde{\mathbf{a}}_k - \bar{\mathbf{b}}_a)^\wedge\tfrac{\partial\Delta\mathbf{R}_{ik}}{\partial\mathbf{b}_g}\Delta t^2.
\end{aligned}
$$

If the bias correction becomes large (VINS uses roughly 0.1 rad/s or 0.1 m/s² as the cutoff), the
first-order model is no longer trustworthy. The implementation then **re-propagates** from the
stored raw samples with the new linearization point.

---

## 7. The IMU residual

Putting §4 and §6 together, the 15-dimensional residual between keyframes $i$ and $j$ is

$$
\mathbf{r}_{\mathcal{I}_{ij}} =
\begin{pmatrix}
\mathbf{r}_{\Delta p}\\ \mathbf{r}_{\Delta\phi}\\ \mathbf{r}_{\Delta v}\\ \mathbf{r}_{b_a}\\ \mathbf{r}_{b_g}
\end{pmatrix} =
\begin{pmatrix}
\mathbf{R}_i^\top\big(\mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i\Delta t_{ij} - \tfrac12\mathbf{g}_W\Delta t_{ij}^2\big) - \Delta\hat{\mathbf{p}}_{ij}(\mathbf{b}_i)\\[2pt]
\mathrm{Log}\big(\Delta\hat{\mathbf{R}}_{ij}(\mathbf{b}_i)^\top\,\mathbf{R}_i^\top\mathbf{R}_j\big)\\[2pt]
\mathbf{R}_i^\top\big(\mathbf{v}_j - \mathbf{v}_i - \mathbf{g}_W\Delta t_{ij}\big) - \Delta\hat{\mathbf{v}}_{ij}(\mathbf{b}_i)\\[2pt]
\mathbf{b}_{a,j} - \mathbf{b}_{a,i}\\
\mathbf{b}_{g,j} - \mathbf{b}_{g,i}
\end{pmatrix},
$$

where the hatted increments are the bias-corrected values of §6, and the cost is
$\|\mathbf{r}_{\mathcal{I}_{ij}}\|^2_{\boldsymbol\Sigma_{ij}}$. The ordering
$(\delta\mathbf{p}, \delta\boldsymbol\phi, \delta\mathbf{v}, \delta\mathbf{b}_a, \delta\mathbf{b}_g)$
follows VINS-Mono. Kestrel keeps it so that the 15×15 covariance blocks line up with the
residual blocks.

**Note on conventions.** VINS-Mono's code uses a vector $\mathbf{G} = (0, 0, +g)$ and writes
$+\tfrac12\mathbf{G}\Delta t^2$. That is the same equation with $\mathbf{G} = -\mathbf{g}_W$.
In practice the rotation residual is often computed with the small-angle form
$2\,\mathrm{vec}(\cdot)$ of the quaternion product instead of the exact Log
([Ch. 01](01_lie_groups.md) §4). The difference is third-order.

---

## 8. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| Noise parameters | `imu:` section of the config (`config.hpp`, available now) |
| `ImuPreintegration`: midpoint integration, 15×15 covariance and Jacobian, bias correction, re-propagation | `kestrel/imu/preintegration.hpp` — **planned (M1)** |
| IMU factor (Ceres, autodiff) | `kestrel/estimator/factors.hpp` — **planned (M3)** |
| Tests: preintegration vs. ground truth on a simulated trajectory; bias-Jacobian vs. numeric re-integration | `test/test_preintegration.cpp` — **planned (M1)** |

---

## 9. Pitfalls

* **Swapping density and discrete σ**, or random-walk and noise density (§2.1). The estimator
  becomes overconfident or blind to the IMU.
* **Gravity sign.** If the residual uses $+\mathbf{g}_W$ where it should use $-\mathbf{g}_W$, the
  velocity estimate runs away at 2g as soon as the initializer finishes.
* **Units.** Some drivers publish acceleration in units of g or angular rate in deg/s. Check
  $\|\tilde{\mathbf{a}}\|\approx 9.81$ at rest.
* **Timestamp jitter / gaps.** A dropped burst of IMU samples silently creates a long $\Delta t$.
  Treat gaps as errors and inflate the covariance, instead of interpolating across them silently.
* **Saturation.** A 4 g / 1000 °/s range is plenty for a ground robot. A collision or a dropped
  camera can exceed it, and the clipped samples are then biased.

---

## 10. Exercises

1. Show from the equations in §4 that $\Delta\mathbf{v}_{ij}$ does not depend on $\mathbf{R}_i$,
   $\mathbf{v}_i$ or $\mathbf{g}_W$. Then explain where gravity re-enters.
2. With $\sigma_a = 0.08$ m/s²/√Hz at 200 Hz, what is the per-sample accelerometer standard
   deviation? What position standard deviation does white noise alone produce after 1 s of pure
   integration? (Hint: velocity is a random walk, and position is its integral.)
3. Derive $\partial\Delta\mathbf{R}_{ij}/\partial\mathbf{b}_g$ from the definition in §4 using the
   right-Jacobian identity, and check that it matches the recursion in §6.
4. Implement zero-order-hold and midpoint preintegration in 30 lines of Python. Use a simulated
   trajectory $\mathbf{p}(t) = (\sin t, \cos 2t, 0.1t)$ and compare the end-point errors at 100 Hz
   and 400 Hz.
