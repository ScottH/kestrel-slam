# 01 — Lie Groups for Rotations and Poses

> **Why this chapter exists.** A SLAM estimator spends its whole life computing derivatives with respect to
> rotations. Rotations are not vectors. You cannot add two of them, and if you "optimize the nine
> entries of a rotation matrix" you get something that is no longer a rotation. Lie theory is the
> small amount of mathematics that makes calculus on rotations and poses behave as it does on vectors.
> Every later chapter uses it.

Primary references: **[Sola18]** (the most readable introduction), **[Barfoot17]** (chapters 6–7),
**[Forster17]** (the identities used for preintegration) and **[Sola17q]** (quaternion conventions).

---

## 1. Intuition

A rotation in 3-D has three degrees of freedom, but every 3-number parameterization of *all*
rotations has a problem. Euler angles have gimbal lock, and axis–angle wraps around at $\pi$. The
fix is to stop insisting on a global parameterization:

* **Store** the rotation in a redundant but singularity-free form: a $3\times3$ orthonormal matrix
  $\mathbf{R}$, or a unit quaternion $\mathbf{q}$.
* **Perturb** it with a small 3-vector $\delta\boldsymbol\theta$ in a *local* chart around the current
  value: $\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\theta)$.

Near $\delta\boldsymbol\theta = 0$ that chart has no singularities. Each optimization step only ever
needs a small step, so it can always work in a fresh local chart centred on the current estimate.
This is the whole idea behind "optimization on manifolds".

Picture the surface of a sphere. The sphere is the set of rotations. At every point there is a flat
**tangent plane**, where we do our vector maths. The **exponential map** $\mathrm{Exp}$ wraps the
tangent plane back onto the sphere, and the **logarithm** $\mathrm{Log}$ unwraps it.

---

## 2. The rotation group SO(3)

$$
SO(3) = \{\mathbf{R}\in\mathbb{R}^{3\times3} \;|\; \mathbf{R}^\top\mathbf{R} = \mathbf{I},\ \det\mathbf{R} = +1\}.
$$

It is a **group**: the product of two rotations is a rotation, $\mathbf{I}$ is the identity, and
$\mathbf{R}^{-1} = \mathbf{R}^\top$. It is also a smooth 3-dimensional manifold. That combination
makes it a **Lie group**.

### 2.1 The Lie algebra and the hat operator

Differentiating $\mathbf{R}(t)^\top\mathbf{R}(t) = \mathbf{I}$ gives
$\dot{\mathbf{R}}^\top\mathbf{R} + \mathbf{R}^\top\dot{\mathbf{R}} = 0$, so
$\mathbf{R}^\top\dot{\mathbf{R}}$ is **skew-symmetric**. Skew-symmetric $3\times3$ matrices have three
free entries, and we identify them with vectors through the *hat* operator:

$$
\boldsymbol\omega^\wedge = [\boldsymbol\omega]_\times =
\begin{pmatrix} 0 & -\omega_z & \omega_y\\ \omega_z & 0 & -\omega_x\\ -\omega_y & \omega_x & 0\end{pmatrix},
\qquad \boldsymbol\omega^\wedge\mathbf{v} = \boldsymbol\omega\times\mathbf{v}.
$$

The *vee* operator $(\cdot)^\vee$ undoes it. The space $\mathfrak{so}(3)$ of these matrices is the
**Lie algebra**, which is the tangent space at the identity. The kinematic equation of a rotating
body follows directly:

$$
\dot{\mathbf{R}}_{WB} = \mathbf{R}_{WB}\,\boldsymbol\omega_B^\wedge ,
$$

where $\boldsymbol\omega_B$ is the angular velocity measured in the body frame, which is what a
gyroscope measures.

### 2.2 Exponential and logarithm maps

With constant $\boldsymbol\omega$ over a time $t$, the solution is the matrix exponential
$\mathbf{R}(t) = \mathbf{R}(0)\exp(t\boldsymbol\omega^\wedge)$. Define
$\mathrm{Exp}(\boldsymbol\phi) = \exp(\boldsymbol\phi^\wedge)$ for a rotation vector $\boldsymbol\phi$
with angle $\theta = \|\boldsymbol\phi\|$ and axis $\mathbf{a} = \boldsymbol\phi/\theta$. The series
collapses because $(\mathbf{a}^\wedge)^3 = -\mathbf{a}^\wedge$, which gives **Rodrigues' formula**:

$$
\mathrm{Exp}(\boldsymbol\phi) = \mathbf{I} + \frac{\sin\theta}{\theta}\boldsymbol\phi^\wedge + \frac{1-\cos\theta}{\theta^2}\left(\boldsymbol\phi^\wedge\right)^2 .
$$

The inverse is

$$
\theta = \arccos\!\left(\frac{\operatorname{tr}\mathbf{R} - 1}{2}\right),\qquad
\mathrm{Log}(\mathbf{R}) = \frac{\theta}{2\sin\theta}\left(\mathbf{R} - \mathbf{R}^\top\right)^\vee .
$$

**Numerical care.** For $\theta\to0$, use the Taylor expansions
$\sin\theta/\theta \approx 1 - \theta^2/6$ and $(1-\cos\theta)/\theta^2 \approx 1/2 - \theta^2/24$.
For $\theta\to\pi$ the formula above is ill-conditioned. Recover the axis from the symmetric part
$\mathbf{R} + \mathbf{I}$ instead, or do the computation with quaternions, which is better
conditioned.

### 2.3 Three identities you will use constantly

1. **Adjoint.** Moving a perturbation from one side of a rotation to the other:
   $$\mathbf{R}\,\mathrm{Exp}(\boldsymbol\phi)\,\mathbf{R}^\top = \mathrm{Exp}(\mathbf{R}\boldsymbol\phi)
   \quad\Longleftrightarrow\quad
   \mathrm{Exp}(\boldsymbol\phi)\,\mathbf{R} = \mathbf{R}\,\mathrm{Exp}(\mathbf{R}^\top\boldsymbol\phi).$$
2. **Hat and rotation.** $(\mathbf{R}\mathbf{a})^\wedge = \mathbf{R}\mathbf{a}^\wedge\mathbf{R}^\top$.
3. **First-order BCH (right Jacobian).** For small $\delta\boldsymbol\phi$,
   $$\mathrm{Exp}(\boldsymbol\phi + \delta\boldsymbol\phi) \approx \mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}\!\left(\mathbf{J}_r(\boldsymbol\phi)\,\delta\boldsymbol\phi\right),$$
   $$\mathrm{Log}\!\left(\mathrm{Exp}(\boldsymbol\phi)\,\mathrm{Exp}(\delta\boldsymbol\phi)\right) \approx \boldsymbol\phi + \mathbf{J}_r^{-1}(\boldsymbol\phi)\,\delta\boldsymbol\phi,$$
   with
   $$\mathbf{J}_r(\boldsymbol\phi) = \mathbf{I} - \frac{1-\cos\theta}{\theta^2}\boldsymbol\phi^\wedge + \frac{\theta - \sin\theta}{\theta^3}\left(\boldsymbol\phi^\wedge\right)^2,$$
   $$\mathbf{J}_r^{-1}(\boldsymbol\phi) = \mathbf{I} + \frac12\boldsymbol\phi^\wedge + \left(\frac{1}{\theta^2} - \frac{1+\cos\theta}{2\theta\sin\theta}\right)\left(\boldsymbol\phi^\wedge\right)^2.$$

$\mathbf{J}_r$ tells you how a change of the rotation *vector* turns into a change of the rotation
seen in the *body* frame. It shows up in IMU preintegration ([Ch. 03](03_imu_and_preintegration.md))
whenever biases change. The left Jacobian satisfies $\mathbf{J}_l(\boldsymbol\phi) = \mathbf{J}_r(-\boldsymbol\phi)$.

---

## 3. Perturbations and derivatives

### 3.1 Right (local) versus left (global) perturbations

There are two ways to put a small error on a rotation:

$$
\text{right / local: } \mathbf{R} = \hat{\mathbf{R}}\,\mathrm{Exp}(\delta\boldsymbol\theta),\qquad
\text{left / global: } \mathbf{R} = \mathrm{Exp}(\delta\boldsymbol\theta)\,\hat{\mathbf{R}}.
$$

For $\mathbf{R}_{WB}$, the right perturbation is an error expressed in the **body** frame and the
left one is in the **world** frame. Both are correct, but a codebase must pick one and use it
everywhere. **Kestrel uses right perturbations**, like [Forster17] and VINS-Mono [Qin18]. Ceres's
built-in `QuaternionManifold` uses a *left* perturbation, which is one reason Kestrel defines its
own pose manifold (§6).

The corresponding "plus" and "minus" operators are

$$
\mathbf{R}\boxplus\delta\boldsymbol\theta = \mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\theta),\qquad
\mathbf{R}_2 \boxminus \mathbf{R}_1 = \mathrm{Log}(\mathbf{R}_1^\top\mathbf{R}_2).
$$

### 3.2 Derivatives as limits in the tangent space

The derivative of a function $f: SO(3)\to\mathbb{R}^m$ is defined through the perturbation:

$$
\frac{\partial f}{\partial\delta\boldsymbol\theta} = \lim_{\delta\boldsymbol\theta\to0}
\frac{f(\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\theta)) - f(\mathbf{R})}{\delta\boldsymbol\theta}.
$$

**Example: rotating a point.** With $\mathrm{Exp}(\delta\boldsymbol\theta)\approx\mathbf{I}+\delta\boldsymbol\theta^\wedge$,

$$
\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\theta)\,\mathbf{p} \approx \mathbf{R}\mathbf{p} + \mathbf{R}\,\delta\boldsymbol\theta^\wedge\mathbf{p}
= \mathbf{R}\mathbf{p} - \mathbf{R}\,\mathbf{p}^\wedge\delta\boldsymbol\theta
\quad\Rightarrow\quad
\frac{\partial(\mathbf{R}\mathbf{p})}{\partial\delta\boldsymbol\theta} = -\mathbf{R}\,\mathbf{p}^\wedge .
$$

**Example: inverse rotation.** $\left(\mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\theta)\right)^\top\mathbf{p} = \mathrm{Exp}(-\delta\boldsymbol\theta)\mathbf{R}^\top\mathbf{p}\approx\mathbf{R}^\top\mathbf{p} + (\mathbf{R}^\top\mathbf{p})^\wedge\delta\boldsymbol\theta$, so
$\partial(\mathbf{R}^\top\mathbf{p})/\partial\delta\boldsymbol\theta = (\mathbf{R}^\top\mathbf{p})^\wedge$.

Together with the chain rule, these two examples cover almost every Jacobian in a visual–inertial
system. For example, the reprojection Jacobian with respect to the body rotation in
[Ch. 06](06_sliding_window_vio.md) is assembled from them.

---

## 4. Unit quaternions

A unit quaternion $\mathbf{q} = (q_w, \mathbf{q}_v)$ with $\|\mathbf{q}\| = 1$ represents the rotation
by angle $\theta$ about axis $\mathbf{a}$ as $\mathbf{q} = (\cos\tfrac\theta2, \mathbf{a}\sin\tfrac\theta2)$.
Kestrel uses the **Hamilton** convention:

$$
\mathbf{p}\otimes\mathbf{q} =
\begin{pmatrix} p_wq_w - \mathbf{p}_v^\top\mathbf{q}_v \\ p_w\mathbf{q}_v + q_w\mathbf{p}_v + \mathbf{p}_v\times\mathbf{q}_v\end{pmatrix},
\qquad
\mathbf{R}(\mathbf{p}\otimes\mathbf{q}) = \mathbf{R}(\mathbf{p})\mathbf{R}(\mathbf{q}).
$$

Quaternion composition matches matrix composition in the same order. JPL-convention papers (e.g.
[Trawny05] and some MSCKF literature) reverse this, so check before copying a formula. [Sola17q]
explains both conventions side by side.

The exponential map is
$\mathrm{Exp}_q(\boldsymbol\phi) = (\cos\tfrac{\theta}{2}, \tfrac{\boldsymbol\phi}{\theta}\sin\tfrac\theta2)$,
and for small angles

$$
\mathrm{Exp}_q(\delta\boldsymbol\theta) \approx \left(1, \tfrac12\delta\boldsymbol\theta\right),\qquad
\mathrm{Log}_q(\mathbf{q}) \approx 2\,\mathbf{q}_v\ \ (\text{for } q_w>0).
$$

That small-angle form is what VINS-style code writes as `deltaQ(theta)`, and what residuals mean by
`2 * q.vec()`.

**Double cover.** $\mathbf{q}$ and $-\mathbf{q}$ are the same rotation. Before taking a logarithm,
flip the sign so that $q_w \ge 0$. Otherwise "a rotation of 1°" can come out as "a rotation of
359°", and the optimizer receives a huge residual.

---

## 5. Poses: SE(3) versus $\mathbb{R}^3\times SO(3)$

A rigid transform is

$$
\mathbf{T} = \begin{pmatrix}\mathbf{R} & \mathbf{t}\\ \mathbf{0}^\top & 1\end{pmatrix}\in SE(3),\qquad
\mathbf{T}^{-1} = \begin{pmatrix}\mathbf{R}^\top & -\mathbf{R}^\top\mathbf{t}\\ \mathbf{0}^\top & 1\end{pmatrix}.
$$

SE(3) has its own exponential map. With $\boldsymbol\xi = (\boldsymbol\rho, \boldsymbol\phi)$,

$$
\mathrm{Exp}(\boldsymbol\xi) = \begin{pmatrix}\mathrm{Exp}(\boldsymbol\phi) & \mathbf{J}_l(\boldsymbol\phi)\boldsymbol\rho\\ \mathbf{0}^\top & 1\end{pmatrix}.
$$

Here the translation is "screwed" along with the rotation.

For estimation we have a choice of how to perturb a pose:

* **SE(3) perturbation:** $\mathbf{T}\,\mathrm{Exp}(\boldsymbol\xi)$. This is mathematically elegant
  and natural for pose graphs of pure poses.
* **Decoupled $\mathbb{R}^3\times SO(3)$ perturbation:** $(\mathbf{p}+\delta\mathbf{p},\ \mathbf{R}\,\mathrm{Exp}(\delta\boldsymbol\theta))$.

**Kestrel uses the decoupled form for body poses**, as VINS-Mono, OKVIS and ORB-SLAM3's inertial
part do. The reason is that a visual–inertial state also contains a velocity and biases that live
in $W$ or in $B$, and the IMU residual ([Ch. 03](03_imu_and_preintegration.md)) is written in terms
of $\mathbf{p}_W$ and $\mathbf{R}_{WB}$ separately. Perturbing the position in the world frame keeps
the IMU Jacobians simple. To first order both choices produce the same optimum; only the
parameterization of the step differs.

---

## 6. Optimization on manifolds (⊞ / ⊟)

Nonlinear least squares ([Ch. 05](05_nonlinear_least_squares.md)) solves for a step $\boldsymbol\delta$
in a vector space. On a manifold we:

1. compute the step in the **tangent space** (dimension 6 for a pose),
2. apply it with $\boxplus$: $\mathbf{x}\leftarrow\mathbf{x}\boxplus\boldsymbol\delta$,
3. never touch the redundant ambient coordinates directly.

Ceres calls this a `Manifold`, older versions called it a `LocalParameterization`, and GTSAM calls
it `retract`/`localCoordinates`. A manifold implementation provides:

| Function | Meaning for Kestrel's pose $\mathbf{x} = (\mathbf{p}, \mathbf{q})$, stored as 7 numbers $(p_x,p_y,p_z,q_x,q_y,q_z,q_w)$ |
|----------|---------|
| `Plus(x, δ)` | $(\mathbf{p}+\delta\mathbf{p},\ \mathbf{q}\otimes\mathrm{Exp}_q(\delta\boldsymbol\theta))$ |
| `Minus(y, x)` | $(\mathbf{p}_y-\mathbf{p}_x,\ \mathrm{Log}_q(\mathbf{q}_x^{-1}\otimes\mathbf{q}_y))$ |
| `PlusJacobian(x)` | $\partial\,\mathrm{Plus}(\mathbf{x},\boldsymbol\delta)/\partial\boldsymbol\delta$ at $\boldsymbol\delta=0$, a $7\times6$ matrix |
| `MinusJacobian(x)` | $\partial\,\mathrm{Minus}(\mathbf{y},\mathbf{x})/\partial\mathbf{y}$ at $\mathbf{y}=\mathbf{x}$, a $6\times7$ matrix |

**Deriving `PlusJacobian` for the quaternion block.** With $\mathbf{q}\otimes(1, \tfrac12\delta\boldsymbol\theta)$
and the Hamilton product from §4:

$$
\frac{\partial\,\mathbf{q}_v'}{\partial\delta\boldsymbol\theta} = \tfrac12\left(q_w\mathbf{I} + \mathbf{q}_v^\wedge\right),\qquad
\frac{\partial\,q_w'}{\partial\delta\boldsymbol\theta} = -\tfrac12\mathbf{q}_v^\top .
$$

Stack those into the $4\times3$ block, and put an identity block for the position. You can check
that the $4\times3$ block $\mathbf{P}$ satisfies $\mathbf{P}^\top\mathbf{P} = \tfrac14\mathbf{I}$ for a
unit quaternion, so $\mathbf{M} = 4\mathbf{P}^\top$ is a valid `MinusJacobian`, with
$\mathbf{M}\mathbf{P} = \mathbf{I}$.

**Why the solver needs these.** Cost functions (written with automatic differentiation) give
Jacobians with respect to the 7 *ambient* numbers. The solver multiplies them by `PlusJacobian` to
get Jacobians with respect to the 6 tangent directions it actually optimizes. Marginalization priors
([Ch. 06](06_sliding_window_vio.md)) need the reverse map, from a tangent-space prior back to ambient
coordinates, which is where `Minus` and `MinusJacobian` come in.

---

## 7. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| hat/vee, Exp/Log, $\mathbf{J}_r$, $\mathbf{J}_r^{-1}$, templated quaternion Log for autodiff | `kestrel/common/lie.hpp` — **planned (M1)** |
| `SE3` helper type (compose, inverse, transform points) | `kestrel/common/types.hpp` — **planned (M1)** |
| Pose manifold with right perturbation (`Plus/Minus` + Jacobians) | `kestrel/estimator/manifold.hpp` — **planned (M3)** |
| Unit tests: Exp/Log round trip, $\mathbf{J}_r$ against numeric differentiation, manifold `Minus(Plus(x,δ),x)=δ` | `kestrel_core/test/test_lie.cpp` — **planned (M1)** |

---

## 8. Pitfalls

* **Mixing perturbation sides.** If a Jacobian derived for $\mathrm{Exp}(\delta)\mathbf{R}$ is fed to a
  solver that applies $\mathbf{R}\,\mathrm{Exp}(\delta)$, the estimator still "works" on easy data
  but converges slowly or wrongly under fast rotation. Test every analytic Jacobian numerically.
* **Quaternion sign.** Normalize $q_w\ge0$ before Log, interpolation or residuals.
* **Hamilton versus JPL.** Copying an equation from a JPL paper into Hamilton code silently
  transposes rotations.
* **Eigen coefficient order.** `Eigen::Quaterniond(w, x, y, z)` takes $w$ first in its
  constructor, but `coeffs()` returns $(x, y, z, w)$. Raw `double*` parameter blocks follow
  `coeffs()` order.
* **Drift off the manifold.** Repeatedly multiplying rotation matrices accumulates round-off.
  Re-normalize quaternions or re-orthonormalize matrices periodically.
* **Small-angle branches with autodiff.** A branch on $\theta < \epsilon$ is fine, but make sure
  the small-angle branch still propagates derivatives. Dividing a Jet by $\theta = 0$ yields NaN
  derivatives even when the value is finite.

---

## 9. Exercises

1. Prove $\mathbf{R}\,\mathrm{Exp}(\boldsymbol\phi)\,\mathbf{R}^\top = \mathrm{Exp}(\mathbf{R}\boldsymbol\phi)$
   using the series definition and identity 2 of §2.3.
2. Derive $\partial\,\mathrm{Log}(\mathbf{R}_1^\top\mathbf{R}_2)/\partial\delta\boldsymbol\theta_2$ under a
   right perturbation of $\mathbf{R}_2$. Answer: $\mathbf{J}_r^{-1}(\mathrm{Log}(\mathbf{R}_1^\top\mathbf{R}_2))$.
   Then derive the derivative with respect to $\delta\boldsymbol\theta_1$.
3. Numerically check the formula for $\mathbf{J}_r$. Pick a random $\boldsymbol\phi$ with
   $\|\boldsymbol\phi\| = 2$ rad, perturb each component by $10^{-6}$, and compare
   $\mathrm{Log}(\mathrm{Exp}(\boldsymbol\phi)^\top\mathrm{Exp}(\boldsymbol\phi + \epsilon\mathbf{e}_i))/\epsilon$
   with the $i$-th column of $\mathbf{J}_r(\boldsymbol\phi)$.
4. Show that for a unit quaternion the `PlusJacobian` block $\mathbf{P}$ from §6 satisfies
   $\mathbf{P}^\top\mathbf{P} = \tfrac14\mathbf{I}$. Explain why this makes $4\mathbf{P}^\top$ a left
   inverse.
