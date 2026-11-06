# 05 — Nonlinear Least Squares

> **Why this chapter exists.** Kestrel's estimator, loop-closure pose graph and initializer all
> solve the same kind of problem: find the states that best explain noisy measurements. This
> chapter derives that problem from probability, shows how it is solved (Gauss–Newton,
> Levenberg–Marquardt), and covers the structure (sparsity, Schur complement) that makes it fast
> enough to run in real time.

Primary references: **[Nocedal06]** (optimization), **[Triggs00]** (bundle adjustment),
**[Dellaert17]** (factor graphs), **[Barfoot17]** ch. 4, **[Huber64]** and **[Zhang97]**
(robust estimation), **[Ceres]** (the solver Kestrel uses).

---

## 1. From probability to least squares

Let $\mathbf{x}$ be all unknown states and $\{\mathbf{z}_i\}$ the measurements. Each measurement
has a model $\mathbf{z}_i = h_i(\mathbf{x}) + \boldsymbol\epsilon_i$ with
$\boldsymbol\epsilon_i\sim\mathcal{N}(\mathbf{0}, \boldsymbol\Sigma_i)$, independent across $i$. The
**maximum a posteriori** estimate is

$$
\mathbf{x}^\star = \arg\max_{\mathbf{x}}\ p(\mathbf{x})\prod_i p(\mathbf{z}_i\mid\mathbf{x})
= \arg\min_{\mathbf{x}}\ \underbrace{\|\mathbf{r}_0(\mathbf{x})\|^2_{\boldsymbol\Sigma_0}}_{\text{prior}} + \sum_i\|\underbrace{h_i(\mathbf{x}) - \mathbf{z}_i}_{\mathbf{r}_i(\mathbf{x})}\|^2_{\boldsymbol\Sigma_i},
$$

because $-\log$ of a Gaussian is a squared Mahalanobis distance plus a constant. **Least squares is
MAP estimation under Gaussian noise.** Every term in Kestrel's cost (reprojection, IMU, prior,
pose-graph edges) is a residual $\mathbf{r}_i$ with a covariance.

### 1.1 Whitening

Write $\boldsymbol\Sigma_i^{-1} = \mathbf{L}_i\mathbf{L}_i^\top$ (Cholesky). Then
$\|\mathbf{r}\|^2_{\boldsymbol\Sigma} = \|\mathbf{L}^\top\mathbf{r}\|^2$. The matrix
$\mathbf{L}^\top$ is the **square-root information** matrix. Multiplying each residual by it
produces unit-variance, uncorrelated residuals, and the solver can treat all of them alike. For a
reprojection residual in normalized coordinates with pixel noise $\sigma_{px}$, the square-root
information is just the scalar $f/\sigma_{px}$.

---

## 2. Gauss–Newton

Stack all whitened residuals into $\mathbf{r}(\mathbf{x})$. Linearize around the current estimate
$\bar{\mathbf{x}}$, with $\mathbf{J} = \partial\mathbf{r}/\partial\boldsymbol\delta$ (on a manifold,
$\mathbf{x} = \bar{\mathbf{x}}\boxplus\boldsymbol\delta$, see [Ch. 01](01_lie_groups.md) §6):

$$
\|\mathbf{r}(\bar{\mathbf{x}}\boxplus\boldsymbol\delta)\|^2 \approx \|\mathbf{r} + \mathbf{J}\boldsymbol\delta\|^2 .
$$

Minimizing the right-hand side gives the **normal equations**

$$
\underbrace{\mathbf{J}^\top\mathbf{J}}_{\mathbf{H}}\ \boldsymbol\delta = -\underbrace{\mathbf{J}^\top\mathbf{r}}_{\mathbf{g}},
\qquad \bar{\mathbf{x}}\leftarrow\bar{\mathbf{x}}\boxplus\boldsymbol\delta .
$$

Repeat until the step is small. $\mathbf{H}$ approximates the Hessian (it drops the second
derivatives of $\mathbf{r}$), and it is also the **information matrix** of the estimate. Its
inverse approximates the posterior covariance. Two views of the same matrix:

* **optimization:** the curvature of the cost;
* **probability:** how much the data tells us about each direction of $\mathbf{x}$.

A direction in which $\mathbf{H}$ has a zero eigenvalue is a direction the data says nothing
about. In VIO that is global position and yaw ([Ch. 06](06_sliding_window_vio.md) §6).

## 3. Levenberg–Marquardt

Far from the optimum, the linearization can be poor and a full Gauss–Newton step can increase
the cost. **Levenberg–Marquardt** damps the step:

$$
(\mathbf{H} + \mu\,\mathbf{D})\,\boldsymbol\delta = -\mathbf{g},
$$

with $\mathbf{D}$ = identity or $\mathrm{diag}(\mathbf{H})$. Large $\mu$ gives a short
gradient-descent step, and small $\mu$ gives Gauss–Newton. After each step, compare the actual
cost decrease with the decrease the linear model predicted. If the step was good, accept it and
shrink $\mu$. If it was bad, reject it and grow $\mu$. This is a **trust-region** method
[Nocedal06]. Ceres uses LM by default, and Kestrel caps both iterations and wall-clock time
(`max_solver_iterations`, `max_solver_time_s`). In a sliding window the estimate starts close to
the optimum, so a few iterations are enough.

---

## 4. Robust kernels

Gaussian noise is a lie in the presence of outliers: wrong matches, moving objects. A residual
10σ off contributes 100 units of cost and drags the solution towards itself. **M-estimators**
replace the square by a function $\rho$ that grows more slowly. With
$s_i = \|\mathbf{r}_i\|^2$:

$$
\min_{\mathbf{x}}\sum_i\rho(s_i).
$$

| Kernel | $\rho(s)$ | Weight $w = \rho'(s)$ | Behaviour |
|--------|-----------|-----------------------|-----------|
| Squared | $s$ | $1$ | outliers dominate |
| **Huber** | $s$ if $s\le\delta^2$, else $2\delta\sqrt{s}-\delta^2$ | $\min(1, \delta/\sqrt{s})$ | quadratic near zero, linear far (convex) |
| **Cauchy** | $\delta^2\log(1 + s/\delta^2)$ | $1/(1 + s/\delta^2)$ | influence goes to zero for large errors (non-convex) |

### 4.1 The IRLS view

Setting the gradient of $\sum\rho(s_i)$ to zero gives
$\sum_i\rho'(s_i)\,\mathbf{J}_i^\top\mathbf{r}_i = 0$. That is the gradient of a *weighted* least
squares problem with weights $w_i = \rho'(s_i)$. So a robust solver is a sequence of weighted
Gauss–Newton problems, re-weighted each iteration (**iteratively re-weighted least squares**). A
measurement with a large error simply gets a small weight. Ceres goes one step further and also
uses $\rho''$ to correct the Gauss–Newton Hessian [Triggs00].

**Choosing $\delta$.** On whitened residuals, $\delta\approx1$–$2$ (in σ units) is typical.
Kestrel uses Cauchy for visual residuals: it suppresses gross outliers almost completely. It is
not used for IMU residuals, because those are not outlier-prone, and a robust kernel there would
just weaken a good constraint.

**Limits.** Robust kernels need a reasonable initial guess (non-convexity), and they do not help
when outliers are *consistent* with each other, such as all features on one moving truck.
[Ch. 07](07_dynamic_objects.md) discusses this.

---

## 5. Factor graphs and sparsity

A **factor graph** is a bipartite graph with variable nodes (poses, velocities, biases, landmarks)
and factor nodes (one per residual), with an edge wherever a residual depends on a variable. It is
simply a picture of the cost function [Dellaert17]:

```
  [prior]─(x0)──[IMU]──(x1)──[IMU]──(x2)──[IMU]──(x3)
            \     \      |  \       |      /
           [cam] [cam] [cam] [cam] [cam] [cam]
              \     \    |   /      |    /
               (l1)  (l2)  ...     (l3)
```

The graph is **sparse**: each factor touches only a few variables. The structure of $\mathbf{H}$
follows the graph. Block $(a, b)$ is non-zero only if some factor touches both $a$ and $b$. For
visual–inertial problems:

* pose/velocity/bias blocks form a band (IMU factors link consecutive frames);
* landmark–landmark blocks are **block-diagonal** (no factor touches two landmarks);
* pose–landmark blocks are non-zero where a pose observes a landmark.

## 6. The Schur complement

Order the variables as poses $\mathbf{p}$ (few, ~15 per keyframe × 11 keyframes) and landmarks
$\boldsymbol\ell$ (many, ~200, but 1 parameter each with inverse depth):

$$
\begin{pmatrix}\mathbf{H}_{pp} & \mathbf{H}_{p\ell}\\ \mathbf{H}_{\ell p} & \mathbf{H}_{\ell\ell}\end{pmatrix}
\begin{pmatrix}\boldsymbol\delta_p\\ \boldsymbol\delta_\ell\end{pmatrix} =
-\begin{pmatrix}\mathbf{g}_p\\ \mathbf{g}_\ell\end{pmatrix}.
$$

Eliminate the landmarks. $\mathbf{H}_{\ell\ell}$ is block-diagonal, so it is trivial to invert:

$$
\big(\underbrace{\mathbf{H}_{pp} - \mathbf{H}_{p\ell}\mathbf{H}_{\ell\ell}^{-1}\mathbf{H}_{\ell p}}_{\text{Schur complement } \mathbf{S}}\big)\boldsymbol\delta_p
= -\big(\mathbf{g}_p - \mathbf{H}_{p\ell}\mathbf{H}_{\ell\ell}^{-1}\mathbf{g}_\ell\big),
\qquad
\boldsymbol\delta_\ell = -\mathbf{H}_{\ell\ell}^{-1}(\mathbf{g}_\ell + \mathbf{H}_{\ell p}\boldsymbol\delta_p).
$$

The reduced system $\mathbf{S}$ is small and dense (~165×165). Solve it, then back-substitute for
each landmark independently. This is Ceres's `DENSE_SCHUR` / `SPARSE_SCHUR` linear solver, and it
is why bundle adjustment with hundreds of landmarks runs in milliseconds.

The same algebra has a second meaning. In probability, the Schur complement **marginalizes**
variables out of a Gaussian. The marginal information of $\mathbf{p}$ is exactly $\mathbf{S}$.
[Ch. 06](06_sliding_window_vio.md) uses this to drop old keyframes while keeping what they taught us.

---

## 7. Gauge freedom

If the cost is invariant to some transformation of all variables, the global position and yaw in
VIO, then $\mathbf{H}$ is singular along those directions (the *gauge*). Solvers cope in one of
three ways:

1. **fix** some variables (e.g. hold the first pose constant),
2. add a weak **prior**,
3. let LM's damping keep the step bounded in the null-space directions.

In a sliding window, the marginalization prior plays the role of option 2.

---

## 8. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| Ceres problem setup, LM, `DENSE_SCHUR`, time budget | `kestrel/estimator/estimator.cpp` — **planned (M3)** |
| Robust kernels on visual factors (Cauchy) | `kestrel/estimator/factors.hpp` — **planned (M3)** |
| Numeric-vs-autodiff Jacobian tests for every factor | `test/test_factors.cpp` — **planned (M3)** |
| Pose-graph optimization (4-DoF) | `kestrel/loop/pose_graph.hpp` — **planned (M5)** |

---

## 9. Pitfalls

* **Forgetting to whiten.** A residual in metres mixed with one in radians, without covariances,
  gives a solution that depends on your choice of units.
* **Over-trusting the covariance** from $\mathbf{H}^{-1}$. It is only as good as the noise model,
  and with robust kernels it is approximate.
* **A robust kernel on everything.** It hides modelling bugs, because a wrong IMU sign gets
  "robustified" away instead of failing loudly.
* **Unbounded solve time.** Real-time systems cap the time and accept a slightly sub-optimal
  solution.

---

## 10. Exercises

1. Derive the Gauss–Newton step for the 1-D problem $\min_x\sum_i(\sin(x) - z_i)^2$ and show that
   it can diverge from a bad start. Then show how LM fixes it.
2. Show that IRLS with Huber weights converges to the Huber M-estimate for a 1-D location problem
   (estimating a mean with outliers). Try data $\{0, 0.1, -0.1, 0.05, 10\}$.
3. For a problem with 11 poses (15 parameters each) and 200 one-parameter landmarks, compare the
   cost of solving the full system densely with solving via the Schur complement. Count flops for
   a Cholesky factorization of each.
4. Prove that the Schur complement $\mathbf{S}$ equals the inverse of the pose block of
   $\mathbf{H}^{-1}$, i.e. that it is the information matrix of the marginal distribution.
