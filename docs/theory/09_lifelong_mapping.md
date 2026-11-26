# 09 — Lifelong Mapping in Changing Environments

> **Why this chapter exists.** This is Kestrel's main reason to exist. A map built on Monday
> describes Monday. By Friday pallets have moved, a shelf has been added, a door is open that was
> closed. If the map still believes in Monday's pallet, the robot will try to localize against
> features that no longer exist, or worse, against a pallet that now sits somewhere else. This
> chapter shows how to decide, *with probabilities rather than ad-hoc counters*, when a map
> feature has gone.

Primary references: **[Rosen16]** (the persistence filter, the core of this chapter),
**[Biber05]** (dynamic maps at multiple timescales), **[Churchill13]** (experience-based
navigation), **[Muhlfellner16]** (summary maps), **[Krajnik17]** (FreMEn), **[Labbe19]**
(RTAB-Map memory management), **[Shi20]** (OpenLORIS-Scene, a benchmark for exactly this problem).

---

## 1. Intuition: semi-static worlds

Split the world by how long things stay put:

* **static:** walls, columns, fixed shelving. Lifetime: the building's;
* **semi-static:** pallets, furniture, parked vehicles, boxes. They stay put for hours to weeks,
  then move;
* **dynamic:** people, moving vehicles. They move while observed ([Ch. 07](07_dynamic_objects.md)).

Semi-static objects are the hard case. While they are present they are *excellent* landmarks,
large and textured. When they leave, they become *misleading* ones. We want to keep using them
while they exist and drop them soon after they disappear, without dropping features just because
they were occluded for a moment.

The key insight of Rosen et al. [Rosen16]: model each feature's **survival time** $T$ explicitly,
and combine a prior over $T$ with a sequence of noisy detect/miss observations, using Bayes' rule.

---

## 2. The model

For one map feature (a map point in Kestrel):

* $T\ge0$: the unknown time at which the feature *vanishes*. Its prior has **survival function**
  $S(t) = P(T > t)$, with CDF $F(t) = 1 - S(t)$.
* $X_t = \mathbb{1}[T > t]$: whether the feature still exists at time $t$.
* At times $t_1 < t_2 < \dots < t_N$ we check whether the feature *should* be visible and obtain
  $y_i\in\{0,1\}$: **detected** (1) or **missed** (0).
* The detector is imperfect:
  $$P(y = 0\mid X = 1) = P_M\ (\text{missed detection}),\qquad P(y = 1\mid X = 0) = P_F\ (\text{false alarm}).$$
  These are `persistence_miss_probability` and `persistence_false_alarm_probability` in the config.

Observations are conditionally independent given $T$.

## 3. The posterior

Let $Y_N = (y_1,\dots,y_N)$. If the feature vanished at time $T$, the observations before $T$ came
from a present feature and those after it from an absent one:

$$
p(Y_N\mid T) = \prod_{t_i\le T}p(y_i\mid X=1)\prod_{t_i> T}p(y_i\mid X=0).
$$

This likelihood is **piecewise constant in $T$**. It only changes when $T$ crosses an observation
time. With $t_0 = 0$ and $t_{N+1} = \infty$, define for $i = 0,\dots,N$

$$
L_i = \prod_{k=1}^{i}p(y_k\mid 1)\prod_{k=i+1}^{N}p(y_k\mid 0),
\qquad\text{the likelihood if } T\in[t_i, t_{i+1}).
$$

Then the evidence (marginal likelihood) is a finite sum:

$$
p(Y_N) = \sum_{i=0}^{N}L_i\,\big[F(t_{i+1}) - F(t_i)\big].
$$

For a query time $t\ge t_N$, the event $X_t = 1$ means $T>t$, so all $N$ observations came
from a present feature, with likelihood $L_N$:

$$
\boxed{\ p(X_t = 1\mid Y_N) = \frac{L_N\,S(t)}{p(Y_N)}\ }
$$

### 3.1 A recursive, constant-time update

Storing all observations would make each update $O(N)$. Instead, keep three running quantities:

* $\Lambda_N = L_N = \prod_{k\le N}p(y_k\mid1)$,
* the *partial evidence* $E^-_N = \sum_{i=0}^{N-1}L_i[F(t_{i+1}) - F(t_i)]$ (all terms except $i=N$),
* the last observation time $t_N$.

When $y_{N+1}$ arrives at $t_{N+1}$, every hypothesis "$T$ before $t_{N+1}$" (all old terms,
**plus** the old open-ended term now truncated to $[t_N, t_{N+1})$) gains a factor $p(y_{N+1}\mid0)$:

$$
\begin{aligned}
E^-_{N+1} &= p(y_{N+1}\mid 0)\,\Big[E^-_N + \Lambda_N\big(F(t_{N+1}) - F(t_N)\big)\Big],\\
\Lambda_{N+1} &= \Lambda_N\,p(y_{N+1}\mid 1),\\
p(Y_{N+1}) &= E^-_{N+1} + \Lambda_{N+1}\,S(t_{N+1}).
\end{aligned}
$$

The update is $O(1)$ per observation and needs only three numbers per map point, so it is cheap
enough for every point in a large map. Because these products underflow after a few hundred
observations, implement them in **log space**, using
$\log(e^a + e^b) = \max(a,b) + \log(1 + e^{-|a-b|})$.

### 3.2 Choosing the survival prior

The simplest choice is **exponential**: $S(t) = e^{-t/\tau}$ with expected lifetime $\tau$
(`persistence_expected_lifetime_s`, default one week). It is memoryless, so the probability of
vanishing in the next hour does not depend on how long the feature has already existed. Rosen et
al. also discuss general priors, including uninformative ones over unknown timescales [Rosen16].
A natural extension is a per-class lifetime: walls ~years, furniture ~weeks, boxes ~days.

### 3.3 Worked examples (why this is better than counting misses)

Take $P_M = 0.2$, $P_F = 0.05$, $\tau = 7$ days. A point is detected once at $t_1\approx0$, then
missed $n$ times in a row at about the same later time $t$.

**Revisit after one day** ($F(1\,\text{d}) = 1 - e^{-1/7} = 0.133$). Approximately (all misses
at the same $t$):

$$
p(X_t = 1\mid Y) \approx \frac{0.867\cdot0.8\cdot0.2^{n}}{0.867\cdot0.8\cdot0.2^{n} + 0.133\cdot0.8\cdot0.95^{n}}
$$

| misses $n$ | 1 | 2 | 3 |
|------------|---|---|---|
| $p(\text{still there})$ | 0.58 | 0.22 | 0.06 |

**Revisit after 10 minutes** ($F = 1 - e^{-600/604800}\approx 0.00099$):

| misses $n$ | 3 | 5 | 6 |
|------------|---|---|---|
| $p(\text{still there})$ | 0.90 | 0.29 | 0.08 |

The **same evidence means different things depending on elapsed time**. Ten minutes after seeing
a pallet, three misses are probably occlusion or bad lighting. A day later, three misses mean it
is gone. A fixed "delete after $k$ misses" rule cannot express this. The prior does it
automatically.

---

## 4. Getting good observations: occluded versus gone

The filter is only as good as its $y_i$. For each stored map point that *should* be in the
current keyframe's view, Kestrel decides between **detected**, **missed** and **no observation**
using geometry and the depth image:

1. **Should it be visible?** Project the point (with the current, loop-corrected pose) into the
   current camera. It must fall inside the image, in front of the camera, within the viewing-angle
   and distance range at which its descriptor is valid.
2. **Compare predicted depth $\hat d$ with measured depth $d$** at that pixel (stereo or on-board
   depth, [Ch. 02](02_cameras_and_stereo.md)), using a tolerance $\tau_d$ that grows with
   $\hat d^2$ (depth noise model):
   * $d < \hat d - \tau_d$: **something is in front.** The point is occluded, possibly by a newly
     *appeared* object. → **No observation** (do not penalize the point).
   * $|d - \hat d|\le\tau_d$: the surface is where we expect. If a current descriptor matches
     nearby → **detected** ($y=1$), else → **missed** ($y=0$), e.g. the object was replaced by a
     different one at the same spot.
   * $d > \hat d + \tau_d$: **we see through where the point should be.** The surface is gone.
     → **missed**, and strong evidence: the depth itself contradicts the point's existence.
   * no valid depth: fall back to descriptor matching only, with a higher effective $P_M$.

This turns depth into evidence. Without it, occlusion and disappearance look identical, both as a
missing descriptor match, and the filter would either delete occluded features or keep vanished
ones for too long.

---

## 5. What the map does with the posteriors

* **Pruning:** remove map points whose $p(X_t = 1\mid Y)$ falls below
  `persistence_prune_threshold` (default 0.1).
* **Stale keyframes:** if most of a keyframe's points are pruned, remove it from the
  place-recognition database ([Ch. 08](08_place_recognition_and_loop_closure.md)). Its pose stays
  in the pose graph for consistency, but it will no longer cause false or failed relocalizations
  against a scene that no longer exists.
* **New structure:** live keyframes over a changed area are added to the map as usual, so the map
  *grows into* the new configuration. The next session localizes against the updated map.
* **Persistence across sessions:** the three filter numbers per point are saved with the map
  (M6), so evidence accumulates over days.

The dense voxel map handles the same problem for *geometry* with a different tool
([Ch. 10](10_volumetric_mapping.md)).

---

## 6. Other approaches to lifelong mapping

* **Multiple timescales** [Biber05]: maintain several maps that forget at different rates, and
  pick the one that explains the current data best.
* **Experience-based navigation** [Churchill13]: instead of one map, store multiple
  "experiences" of the same place (day, night, before and after) and localize against whichever
  matches. Robust to appearance change, at the cost of memory.
* **Summary maps** [Muhlfellner16]: score landmarks by how useful they have been for
  localization over many sessions, and keep only the best.
* **Periodic dynamics (FreMEn)** [Krajnik17]: model each feature's presence as a sum of periodic
  components (daily or weekly patterns, such as doors open during working hours) and predict
  future states.
* **Memory management** [Labbe19]: RTAB-Map moves old locations between working and long-term
  memory to bound computation in large, long-running maps.

The persistence filter fits Kestrel because it is per-feature, has $O(1)$ cost, rests on
probability, and composes well with the others. Multiple experiences and periodic models are
natural future extensions.

---

## 7. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| `PersistenceFilter` (log-space recursion, exponential prior) | `kestrel/mapping/persistence_filter.hpp` — **planned (M7)** |
| Visibility test, depth-based occlusion vs. disappearance | `kestrel/loop/map_manager.cpp` — **planned (M7)** |
| Pruning, stale keyframes, serialization of filter state | **planned (M6/M7)** |
| Tests: recursion vs. closed form; synthetic "object removed" scenario | `test/test_persistence.cpp` — **planned (M7)** |
| Parameters | `mapping.persistence_*` in the config (available now) |

---

## 8. Pitfalls

* **Observation dependence.** The model assumes conditionally independent observations. Twenty
  consecutive frames at 30 Hz of the same view are *not* independent evidence. Update at most once
  per keyframe (or per time interval) per point.
* **Pose error looks like change.** If the current pose is off by 20 cm, projected points miss
  their surfaces and "disappear". Only update persistence when localization is confident
  (recently verified loop or relocalization).
* **Lighting changes** reduce descriptor matches (higher effective $P_M$). This is why the
  depth-based evidence of §4 matters, and why IR (with its own illumination) helps.
* **Over-trusting the prior.** A very long lifetime $\tau$ makes the map stubborn. A very short one
  deletes things during a lunch break. Tune it on recordings of your environment (record "before"
  and "after" bags with the capture tool).

---

## 9. Exercises

1. Derive the recursion of §3.1 from the definition of $p(Y_N)$, by checking how each $L_i$
   changes when $y_{N+1}$ is appended.
2. Implement the filter in 40 lines of Python in log space and reproduce the two tables of §3.3
   (exactly, without the "all misses at the same time" approximation).
3. Show that with $P_F = 0$ and $P_M = 0$ (perfect detector), a single miss drives the posterior to
   zero, and a single detection after a miss is impossible. Why do real detectors need non-zero
   $P_M, P_F$?
4. A door is open 9:00–17:00 and closed otherwise. What does the persistence filter conclude for
   features on the door after a week of observations at random times? What would FreMEn conclude?
