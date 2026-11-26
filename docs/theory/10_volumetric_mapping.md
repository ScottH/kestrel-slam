# 10 — Volumetric Mapping and Change Detection

> **Why this chapter exists.** The sparse map of features is excellent for *localization* but
> useless for *navigation*: a planner needs to know which space is free and which is blocked.
> Kestrel therefore also builds a **dense volumetric map** from the camera's depth images, and
> because the environment changes, the map must be able to *un-learn* obstacles that were moved
> away and report where changes happened.

Primary references: **[Moravec85]**, **[Thrun05]** ch. 9 (occupancy grids),
**[Hornung13]** (OctoMap, clamping), **[Amanatides87]** (voxel traversal), **[Teschner03]**,
**[Niessner13]** (spatial hashing), **[Curless96]**, **[Newcombe11]**, **[Oleynikova17]**
(TSDF-based mapping).

---

## 1. Intuition

Chop space into small cubes (**voxels**, default 5 cm, `voxel_size_m`). For each voxel keep a
belief "how likely is it occupied?". Every depth pixel is a ray from the camera: the space along
the ray up to the measured surface was seen **free**, and the voxel at the end is **occupied**.
Accumulate many rays from many viewpoints and the belief sharpens. If something is removed, rays
start passing through where it was, and the belief must flip back to free. Making that flip
happen *quickly* is the main design problem for changing environments.

---

## 2. Occupancy in log-odds

For a voxel with occupancy probability $p$, store the **log-odds**
$l = \log\frac{p}{1-p}$. Under the standard assumption that voxels are independent and the world
is static between updates, Bayes' rule for a new measurement $z$ becomes a **sum**:

$$
l \leftarrow l + \underbrace{\log\frac{p(\text{occ}\mid z)}{1-p(\text{occ}\mid z)}}_{\text{inverse sensor model}} - l_0 ,
$$

where $l_0$ is the prior log-odds (0 for $p_0 = 0.5$). The **inverse sensor model** says how much
one observation shifts belief: $l_{\text{hit}} = \mathrm{logit}(p_{\text{hit}})$ for the endpoint
voxel, $l_{\text{miss}} = \mathrm{logit}(p_{\text{miss}})$ for voxels the ray passed through.
With OctoMap's defaults [Hornung13]:

| | probability | log-odds |
|---|---|---|
| hit | 0.7 | +0.847 |
| miss | 0.4 | −0.405 |
| clamp max | 0.97 | +3.476 |
| clamp min | 0.12 | −1.992 |

Additions instead of multiplications, and one float per voxel: cheap.

---

## 3. Clamping: what makes change possible

Without limits, a voxel observed occupied 100 times has $l = 84.7$. After the object is removed,
it takes $84.7/0.405\approx 210$ "free" observations before the voxel flips. In practice it never
does, and the map keeps a ghost of the pallet forever. The static-world assumption of §2 is
exactly what fails in a changing environment.

**Clamping** limits $l$ to $[l_{\min}, l_{\max}]$. A fully confident occupied voxel then needs
only $3.476/0.405\approx 9$ free observations to drop below $p = 0.5$. Clamping is effectively a
statement that *no observation history makes us more than 97% sure*, which is the honest thing to
say about a world that changes. It also bounds the dynamic range, which allows compact storage.

The clamping bounds set a trade-off. Tighter bounds react faster to change but are more easily
disturbed by noise and by people walking through ([Ch. 07](07_dynamic_objects.md)). Kestrel
starts from OctoMap's values and exposes them for tuning.

---

## 4. Ray casting: the 3-D DDA

For each depth pixel, we need the exact list of voxels the ray passes through.
**Amanatides–Woo** [Amanatides87] traverse a uniform grid with additions only:

1. Start in the voxel containing the ray origin $\mathbf{o}$, with direction $\mathbf{d}$.
2. For each axis $a$, compute `step[a]` $=\pm1$ (sign of $d_a$), `tMax[a]` (the ray parameter at
   which it crosses the first voxel boundary on that axis), and `tDelta[a]` $= s/|d_a|$ (the
   parameter distance between boundaries, with $s$ the voxel size).
3. Loop: move along the axis with the smallest `tMax`, i.e. `voxel[a] += step[a]` and
   `tMax[a] += tDelta[a]`, until reaching the endpoint voxel.

Every visited voxel except the last gets a *miss* update, and the last gets a *hit*. Two
refinements matter:

* **Deduplicate per scan.** Many rays from one depth image pass through the same voxels near the
  camera. Collect a *set* of free voxels and a set of occupied voxels per scan, give occupied
  priority, and apply each voxel's update once. Otherwise near-camera voxels get absurdly many
  updates (OctoMap does the same).
* **Range limit.** Stereo depth noise grows as $Z^2$ ([Ch. 02](02_cameras_and_stereo.md) §4.1).
  Beyond `max_integration_range_m` (default 5 m), Kestrel only clears free space up to the limit
  and does not insert the endpoint.

The cost per keyframe is roughly (number of rays) × (voxels per ray). With a 4× subsampled 848×480
depth image (~25 k rays) and up to 100 voxels per ray, that is a few million cheap operations, so
it runs comfortably at keyframe rate.

---

## 5. Storage: voxel hashing

A dense 3-D array for a 100 m × 100 m × 3 m building at 5 cm resolution has 240 million voxels,
mostly unknown. Instead, store only observed voxels in a **hash map** keyed by integer
coordinates $(\lfloor x/s\rfloor, \lfloor y/s\rfloor, \lfloor z/s\rfloor)$ [Niessner13]. A classic
hash function for such keys XORs the coordinates multiplied by large primes [Teschner03]:

$$
h(i,j,k) = (i\cdot 73856093)\oplus(j\cdot 19349663)\oplus(k\cdot 83492791).
$$

Hashing gives $O(1)$ access, unbounded extent (the robot can drive anywhere), and memory
proportional to the observed volume. OctoMap's octree is the alternative: it is more compact for
large uniform regions and supports multi-resolution queries, but is slower to update.

---

## 6. Change detection with hysteresis

For planning we need the *current* state. For lifelong mapping we also want to know **what
changed and where**. Each voxel carries a small state machine:

```
             l > l_occ for ≥ t_dwell                l < l_free for ≥ t_dwell
  UNKNOWN ───────────────────────────► OCCUPIED ────────────────────────────► FREE
     │                                    ▲                                    │
     └────── l < l_free for ≥ t_dwell ────┼──► FREE ── l > l_occ for ≥ t_dwell ┘
```

* Two thresholds $l_{\text{occ}} > 0 > l_{\text{free}}$ (**hysteresis**) prevent flickering when
  $l$ hovers around 0.
* A **dwell time** requires the new state to persist before it is confirmed. A person walking
  through a voxel for half a second is not a change.
* A confirmed transition OCCUPIED → FREE emits a **disappeared** event, and FREE → OCCUPIED an
  **appeared** event, with timestamp and voxel. UNKNOWN → anything is just exploration, not change.

The events are published as a point cloud (`/changes`) and aggregated over time. They feed both
the operator ("what moved in the warehouse today?") and the sparse map: a disappeared region is a
strong hint that persistence filters ([Ch. 09](09_lifelong_mapping.md)) of points there should be
re-evaluated.

---

## 7. A 2-D grid for Nav2

Nav2 consumes `nav_msgs/OccupancyGrid`: a 2-D grid with values 0 (free), 100 (occupied) and −1
(unknown). Project the voxel map onto it column by column, within a height band
$[z_{\min}, z_{\max}]$ (`grid_min_height_m`, `grid_max_height_m`). The band excludes the floor and
anything above the robot:

* **occupied** if any voxel in the band is OCCUPIED,
* **free** if at least one voxel in the band was observed and all observed ones are FREE,
* **unknown** otherwise.

The floor itself must not count as an obstacle. That is why $z_{\min}$ sits a few centimetres
above the floor height, which is known after gravity alignment.

---

## 8. Keeping the map consistent with loop closures

Voxels are integrated with the poses known *at the time*. When a loop closure corrects the
trajectory by 30 cm, the voxel map is now inconsistent: a double wall appears. There are three
options:

1. **Rebuild:** keep a subsampled point cloud per keyframe in its *own* camera frame, and when the
   pose graph moves keyframes significantly, clear the map and re-integrate all clouds with the
   new poses. This is simple and exact, and Kestrel's choice, run in a background thread.
2. **De-integration:** subtract a keyframe's old contribution and re-add it at the new pose, as
   done for TSDF fusion in BundleFusion. This is cheaper per correction but needs to remember
   each update.
3. **Submaps:** integrate into local submaps attached to keyframes and move the submaps rigidly
   (Voxblox-style extensions). This scales best to very large maps, and is a future option.

---

## 9. Occupancy versus TSDF

| | Occupancy (log-odds) | TSDF (truncated signed distance) [Curless96, Newcombe11, Oleynikova17] |
|---|---|---|
| Stores | probability occupied | distance to nearest surface + weight |
| Best for | planning, free-space reasoning, *change* | high-quality surface reconstruction (meshes) |
| Free space | explicit | only within the truncation band (explicit free space needs extra handling) |
| Handling change | natural with clamping | weights must be capped or decayed |
| Cost | lower | higher (weighted averages near surfaces) |

Kestrel's goals are navigation and change detection, so it uses occupancy. A TSDF layer for
meshing could be added later.

---

## 10. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| `VoxelMap`: hashing, DDA ray casting, clamped log-odds, per-scan dedup | `kestrel/mapping/voxel_map.hpp` — **planned (M8)** |
| Hysteresis state machine and change events | `kestrel/mapping/voxel_map.hpp` — **planned (M8)** |
| 2-D grid projection, rebuild after loop correction | **planned (M8)** |
| ROS topics `/voxels`, `/changes`, `/map` | `kestrel_ros` — **planned (M9)** |
| Parameters | `mapping:` section of the config (available now) |

---

## 11. Pitfalls

* **Integrating with an unconverged pose** (right after initialization or relocalization) smears
  the map. Integrate only keyframes with confident, loop-consistent poses.
* **Depth artifacts:** stereo depth "flying pixels" at object edges create false obstacles.
  Filter depth discontinuities, and require several hits before a voxel becomes OCCUPIED (that is
  part of what the dwell time does).
* **Laser projector off** means passive stereo depth only. Blank walls then produce holes, which
  become *unknown*, not free. That is correct behaviour, but it surprises people.
* **Dynamic objects** (people) passing through leave transient occupied voxels. Clamping and dwell
  time handle them, and semantic masks ([Ch. 07](07_dynamic_objects.md)) can exclude them from
  integration entirely.

---

## 12. Exercises

1. With the table of §2, how many consecutive *hits* does an unknown voxel need to exceed $p = 0.9$?
   How many *misses* does a clamped-occupied voxel need to fall below $p = 0.2$?
2. Implement the 3-D DDA in Python and verify on random rays that it visits exactly the voxels a
   brute-force sampling at 1/100 of the voxel size visits.
3. Estimate the memory of a hashed map of a 50 m × 30 m single-floor warehouse at 5 cm resolution,
   assuming only voxels within 0.5 m of observed surfaces or along observed rays are stored. Use
   16 bytes per voxel plus hash overhead.
4. Propose a rule that decides when a loop correction is large enough to trigger a full rebuild
   (§8). What quantities would you threshold?
