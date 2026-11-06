# 08 — Place Recognition and Loop Closure

> **Why this chapter exists.** Odometry drifts. Even excellent VIO accumulates a fraction of a
> percent of the distance travelled, plus an unbounded yaw error ([Ch. 06](06_sliding_window_vio.md) §6).
> **Loop closure** recognizes that the robot is back somewhere it has been, measures the
> relative pose to that earlier visit, and spreads the correction over the whole trajectory.
> The same machinery handles **relocalization** in a saved map and **merging** of separate
> sessions. A single false loop can destroy a map, so this chapter is as much about *rejecting*
> loops as about finding them.

Primary references: **[Sivic03]** (bag of words), **[Nister06]** (vocabulary tree),
**[GalvezLopez12]** (DBoW2), **[Rublee11]** (ORB), **[Lowe04]** (ratio test),
**[Kneip11]**, **[Lepetit09]** (P3P, EPnP), **[Qin18]** (4-DoF pose graph),
**[MurArtal15]**, **[Campos21]** (ORB-SLAM loop closing and Atlas), **[Grisetti10]** (graph SLAM).

---

## 1. Intuition

Recognizing a place is an *image retrieval* problem: given the current keyframe, find the most
similar past keyframe among thousands, in milliseconds. Comparing descriptors exhaustively is far
too slow. The bag-of-words idea, borrowed from text search, represents each image by a histogram of
"visual words". Retrieval then becomes a sparse vector comparison through an inverted index, just
as a search engine finds documents containing your query words. Retrieval only proposes
*candidates*, and **geometry** decides: a true revisit must admit a single rigid transform that
explains many feature matches.

---

## 2. Visual words and the vocabulary tree

**Descriptors.** At each keyframe Kestrel extracts ORB features [Rublee11]: FAST corners with a
256-bit rotated-BRIEF binary descriptor. These are compared by **Hamming distance** (bit
differences, a single `popcount`).

**Vocabulary.** Offline, cluster a large set of descriptors from training images. Each cluster
centre is a *word*. A flat vocabulary of $10^6$ words would make quantization slow, so use a
**hierarchical k-means tree** [Nister06]: cluster into $k$ groups, recursively cluster each
group, and repeat to depth $L$. The leaves are the words ($k^L$ of them, e.g. $10^6$ for
$k = 10$, $L = 6$). Quantizing a descriptor costs only $kL$ distance computations, by descending
the tree.

**Binary descriptors and k-majority.** The mean of binary vectors is not binary. DBoW2
[GalvezLopez12] uses **k-majority**: each bit of a cluster centre is the majority vote of that
bit across its members. This is the binary analogue of the mean under Hamming distance.

Kestrel ships its own vocabulary **trainer** rather than ORB-SLAM's vocabulary file, which is
GPL-licensed (see DEV_PLAN risks). Train on images similar to your deployment (indoor IR images
for indoor robots) for best results.

---

## 3. Scoring images: tf–idf and the L1 score

An image becomes a sparse vector $\mathbf{v}$ with one entry per word that occurs in it:

$$
v_w = \underbrace{\frac{n_{w}}{n}}_{\text{term frequency}}\cdot\underbrace{\log\frac{N}{N_w}}_{\text{inverse document frequency}},
$$

where $n_w$ is the number of features in this image quantized to word $w$, $n$ the total number
of features in the image, $N$ the number of training images, and $N_w$ the number of training
images containing $w$. Words that occur everywhere (e.g. corners of ceiling lights) get low
weight, while distinctive words get high weight.

The DBoW2 similarity between two L1-normalized vectors is

$$
s(\mathbf{v}_1, \mathbf{v}_2) = 1 - \frac12\left\|\frac{\mathbf{v}_1}{\|\mathbf{v}_1\|_1} - \frac{\mathbf{v}_2}{\|\mathbf{v}_2\|_1}\right\|_1\ \in [0, 1],
$$

which can be computed by touching only the words present in both images.

**Normalizing the score.** Absolute scores vary a lot between environments: a feature-rich
office scores higher than a bare corridor. DBoW2 normalizes by the score of the current image
against its *own recent past*, which is the best a true match could reasonably score:

$$
\eta(\mathbf{v}_t, \mathbf{v}_j) = \frac{s(\mathbf{v}_t, \mathbf{v}_j)}{s(\mathbf{v}_t, \mathbf{v}_{t-\Delta t})}.
$$

### 3.1 The inverted index

For every word, keep a list of the keyframes that contain it. A query visits only the lists of
its own words, accumulating scores per candidate keyframe. That way the cost grows with the
number of *relevant* keyframes, not all of them. A **direct index** (per keyframe: features
grouped by tree node at some level) also speeds up the feature matching in §5, because only
features under the same node need comparing.

---

## 4. From candidates to hypotheses

1. **Exclude the recent past.** The immediately preceding keyframes trivially look similar. Skip
   the last `exclude_recent_keyframes`, and only within the *same* sequence: a loaded map is never
   "recent".
2. **Threshold.** Keep candidates with score above `min_bow_score` (or $\eta$ above a threshold).
3. **Group into islands.** Consecutive keyframes in the database that match together are summed
   into an "island" score, so the best *place* wins rather than a single lucky keyframe.
4. **Temporal consistency.** Require the same island to be the winner for
   `temporal_consistency` consecutive queries. One-off coincidences rarely persist.

---

## 5. Geometric verification

A candidate is accepted only if geometry agrees:

1. **Match descriptors** between the current keyframe (2-D keypoints) and the candidate (keypoints
   with **3-D positions** from stereo triangulation, in the candidate's camera frame). Use the
   ratio test [Lowe04]: keep a match only if its best distance is clearly better than the second
   best.
2. **PnP-RANSAC.** Estimate the current camera pose relative to the candidate's 3-D points with a
   minimal **P3P** solver [Kneip11] inside RANSAC (35 iterations suffice for 50% inliers at 99%
   confidence, see [Ch. 04](04_feature_tracking.md) §5). Refine on the inliers with EPnP
   [Lepetit09] plus nonlinear refinement.
3. **Accept** only if the number of inliers is at least `min_inliers` (default 25) and the inlier
   ratio is reasonable.

Stereo is a real advantage here. The candidate keyframe's points have **metric 3-D positions**,
so PnP returns a metric relative pose directly, with no scale ambiguity as in monocular systems.

The output is a **loop edge** $\tilde{\mathbf{T}}_{B_mB_c}$, the relative pose between the matched
keyframe $m$ and the current keyframe $c$.

---

## 6. The 4-DoF pose graph

### 6.1 Why only 4 degrees of freedom?

VIO estimates roll and pitch against gravity: they are observable and accurate. Only **x, y, z
and yaw** drift ([Ch. 06](06_sliding_window_vio.md) §6). A loop correction should therefore
change only those four. Optimizing full 6-DoF poses would let a noisy loop edge tilt parts of the
map away from gravity, which is wrong, and it would conflict with the VIO [Qin18].

### 6.2 Nodes, edges, residuals

Each keyframe $i$ is a node with variables $(\mathbf{p}_i, \psi_i)$, position and yaw. Its roll
$\hat\theta_i$ and pitch $\hat\phi_i$ stay fixed at their VIO values. There are two kinds of
edges:

* **sequential edges** between neighbouring keyframes (the VIO's relative motion, very accurate
  over short spans),
* **loop edges** from §5.

For an edge $(i, j)$ with measured relative translation $\hat{\mathbf{p}}_{ij}$ (expressed in
frame $i$) and relative yaw $\hat\psi_{ij}$:

$$
\mathbf{r}_{ij} = \begin{pmatrix}
\mathbf{R}(\psi_i, \hat\theta_i, \hat\phi_i)^\top(\mathbf{p}_j - \mathbf{p}_i) - \hat{\mathbf{p}}_{ij}\\[2pt]
\mathrm{wrap}(\psi_j - \psi_i - \hat\psi_{ij})
\end{pmatrix}\in\mathbb{R}^4,
$$

where $\mathrm{wrap}$ maps an angle to $(-\pi, \pi]$. Loop edges get a robust kernel (Huber or
Cauchy), so that a wrong loop that slipped through verification is down-weighted rather than
obeyed. The oldest keyframe of the reference sequence is held fixed (gauge,
[Ch. 05](05_nonlinear_least_squares.md) §7).

### 6.3 Publishing the correction

The VIO keeps running in its own drifting **odom** frame and never jumps. That is essential for
control: a robot's velocity controller must not see a teleport. After optimization, the
correction is published as a single transform

$$
\mathbf{T}_{\text{map,odom}} = \mathbf{T}^{\text{corrected}}_{WB_{\text{last}}}\left(\mathbf{T}^{\text{VIO}}_{WB_{\text{last}}}\right)^{-1},
$$

which gives exactly the ROS REP-105 frame tree **map → odom → base_link**. Nav2 plans in `map`
and tracks in `odom`.

---

## 7. Sequences, relocalization and map merging

A **sequence** is one continuous run of VIO. A new sequence starts when tracking fails and the
estimator restarts ([Ch. 06](06_sliding_window_vio.md) §8), and when a session begins with a
**loaded map**, which becomes a fixed reference sequence.

* **Same-sequence loop:** add the edge and optimize.
* **Cross-sequence loop:** the two sequences have unrelated world frames. The loop edge gives the
  4-DoF transform between them. Move the newer sequence rigidly into the older one's frame,
  mark them merged, and optimize jointly. This mechanism provides **relocalization** in a saved
  map and **recovery** after tracking loss (VINS-Fusion's map merging and ORB-SLAM3's Atlas
  [Campos21] are the same idea).
* **Before the first cross-sequence loop**, the live sequence has no known relation to the map.
  Kestrel then publishes odometry only, and reports "not localized" instead of guessing.

---

## 8. Precision versus recall

A place-recognition system can be judged by:

* **Precision:** of the loops accepted, how many are correct?
* **Recall:** of the true revisits, how many were detected?

They trade off against each other through every threshold above. In SLAM the costs are wildly
asymmetric. A missed loop leaves some drift for a while longer. **A false loop** welds two
different places together, and the optimized map is then wrong *everywhere*. Robust kernels on
loop edges reduce the damage but do not eliminate it. Kestrel's acceptance criterion
for M5 is therefore **zero false loops** on the benchmark set, at whatever recall that allows.
Layered checks (score, island, temporal consistency, ratio test, PnP inliers, robust pose graph)
each remove a different kind of false positive.

Learned alternatives (global descriptors like NetVLAD [Arandjelovic16]; learned local features
and matchers like SuperPoint [DeTone18] with SuperGlue [Sarlin20] or LightGlue
[Lindenberger23]) are much more robust to lighting and viewpoint change. They are candidates for
the optional GPU plug-ins listed in the DEV_PLAN.

---

## 9. In Kestrel

| Concept | Where (planned) |
|---------|-----------------|
| ORB extraction + stereo 3-D on keyframes | `kestrel/loop/keyframe.hpp` — **planned (M5)** |
| Vocabulary (k-majority tree, tf–idf), trainer tool | `kestrel/loop/vocabulary.hpp`, `apps/train_vocabulary.cpp` — **planned (M5)** |
| Database with inverted index, islands, temporal consistency | `kestrel/loop/keyframe_database.hpp` — **planned (M5)** |
| PnP-RANSAC verification | `kestrel/loop/loop_closing.cpp` — **planned (M5)** |
| 4-DoF pose graph in its own thread, `T_map_odom` | `kestrel/loop/pose_graph.hpp` — **planned (M5)** |
| Sequences, load/save, merging | `kestrel/io/map_io.hpp` — **planned (M6)** |
| Parameters | `loop_closure:` section of the config (available now) |

---

## 10. Pitfalls

* **Perceptual aliasing:** identical corridors, shelving rows and office cubicles. Expect high
  BoW scores for *different* places, and lean on geometry and temporal consistency.
* **Vocabulary mismatch:** a vocabulary trained on outdoor RGB images works poorly on indoor IR
  images. Retrain.
* **Points without depth** in the candidate keyframe cannot be used for PnP. Keep enough stereo
  points per keyframe.
* **Correcting roll/pitch** with loop closures (6-DoF graph on VIO). It tilts the map.
* **Blocking tracking** while optimizing a large pose graph. Run it in a separate thread and only
  swap in the result.

---

## 11. Exercises

1. Compute the tf–idf vectors and the L1 score for two toy images over a 5-word vocabulary with
   word counts $(2,0,1,0,3)$ and $(1,1,0,0,3)$, and document frequencies $N_w = (50, 5, 80, 2, 10)$
   out of $N = 100$.
2. Show that the 4-DoF residual of §6.2 is invariant to a global translation and a global yaw
   rotation applied to all nodes.
3. A database of 5000 keyframes, 1000 features each, vocabulary $10^6$ words: estimate the
   average inverted-list length and the cost of one query.
4. Design a test for "zero false loops": what data would you use, and how would you label
   ground-truth revisits?
