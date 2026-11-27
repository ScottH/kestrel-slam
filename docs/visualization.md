# Visualizing Kestrel in Foxglove

Kestrel streams its internals through the [Foxglove SDK](https://github.com/foxglove/foxglove-sdk)
(MIT), so you can **watch a run live** or **record it to an MCAP file** and scrub through it
later. No ROS is needed: the messages use Foxglove's built-in types.

## Live

```bash
export KESTREL_DATA_DIR=~/code/kestrel_slam/data
docker compose run --rm build
docker compose run --rm shell \
  /ws/install/kestrel_core/lib/kestrel_core/run_vio \
  /data/datasets/tumvi/room1_512_16 \
  /ws/src/kestrel_slam/kestrel_core/config/tumvi_512.yaml \
  /data/results/live --foxglove --rate 1.0
```

In Foxglove: **Open connection → Foxglove WebSocket → `ws://<machine>:8765`**. The containers
use host networking, so other machines on your network can connect too (port 8765). `--rate 1.0`
paces processing to real time; leave it out to run as fast as possible.

## Recorded (MCAP)

Add `--mcap /data/results/room1.mcap` (it can be combined with `--foxglove`). Then open the file in
Foxglove (**Open local file**) on any machine. Images are stored as JPEG; a 2-minute TUM-VI run is
a few hundred MB.

## Layout

Import **`tools/foxglove/kestrel_layout.json`** (Layouts → Import from file). It sets up:

| Panel | Shows |
|---|---|
| **3D** (follows `body`) | keyframe trajectory `/vio/path`, aligned ground truth `/groundtruth/path`, landmarks in the window `/vio/landmarks`, camera frustum `/vio/camera`, the **factor graph** of the estimation window `/vio/factors` (orange keyframe nodes, green IMU factors, cyan rays = visual factors of the newest keyframe), the **pose graph** `/slam/graph` (grey odometry edges, **magenta loop closures**), the loop-corrected path `/slam/path`; TF `map → world → body → camera_left` and `body_gt` |
| **Image** | `/camera/left/image` with `/camera/left/features`: dots coloured by track age (red = new → green = ≥ 20 frames), lines = motion since the last frame, cyan rings = stereo match |
| **Plot: Front end** | features, stereo matches, new detections, losses by cause (forward–backward, RANSAC, KLT) |
| **Plot: Estimator / factor graph** | visual and IMU factors in the window, landmarks, new landmarks, outliers rejected, loop closures so far |
| **Plot: Timing** | total / front-end / solve time per frame |
| **Plot: Error** | position error vs. (first-pose aligned) ground truth, speed |
| **Plot: IMU bias** | gyro (and accelerometer) bias estimates |
| **Log** | `/log`: initialization, resets, warnings |

The layout file was written by hand. If a panel opens empty in your Foxglove version, open its
settings and pick the topic from the table above. Every message path for plots is
`/vio/stats.<field>`.

## Reading the views (what to look for)

* **Red dots everywhere, few green**: tracks are short-lived. Look at the loss plot to see which
  check is responsible.
* **Error rising steadily while the 3D view looks smooth**: odometry drift. Watch for magenta
  loop edges in `/slam/graph`: each one pulls the map back, and the error plot drops.
* **Few cyan rays from the newest keyframe**: few visual factors, so the IMU carries the
  estimate; expect drift until texture returns.
* **Spikes in `solve_ms`**: keyframes with many new landmarks; compare with `new_landmarks`.
* **Bias estimates still moving after the first ~10 s**: weak excitation or a noise model that
  doesn't match the IMU.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/tf` | `foxglove.FrameTransform` | `world → body`, `body → camera_left`, `world → body_gt` |
| `/camera/left/image` | `foxglove.CompressedImage` | JPEG |
| `/camera/left/features` | `foxglove.ImageAnnotations` | tracks |
| `/vio/pose` | `foxglove.PoseInFrame` | every frame |
| `/vio/path` | `foxglove.PosesInFrame` | keyframes |
| `/groundtruth/path` | `foxglove.PosesInFrame` | aligned to the first estimated pose |
| `/vio/landmarks` | `foxglove.PointCloud` | x, y, z, id; at keyframes |
| `/vio/camera` | `foxglove.SceneUpdate` | frustum, frame-locked to `camera_left` |
| `/vio/stats` | JSON (`kestrel.FrameStats`) | numbers for plots (incl. `loops`) |
| `/vio/factors` | `foxglove.SceneUpdate` | window factor graph, at keyframes |
| `/vio/factor_stats` | JSON (`kestrel.FactorStats`) | factor counts by type |
| `/slam/graph` | `foxglove.SceneUpdate` | pose graph: nodes, odometry edges, loop closures (frame `map`) |
| `/slam/path` | `foxglove.PosesInFrame` | loop-corrected keyframe path (frame `map`) |
| `/log` | `foxglove.Log` | estimator messages |
