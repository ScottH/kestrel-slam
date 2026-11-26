# Recording Gemini 335Lg datasets for Kestrel

This guide explains how to record rosbags that are useful for developing, calibrating and
evaluating Kestrel. Everything runs in Docker; nothing needs installing on your laptop.

## 1. One-time setup

```bash
git clone <this repo> kestrel_slam && cd kestrel_slam
docker compose build          # ~10-15 min the first time (builds ROS 2 + the Orbbec driver)
```

Plug the camera into a **USB 3** port. A USB 2 link cannot carry stereo IR, depth and IMU at
30 Hz; the monitor will show low rates and dropped frames if that happens.

> **Gemini 335Lg (GMSL2) on a Jetson:** the GMSL deserializer and camera drivers are part of the
> Jetson's JetPack/BSP image and must be installed on the Jetson itself (Orbbec provides them).
> The container is the same either way.

## 2. Recording

```bash
docker compose run --rm capture session:=office scenario:=loop
```

* Recordings land in `./data/bags/<session>_<scenario>_<date_time>/` (MCAP, zstd-compressed), owned by
  your user.
* A `….capture.yaml` summary is written next to each bag (rates, dropped frames, stereo sync and
  warnings). Keep it with the bag.
* Stop with **Ctrl-C**, or pass `duration:=120` to stop automatically.
* `docker compose run --rm capture --help` lists every option.

While recording, the monitor prints a status block every 2 s:

```
[    12 s] recording — OK
  left_ir    30.0 Hz (want   30)  max gap   34.1 ms  drops 0
  right_ir   30.0 Hz (want   30)  max gap   34.0 ms  drops 0
  depth      30.0 Hz (want   30)  max gap   35.2 ms  drops 0
  imu       199.8 Hz (want  200)  max gap    5.6 ms  drops 0
  stereo   pairs 360  unmatched 0   image mean 92  sat 0.3%  corners 412
```

If the status says **CHECK**, read the ⚠ lines. Typical causes are a USB 2 port, too little light,
sunlight saturating the IR, or a blank wall filling the view.

### Watching live (optional)

In a second terminal, run `docker compose up foxglove`. Then open <https://app.foxglove.dev>, choose
"Open connection" and enter `ws://localhost:8765`. You can view the IR images and IMU plots in
the browser without any X11 setup.

## 3. Options that matter

| Argument | Default | Notes |
|----------|---------|-------|
| `laser_mode` | `off` | `off` gives clean IR for tracking. `interleave` alternates laser on/off frames (record both; SLAM uses the off-frames, mapping uses on-frame depth). `on` is for depth-only experiments. **Never track on laser-on frames:** the dots move with the camera. |
| `resolution` | `848x480` | `1280x800` is more accurate but costs more CPU; keep `fps` ≤ 30 at that size. |
| `fps` | `30` | 30 Hz is the sweet spot for VIO. |
| `imu_rate` | `200hz` | Use `500hz` or `1khz` for the IMU calibration recording. |
| `record_depth` | `true` | Used for the dense map and change detection, never for tracking. |
| `notes` | – | Write down what changed, the lighting, and anything unusual. |

## 4. What to record — the shopping list

Each row is one recording. Use the `scenario` names shown so the evaluation scripts can find them
later.

### A. Calibration (once per camera; redo if the camera is bumped or remounted)

| Scenario | How | Why |
|----------|-----|-----|
| `calib_camera` | Move the camera slowly in front of a printed **AprilGrid** (Kalibr's `6x6`, 2-3 cm tags), covering all image corners at several distances and tilts. 60-90 s. `fps:=10` is fine. | Stereo intrinsics and extrinsics (Kalibr). |
| `calib_imu` | Same AprilGrid, but **excite every axis**: brisk rotations about roll, pitch and yaw, plus up/down, left/right and forward/back translations. Keep the board in view and avoid motion blur. 60-90 s at `imu_rate:=500hz`. | Camera-to-IMU extrinsics and time offset (Kalibr). |
| `imu_static` | Camera **completely still** on a table, untouched, for 2-3 hours. Use `record_depth:=false` and `fps:=5` to keep the bag small. | Allan variance gives real IMU noise parameters. Optional, but it measurably improves VIO. |

### B. Odometry and loop-closure sequences

| Scenario | How |
|----------|-----|
| `static_start` | Hold still for 3 s (the monitor tells you when), then walk a short path. This is the basic sanity sequence. |
| `loop` | Walk a closed loop of 30-100 m and **end where you started, facing the same way**, so loop closure has something to find. Do 2-3 laps. |
| `figure8` | Two loops sharing a central corridor. This exercises loop detection from different headings. |
| `fast` | Faster motion and quicker turns. This is a robustness test. |
| `outdoor` | The same pattern outdoors (shade, then sun). Watch the saturation warning. |

Tips:

* Start every sequence with the 3 s static start; the system initializes faster and more accurately.
* Avoid pointing at blank walls or the floor for long; keep texture in view.
* If you can, mark a start/end point on the floor (tape). Returning precisely to it gives a free
  end-point error measurement even without ground truth.

### C. Changing-environment pairs (the important ones for lifelong mapping)

Record the **same route** twice in the same place, changing the environment in between:

1. `scenario:=before_change`: the space as it normally is.
2. Move **large objects**: shelves, carts, boxes, chairs, a parked vehicle. Remove some and add
   others. Write down what you did.
3. `scenario:=after_change notes:="moved 2 shelves from aisle 3 to the wall; removed pallet stack"`.

Variations worth having:

* the same pair at a different time of day (lighting change),
* a third session days later (`after_change_2`),
* a sequence with **people walking around** (`scenario:=dynamic`).

### D. Your robot (when available)

Mount the camera on the robot and drive the routes above. Note the mounting position (a photo
and rough measurements) so we can set up the `base_link → camera` transform.

## 5. Checking a recording afterwards

```bash
docker compose run --rm shell ros2 bag info /data/bags/<name>
cat data/bags/<name>.capture.yaml
```

The summary's `warnings_seen` should be empty, `stereo.unmatched` should be 0, and
`static_start_complete` should be `true`.

## 6. Sharing recordings

Bags are large: about 1.5-3 GB per minute at 848x480 with depth, after compression. Copy the bag
directory together with its `.capture.yaml` file.
