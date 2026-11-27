# Benchmarks

Only **measured** numbers go here, each with the commit, configuration and command that produced
it, so anyone can reproduce them with Docker. Comparisons with other systems will be added from
their papers' tables, with citations, never from memory.

## How to reproduce

```bash
export KESTREL_DATA_DIR=~/code/kestrel_slam/data   # datasets under $KESTREL_DATA_DIR/datasets
docker compose run --rm build
docker compose run --rm shell /ws/src/kestrel_slam/tools/benchmark_tumvi.sh      # rooms 1-6
```

TUM-VI "euroc" exports (512×512, 16-bit) are available from
`https://cdn2.vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room<N>_512_16.tar`
(≈ 1.7 GB each, ~5 MB/s). Extract to `$KESTREL_DATA_DIR/datasets/tumvi/room<N>_512_16/`.

Metrics (`tools/evaluate.py`, evo):

* **ATE RMSE**: absolute trajectory error after SE(3) Umeyama alignment (no scale: stereo knows
  metric scale).
* **RPE 1 m**: relative translation error over 1 m segments (drift per metre).

## M3: stereo-inertial odometry (VIO only, no loop closure)

Commit: branch `m3` (2026-11-26) · config `kestrel_core/config/tumvi_512.yaml` · x86 laptop CPU
inside Docker.

| Sequence | Path [m] | ATE RMSE [m] | RPE 1 m [m] | Resets | ms / frame (median) |
|---|---|---|---|---|---|
| room1 | 146.4 | 0.063 | 0.019 | 0 | 27.9 |
| room2 | 141.0 | 0.183 | 0.019 | 0 | 28.1 |
| room3 | 135.0 | 0.069 | 0.016 | 0 | 27.8 |
| room4 | 68.3 | 0.049 | 0.020 | 0 | 30.1 |
| room5 | 131.3 | 0.115 | 0.018 | 0 | 38.4 * |
| room6 | 66.9 | 0.065 | 0.021 | 0 | 25.1 |
| **mean** | | **0.091** | **0.019** | **0** | |

\* room5 ran while another dataset was being extracted, so its timing is inflated.

Notes

* All six sequences run to completion with zero resets. Drift per metre is consistent
  (1.6–2.1 cm).
* room2's larger ATE is accumulated drift in its inner loops (peak 34 cm). The estimate keeps the
  correct shape throughout. Loop closure (M5) is what removes this kind of error.
* TUM-VI cameras run at 20 Hz, so 28 ms/frame is real-time with margin. The keyframe solve is
  the largest cost (median ≈ 28 ms) and is the first target for M10 performance work.

### EuRoC (machine hall)

Config `kestrel_core/config/euroc.yaml` (dataset calibration, default front end). Full sequences
from frame 0; both start with the drone being picked up, which the motion-aware initialization
detects.

| Sequence | Path [m] | ATE RMSE [m] | RPE 1 m [m] | Resets | ms / frame (median / p95) |
|---|---|---|---|---|---|
| MH_01_easy | 80.5 | 0.065 | 0.023 | 0 | 39.1 / 94.5 |
| MH_02_easy | 73.4 | 0.043 | 0.018 | 0 | 39.5 / 91.5 |

At 752×480 the p95 frame time (~92 ms) exceeds the 50 ms budget of 20 Hz: the keyframe solve
(median ≈ 50 ms) is the bottleneck. Performance work is scheduled (M10). Reproduce with
`run_vio /data/datasets/euroc/MH_01_easy /ws/src/kestrel_slam/kestrel_core/config/euroc.yaml
/data/results/euroc_MH_01_easy` and `tools/evaluate.py`.

### Tuning log (room1)

| Change | ATE RMSE | Notes |
|---|---|---|
| First run | 0.082 m | 20 resets (cheirality → indeterminate landmarks) |
| + weak landmark prior (5 m) | 0.061 m | 0 resets |
| + motion-aware static init (no bogus gyro bias when starting in motion) | 0.072 m | correct init; RPE 1.97 → 1.85 cm |
| + front end FB 1.0 px, RANSAC 2.0 px (CLAHE on) | **0.063 m** | |
| (front end without CLAHE) | 0.120 m | ~25 % fewer stereo matches; rejected |

### Front end on real images (room1, `run_frontend`)

| Setting | Feature age in view (median, frames) | Stereo matches / frame | Tracks lost / frame | ms / frame |
|---|---|---|---|---|
| defaults (CLAHE, FB 0.5 px, RANSAC 1 px) | 5 | 114 | 26 % | 22 |
| no CLAHE, FB 1.0, RANSAC 2.0 | 7 | 85 | 18 % | 13 |
