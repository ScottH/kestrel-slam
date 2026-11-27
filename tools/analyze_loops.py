#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Check every verified loop closure of a run_vio output folder against ground truth.

    tools/analyze_loops.py <run_dir>

For each loop in <run_dir>/loops.txt (query body pose measured in the match body frame) the true
relative pose is computed from <run_dir>/groundtruth.tum (nearest samples) and the error is
reported. Wrong loops are the most damaging failure of a SLAM system, so this is the first thing
to look at when loop closure does not improve accuracy.
"""

import sys

import numpy as np


def quat_to_rot(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def pose(row):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(*row[3:7])
    T[:3, 3] = row[0:3]
    return T


def main() -> int:
    run = sys.argv[1]
    gt = np.loadtxt(f'{run}/groundtruth.tum')
    loops = np.loadtxt(f'{run}/loops.txt', ndmin=2)
    if loops.size == 0:
        print('no loops')
        return 0

    def gt_at(t):
        i = int(np.argmin(np.abs(gt[:, 0] - t)))
        return pose(gt[i, 1:8]), abs(gt[i, 0] - t)

    rows = []
    for l in loops:
        Tq, dq = gt_at(l[0])
        Tm, dm = gt_at(l[1])
        if max(dq, dm) > 0.02:
            continue
        T_true = np.linalg.inv(Tm) @ Tq
        T_meas = pose(l[2:9])
        dt = np.linalg.norm(T_true[:3, 3] - T_meas[:3, 3])
        R = T_true[:3, :3].T @ T_meas[:3, :3]
        dr = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
        rows.append((l[0], dt, dr, int(l[9]), l[10], np.linalg.norm(T_true[:3, 3])))
    rows = np.array(rows)
    print(f'{len(rows)} loops checked against ground truth')
    print(f'translation error: median {np.median(rows[:, 1]):.3f} m, '
          f'90% {np.percentile(rows[:, 1], 90):.3f} m, max {rows[:, 1].max():.3f} m')
    print(f'rotation error:    median {np.median(rows[:, 2]):.2f} deg, '
          f'90% {np.percentile(rows[:, 2], 90):.2f} deg, max {rows[:, 2].max():.2f} deg')
    bad = rows[(rows[:, 1] > 0.2) | (rows[:, 2] > 5)]
    print(f'{len(bad)} loops off by > 0.2 m or > 5 deg (likely wrong):')
    for r in bad[:15]:
        print(f'  t={r[0]:.2f}  err {r[1]:.3f} m {r[2]:.1f} deg  inliers {int(r[3])}  score {r[4]:.2f}'
              f'  true distance {r[5]:.2f} m')
    good = rows[(rows[:, 1] <= 0.2) & (rows[:, 2] <= 5)]
    if len(good):
        print(f'good loops: inliers median {np.median(good[:, 3]):.0f}; '
              f'bad loops: inliers median {np.median(bad[:, 3]) if len(bad) else float("nan"):.0f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
