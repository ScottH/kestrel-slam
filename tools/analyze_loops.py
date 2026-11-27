#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Check every verified loop closure of a run_vio output folder against ground truth.

    tools/analyze_loops.py <run_dir>

For each loop in <run_dir>/loops.txt (query body pose measured in the match body frame) the true
relative pose is computed from <run_dir>/groundtruth.tum (nearest samples) and the error is
reported. Wrong loops are the most damaging failure of a SLAM system, so this is the first thing
to look at when loop closure does not improve accuracy. Also importable: `audit(run_dir)`.
"""

import pathlib
import sys

import numpy as np

# A loop whose measured relative pose is off by more than this is counted as wrong.
WRONG_TRANSLATION_M = 0.2
WRONG_ROTATION_DEG = 5.0


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


def audit(run_dir):
    """Per-loop errors vs. ground truth. Returns a dict, or None without loops/ground truth."""
    run_dir = pathlib.Path(run_dir)
    gt_file, loop_file = run_dir / 'groundtruth.tum', run_dir / 'loops.txt'
    if not gt_file.exists() or not loop_file.exists():
        return None
    gt = np.loadtxt(gt_file)
    loops = np.loadtxt(loop_file, ndmin=2)
    if loops.size == 0:
        return {'checked': 0, 'wrong': 0, 'rows': []}

    def gt_at(t):
        i = int(np.argmin(np.abs(gt[:, 0] - t)))
        return pose(gt[i, 1:8]), abs(gt[i, 0] - t)

    rows = []
    for l in loops:
        Tq, dq = gt_at(l[0])
        Tm, dm = gt_at(l[1])
        if max(dq, dm) > 0.02:  # no ground truth close enough in time (e.g. mocap gap)
            continue
        T_true = np.linalg.inv(Tm) @ Tq
        T_meas = pose(l[2:9])
        dt = float(np.linalg.norm(T_true[:3, 3] - T_meas[:3, 3]))
        R = T_true[:3, :3].T @ T_meas[:3, :3]
        dr = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
        rows.append({'t': float(l[0]), 'translation_error_m': dt, 'rotation_error_deg': dr,
                     'inliers': int(l[9]), 'score': float(l[10]),
                     'distance_m': float(np.linalg.norm(T_true[:3, 3]))})
    wrong = [r for r in rows if r['translation_error_m'] > WRONG_TRANSLATION_M
             or r['rotation_error_deg'] > WRONG_ROTATION_DEG]
    te = [r['translation_error_m'] for r in rows] or [float('nan')]
    re = [r['rotation_error_deg'] for r in rows] or [float('nan')]
    return {
        'checked': len(rows),
        'wrong': len(wrong),
        'translation_error_median_m': round(float(np.median(te)), 4),
        'translation_error_p90_m': round(float(np.percentile(te, 90)), 4),
        'rotation_error_median_deg': round(float(np.median(re)), 3),
        'rows': rows,
        'wrong_rows': wrong,
    }


def main() -> int:
    result = audit(sys.argv[1])
    if result is None:
        print('no loops.txt or groundtruth.tum')
        return 1
    print(f"{result['checked']} loops checked against ground truth")
    if not result['checked']:
        return 0
    print(f"translation error: median {result['translation_error_median_m']:.3f} m, "
          f"90% {result['translation_error_p90_m']:.3f} m")
    print(f"rotation error:    median {result['rotation_error_median_deg']:.2f} deg")
    print(f"{result['wrong']} loops off by > {WRONG_TRANSLATION_M} m or > {WRONG_ROTATION_DEG} deg "
          f"(likely wrong):")
    for r in result['wrong_rows'][:15]:
        print(f"  t={r['t']:.2f}  err {r['translation_error_m']:.3f} m "
              f"{r['rotation_error_deg']:.1f} deg  inliers {r['inliers']}  score {r['score']:.2f}"
              f"  true distance {r['distance_m']:.2f} m")
    return 0


if __name__ == '__main__':
    sys.exit(main())
