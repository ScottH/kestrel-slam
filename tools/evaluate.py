#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Score a run_vio output folder against its ground truth with evo.

    tools/evaluate.py <run_dir> [--plot]

Reads <run_dir>/estimate.tum and <run_dir>/groundtruth.tum (written by run_vio) and reports:

* ATE (absolute trajectory error): RMSE of position error after aligning the estimate to the
  ground truth with a rigid SE(3) transform (Umeyama, no scale). Stereo-inertial systems know
  metric scale, so scale must NOT be corrected; see docs/theory/11_evaluation.md.
* RPE (relative pose error) over 1 m segments: drift per metre, independent of where errors
  happened.

Results are printed and saved to <run_dir>/metrics.json (and plots if --plot is given).
Runs inside the dev container (`docker compose run --rm shell tools/evaluate.py ...`), where
evo is installed.
"""

import argparse
import json
import pathlib
import sys

from evo.core import metrics, sync
from evo.tools import file_interface


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('run_dir', type=pathlib.Path)
    parser.add_argument('--plot', action='store_true', help='save trajectory/error plots')
    parser.add_argument('--estimate', default='estimate.tum',
                        help='trajectory file in run_dir to score: estimate.tum (odometry), '
                             'estimate_map.tum (online, loop-corrected) or keyframes_map.tum '
                             '(final pose graph)')
    parser.add_argument('--max-diff', type=float, default=0.01,
                        help='max timestamp difference for association [s]')
    args = parser.parse_args()

    est_path = args.run_dir / args.estimate
    stem = pathlib.Path(args.estimate).stem
    gt_path = args.run_dir / 'groundtruth.tum'
    if not gt_path.exists():
        print(f'no ground truth in {args.run_dir}', file=sys.stderr)
        return 1

    gt = file_interface.read_tum_trajectory_file(str(gt_path))
    est = file_interface.read_tum_trajectory_file(str(est_path))
    gt, est = sync.associate_trajectories(gt, est, max_diff=args.max_diff)
    est.align(gt, correct_scale=False)  # SE(3) Umeyama alignment

    ate = metrics.APE(metrics.PoseRelation.translation_part)
    ate.process_data((gt, est))
    rpe = metrics.RPE(metrics.PoseRelation.translation_part, delta=1.0,
                      delta_unit=metrics.Unit.meters, all_pairs=True)
    rpe.process_data((gt, est))

    length = gt.path_length
    result = {
        'poses': est.num_poses,
        'path_length_m': round(length, 3),
        'ate_rmse_m': round(ate.get_statistic(metrics.StatisticsType.rmse), 4),
        'ate_max_m': round(ate.get_statistic(metrics.StatisticsType.max), 4),
        'rpe_1m_rmse_m': round(rpe.get_statistic(metrics.StatisticsType.rmse), 4),
    }
    result['ate_percent_of_path'] = round(100 * result['ate_rmse_m'] / max(length, 1e-9), 3)
    name = 'metrics.json' if stem == 'estimate' else f'metrics_{stem}.json'
    (args.run_dir / name).write_text(json.dumps(result, indent=2) + '\n')
    for k, v in result.items():
        print(f'{k:22s} {v}')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        from evo.tools import plot
        import matplotlib.pyplot as plt
        fig = plt.figure(figsize=(8, 8))
        ax = plot.prepare_axis(fig, plot.PlotMode.xy)
        plot.traj(ax, plot.PlotMode.xy, gt, '--', 'gray', 'ground truth')
        plot.traj_colormap(ax, est, ate.error, plot.PlotMode.xy,
                           min_map=0.0, max_map=float(max(ate.error)))
        ax.set_title(f"ATE RMSE {result['ate_rmse_m']} m over {result['path_length_m']} m")
        plot_name = 'trajectory_xy.png' if stem == 'estimate' else f'trajectory_xy_{stem}.png'
        fig.savefig(args.run_dir / plot_name, dpi=120)
        print(f"plot: {args.run_dir / plot_name}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
