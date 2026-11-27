#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Score a run_vio output folder against its ground truth with evo.

    tools/evaluate.py <run_dir> [--estimate FILE] [--plot]

Reads <run_dir>/<estimate> (default estimate.tum) and <run_dir>/groundtruth.tum and reports:

* ATE (absolute trajectory error): RMSE of position error after aligning the estimate to the
  ground truth with a rigid SE(3) transform (Umeyama, no scale). Stereo-inertial systems know
  metric scale, so scale must NOT be corrected; see docs/theory/11_evaluation.md.
* RPE (relative pose error) over 1 m segments: drift per metre, independent of where errors
  happened.

Estimates written by run_vio: estimate.tum (odometry), estimate_map.tum (online, loop-corrected),
keyframes_map.tum (final pose graph). Results are printed and saved to metrics[_<stem>].json.
Also importable: `evaluate(run_dir, estimate)` returns the metrics dict (used by
tools/regression_report.py). Runs inside the dev container, where evo is installed.
"""

import argparse
import json
import os
import pathlib
import sys

# evo lives in its own virtualenv in the dev image; re-launch under it when run as a script with
# the system Python (imports from regression_report.py already run under it).
try:
    import evo  # noqa: F401
except ImportError:
    if (__name__ == '__main__' and os.path.exists('/opt/evo/bin/python')
            and sys.executable != '/opt/evo/bin/python'):
        os.execv('/opt/evo/bin/python', ['/opt/evo/bin/python', *sys.argv])
    raise

from evo.core import metrics, sync
from evo.tools import file_interface


def evaluate(run_dir, estimate='estimate.tum', plot=False, max_diff=0.01):
    """ATE/RPE of run_dir/estimate vs. run_dir/groundtruth.tum; None if either file is missing."""
    run_dir = pathlib.Path(run_dir)
    est_path, gt_path = run_dir / estimate, run_dir / 'groundtruth.tum'
    if not est_path.exists() or not gt_path.exists() or est_path.stat().st_size == 0:
        return None
    gt = file_interface.read_tum_trajectory_file(str(gt_path))
    est = file_interface.read_tum_trajectory_file(str(est_path))
    gt, est = sync.associate_trajectories(gt, est, max_diff=max_diff)
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

    stem = pathlib.Path(estimate).stem
    name = 'metrics.json' if stem == 'estimate' else f'metrics_{stem}.json'
    (run_dir / name).write_text(json.dumps(result, indent=2) + '\n')

    if plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from evo.tools import plot as evo_plot
        fig = plt.figure(figsize=(8, 8))
        ax = evo_plot.prepare_axis(fig, evo_plot.PlotMode.xy)
        evo_plot.traj(ax, evo_plot.PlotMode.xy, gt, '--', 'gray', 'ground truth')
        evo_plot.traj_colormap(ax, est, ate.error, evo_plot.PlotMode.xy,
                               min_map=0.0, max_map=float(max(ate.error)))
        ax.set_title(f"{estimate}: ATE RMSE {result['ate_rmse_m']} m over "
                     f"{result['path_length_m']} m")
        plot_name = 'trajectory_xy.png' if stem == 'estimate' else f'trajectory_xy_{stem}.png'
        fig.savefig(run_dir / plot_name, dpi=120)
        plt.close(fig)
    return result


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
    result = evaluate(args.run_dir, args.estimate, args.plot, args.max_diff)
    if result is None:
        print(f'missing {args.estimate} or groundtruth.tum in {args.run_dir}', file=sys.stderr)
        return 1
    for k, v in result.items():
        print(f'{k:22s} {v}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
