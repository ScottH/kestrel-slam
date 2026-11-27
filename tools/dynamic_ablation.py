#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Ablation of dynamic-object handling (M4) on real sequences with an injected moving object.

    # inside the dev container:
    tools/dynamic_ablation.py                                  # default sequences
    tools/dynamic_ablation.py --sequences tumvi_room1 euroc_MH_01_easy

For each sequence of tools/benchmarks.yaml it runs run_vio four ways:

    clean      the original images, M4 on (the shipped default)
    off        a large object crossing the view (run_vio --inject-dynamic), M4 off (= M3)
    on         the same injected object, M4 on
    oracle     the same, plus the object's true pixels as external mask (perfect segmentation)

and writes a table of ATE (VIO and final SLAM), resets and M4 statistics to
<results>/dynamic_ablation/<date>_<commit>/report.md. The object (kestrel/eval/dynamic_overlay.hpp)
is a textured panel 1.5 m away, 40 % × 60 % of the image, crossing in 4 s every 10 s: the "large
mover" regime of theory ch. 07. Expectation: on ≈ clean, off clearly worse, oracle ≈ clean.
"""

import argparse
import datetime
import json
import os
import pathlib
import subprocess
import sys

import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import regression_report as rr  # noqa: E402  (also re-launches under the evo virtualenv)

VARIANTS = [
    # name, dynamic.enable, extra run_vio options
    ('clean', True, []),
    ('off', False, ['--inject-dynamic']),
    ('on', True, ['--inject-dynamic']),
    ('oracle', True, ['--inject-dynamic', '--oracle-mask']),
]
DEFAULT = ['tumvi_room1', 'tumvi_room2', 'tumvi_room4', 'euroc_MH_01_easy']


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--sequences', nargs='*', default=DEFAULT)
    args = parser.parse_args()

    registry = yaml.safe_load((rr.REPO / 'tools' / 'benchmarks.yaml').read_text())
    by_name = {s['name']: s for s in registry['sequences'] + registry.get('ci_sequences', [])}
    commit = os.environ.get('KESTREL_COMMIT') or rr.git('rev-parse', '--short', 'HEAD')
    now = datetime.datetime.now()
    out_root = rr.RESULTS.parent / 'dynamic_ablation' / f"{now:%Y%m%d_%H%M%S}_{commit}"

    rows = []
    for name in args.sequences:
        seq = by_name[name]
        if not (rr.DATA / 'datasets' / seq['dataset'] / 'mav0').is_dir():
            print(f'{name}: data not found, skipped', file=sys.stderr)
            continue
        for variant, enable, options in VARIANTS:
            out_dir = out_root / name / variant
            out_dir.mkdir(parents=True, exist_ok=True)
            # Same configuration as the benchmark, with only dynamic.enable changed.
            config = yaml.safe_load((rr.REPO / seq['config']).read_text()) or {}
            config.setdefault('dynamic', {})['enable'] = enable
            config_file = out_dir / 'ablation_config.yaml'
            config_file.write_text(yaml.safe_dump(config, sort_keys=False))
            print(f'{name} / {variant}: running ...', file=sys.stderr, flush=True)
            cmd = [rr.RUN_VIO, str(rr.DATA / 'datasets' / seq['dataset']), str(config_file),
                   str(out_dir), *seq.get('options', []), *options]
            with open(out_dir / 'run.log', 'w') as log:
                subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
            row = rr.score(seq, out_dir)
            stats_file = out_dir / 'run_stats.json'
            stats = json.loads(stats_file.read_text()) if stats_file.exists() else {}
            row.update(variant=variant, outliers=stats.get('outliers'),
                       removed_factors=stats.get('removed_factors'),
                       injected=stats.get('injected_frames'),
                       blocked=stats.get('heat_blocked_fraction_mean'))
            rows.append(row)
            print(f"  ATE VIO {row.get('ate_vio_m')}  SLAM {row.get('ate_slam_final_m')}  "
                  f"resets {row.get('resets')}", file=sys.stderr, flush=True)

    def cell(v, f='{:.3f}'):
        return '–' if v is None else f.format(v)

    lines = [f'# Dynamic-object ablation — {now:%Y-%m-%d %H:%M}', '',
             f'Commit `{commit}`. Injected object: textured panel at 1.5 m, 40 % × 60 % of the '
             'image, crossing in 4 s every 10 s (from t = 5 s).', '',
             '| Sequence | Variant | ATE VIO [m] | ATE SLAM final [m] | Resets | Outliers | '
             'Factors removed | Heat-blocked image [%] | Loops | Wrong loops |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        if r['status'] != 'ok':
            lines.append(f"| {r['name']} | {r['variant']} | failed | | | | | | | |")
            continue
        lines.append(
            f"| {r['name']} | {r['variant']} | {cell(r.get('ate_vio_m'))} | "
            f"{cell(r.get('ate_slam_final_m'))} | {r.get('resets')} | "
            f"{cell(r.get('outliers'), '{}')} | {cell(r.get('removed_factors'), '{}')} | "
            f"{cell(r.get('blocked') and 100 * r['blocked'], '{:.1f}')} | "
            f"{r.get('loops')} | {r.get('wrong_loops')} |")
    lines += ['', '*clean*: original images, M4 on. *off*: injected object, M4 off (= M3). '
              '*on*: injected object, M4 on. *oracle*: injected object plus its true mask as '
              'external mask (perfect segmentation).']
    text = '\n'.join(lines) + '\n'
    out_root.mkdir(parents=True, exist_ok=True)
    (out_root / 'report.md').write_text(text)
    (out_root / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(text)
    print(f'report: {out_root / "report.md"}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
