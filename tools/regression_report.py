#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Benchmark + regression report for Kestrel: runs every registered sequence the same way and
summarizes accuracy, robustness and timing in one table.

    # inside the dev container (docker compose run --rm shell ...):
    tools/regression_report.py                       # run all sequences, compare to baseline
    tools/regression_report.py --sequences tumvi_room2 euroc_MH_01_easy
    tools/regression_report.py --skip-run            # re-score existing results only
    tools/regression_report.py --update-baseline     # accept the current results as baseline
    tools/regression_report.py --ci                  # quick CI check (short clip, run twice)

Inputs:
    tools/benchmarks.yaml            sequence registry + regression tolerances
    tools/regression_baseline.json   accepted reference results (committed)
Outputs (under $KESTREL_RESULTS, default /data/results/regression):
    <name>/                          run_vio output per sequence (trajectories, loops, plots)
    reports/<date>_<commit>/report.md, summary.json
    latest.md                        copy of the newest report

Per sequence: ATE of the odometry, of the online map-frame estimate and of the final pose graph;
RPE per metre; loop closures and how many of them ground truth says are wrong; resets; timing.
Summary rows give mean / median / max. The exit code is 1 if any sequence regressed against the
baseline beyond the tolerances (so CI or a pre-merge check can use it).

--ci selects the `ci_sequences` / `ci_tolerances` of the registry, the baseline
tools/regression_baseline_ci.json and the results folder <results>/ci. Each clip is run twice and
the trajectories must be byte-identical (offline mode is deterministic), so a CI failure is a
real change in behaviour, not noise. Missing clip data is an error in this mode, not a skip.
"""

import argparse
import datetime
import json
import os
import pathlib
import platform
import shutil
import statistics
import subprocess
import sys

import yaml

# evo lives in its own virtualenv in the dev image; re-launch under it if we were started with
# the system Python, so `tools/regression_report.py` works no matter how it is invoked.
try:
    import evo  # noqa: F401
except ImportError:
    if os.path.exists('/opt/evo/bin/python') and sys.executable != '/opt/evo/bin/python':
        os.execv('/opt/evo/bin/python', ['/opt/evo/bin/python', *sys.argv])
    raise

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from analyze_loops import audit  # noqa: E402
from evaluate import evaluate  # noqa: E402

REPO = pathlib.Path(__file__).resolve().parent.parent
DATA = pathlib.Path(os.environ.get('KESTREL_DATA', '/data'))
RESULTS = pathlib.Path(os.environ.get('KESTREL_RESULTS', str(DATA / 'results' / 'regression')))
RUN_VIO = os.environ.get('RUN_VIO', '/ws/install/kestrel_core/lib/kestrel_core/run_vio')
BASELINE = REPO / 'tools' / 'regression_baseline.json'
BASELINE_CI = REPO / 'tools' / 'regression_baseline_ci.json'
# Files that must be identical between two runs of the same input (determinism check).
DETERMINISTIC_FILES = ('estimate.tum', 'keyframes_map.tum', 'loops.txt')

# Columns of the summary table: (key, header, format, lower_is_better).
COLUMNS = [
    ('path_m', 'Path [m]', '{:.1f}', None),
    ('ate_vio_m', 'ATE VIO [m]', '{:.3f}', True),
    ('ate_slam_online_m', 'ATE SLAM online [m]', '{:.3f}', True),
    ('ate_slam_final_m', 'ATE SLAM final [m]', '{:.3f}', True),
    ('rpe_vio_m', 'RPE/m VIO [m]', '{:.3f}', True),
    ('loops', 'Loops', '{:d}', None),
    ('wrong_loops', 'Wrong loops', '{:d}', True),
    ('resets', 'Resets', '{:d}', True),
    ('frame_ms_median', 'ms/frame med', '{:.1f}', True),
    ('frame_ms_p95', 'ms/frame p95', '{:.1f}', True),
]


def git(*args):
    try:
        return subprocess.run(['git', '-C', str(REPO), *args], capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


def machine():
    cpu = platform.processor() or 'unknown'
    try:
        for line in open('/proc/cpuinfo'):
            if line.startswith('model name'):
                cpu = line.split(':', 1)[1].strip()
                break
    except OSError:
        pass
    return {'cpu': cpu, 'cores': os.cpu_count(), 'arch': platform.machine()}


def run_sequence(seq, out_dir):
    cmd = [RUN_VIO, str(DATA / 'datasets' / seq['dataset']), str(REPO / seq['config']),
           str(out_dir), *seq.get('options', [])]
    log = out_dir.parent / f"{seq['name']}.log"
    out_dir.mkdir(parents=True, exist_ok=True)
    with open(log, 'w') as f:
        return subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT).returncode


def score(seq, out_dir):
    """Collects all metrics for one sequence from its run_vio output folder."""
    row = {'name': seq['name'], 'group': seq.get('group', ''), 'status': 'ok'}
    stats_file = out_dir / 'run_stats.json'
    if not stats_file.exists():
        row['status'] = 'failed'
        return row
    stats = json.loads(stats_file.read_text())
    vio = evaluate(out_dir, 'estimate.tum')
    online = evaluate(out_dir, 'estimate_map.tum')
    final = evaluate(out_dir, 'keyframes_map.tum', plot=True)
    loops = audit(out_dir) or {}
    row.update({
        'path_m': vio['path_length_m'] if vio else None,
        'ate_vio_m': vio['ate_rmse_m'] if vio else None,
        'ate_slam_online_m': online['ate_rmse_m'] if online else None,
        'ate_slam_final_m': final['ate_rmse_m'] if final else None,
        'rpe_vio_m': vio['rpe_1m_rmse_m'] if vio else None,
        'loops': stats.get('loops', 0),
        'wrong_loops': loops.get('wrong', 0),
        'resets': stats.get('resets', 0),
        'frames': stats.get('frames'),
        'frame_ms_median': stats.get('frame_ms_median'),
        'frame_ms_p95': stats.get('frame_ms_p95'),
        'solve_ms_median': stats.get('solve_ms_median'),
        'loop_closure_async': stats.get('loop_closure_async', False),
    })
    return row


def fmt(value, pattern):
    if value is None:
        return '–'
    return pattern.format(int(value)) if pattern == '{:d}' else pattern.format(value)


def summary_rows(rows):
    out = []
    for label, fn in (('**mean**', statistics.mean), ('**median**', statistics.median),
                      ('**max**', max)):
        cells = []
        for key, _, pattern, _ in COLUMNS:
            values = [r[key] for r in rows if r.get(key) is not None]
            if not values:
                cells.append('–')
            elif pattern == '{:d}' and label != '**max**':
                cells.append(f'{fn(values):.1f}')
            else:
                cells.append(fmt(fn(values), pattern))
        out.append(f"| {label} | | {' | '.join(cells)} |")
    return out


def compare(rows, baseline, tol):
    """Regression check of each sequence against the baseline. Returns (lines, any_failed)."""
    lines, failed = [], False
    base = baseline.get('sequences', {}) if baseline else {}
    for r in rows:
        b = base.get(r['name'])
        if r['status'] != 'ok':
            lines.append(f"| {r['name']} | ❌ FAIL | run failed |")
            failed = True
            continue
        if not b:
            lines.append(f"| {r['name']} | ➕ new | no baseline yet |")
            continue
        reasons = []
        for key in ('ate_vio_m', 'ate_slam_final_m'):
            cur, ref = r.get(key), b.get(key)
            if cur is None or ref is None:
                continue
            allowed = max(ref * (1 + tol['ate_relative']), ref + tol['ate_absolute_m'])
            if cur > allowed:
                reasons.append(f'{key} {ref:.3f} → {cur:.3f} m (allowed ≤ {allowed:.3f})')
        for key, tkey in (('wrong_loops', 'wrong_loops'), ('resets', 'resets')):
            if r.get(key, 0) > b.get(key, 0) + tol[tkey]:
                reasons.append(f"{key} {b.get(key, 0)} → {r.get(key, 0)}")
        # Loop closure silently switched off (e.g. vocabulary missing) would only show up as a
        # small ATE change on a short clip, so the loop count has a floor too.
        if 'loops_min_fraction' in tol and b.get('loops'):
            floor = int(b['loops'] * tol['loops_min_fraction'])
            if r.get('loops', 0) < floor:
                reasons.append(f"loops {b['loops']} → {r.get('loops', 0)} (minimum {floor})")
        if r.get('deterministic') is False:
            reasons.append('two runs differ: ' + ', '.join(r.get('differs', [])))
        if reasons:
            failed = True
            lines.append(f"| {r['name']} | ❌ REGRESSION | {'; '.join(reasons)} |")
        else:
            d = []
            for key in ('ate_vio_m', 'ate_slam_final_m'):
                if r.get(key) is not None and b.get(key) is not None:
                    d.append(f"{key} {b[key]:.3f} → {r[key]:.3f}")
            lines.append(f"| {r['name']} | ✅ pass | {'; '.join(d)} |")
    return lines, failed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--sequences', nargs='*', help='subset of registry names')
    parser.add_argument('--skip-run', action='store_true', help='only re-score existing results')
    parser.add_argument('--update-baseline', action='store_true',
                        help=f'write the results as the new baseline ({BASELINE.relative_to(REPO)})')
    parser.add_argument('--no-fail', action='store_true', help='exit 0 even on regressions')
    parser.add_argument('--ci', action='store_true',
                        help='quick CI check: ci_sequences, CI baseline, determinism check')
    args = parser.parse_args()

    registry = yaml.safe_load((REPO / 'tools' / 'benchmarks.yaml').read_text())
    sequences = registry['ci_sequences' if args.ci else 'sequences']
    tolerances = registry['ci_tolerances' if args.ci else 'tolerances']
    baseline_file = BASELINE_CI if args.ci else BASELINE
    results = RESULTS / 'ci' if args.ci else RESULTS
    if args.sequences:
        unknown = set(args.sequences) - {s['name'] for s in sequences}
        if unknown:
            print(f'unknown sequences: {sorted(unknown)}', file=sys.stderr)
            return 2
        sequences = [s for s in sequences if s['name'] in args.sequences]

    # KESTREL_COMMIT overrides (git cannot resolve a worktree's .git link inside the container).
    commit = os.environ.get('KESTREL_COMMIT') or git('rev-parse', '--short', 'HEAD')
    dirty = not os.environ.get('KESTREL_COMMIT') and bool(git('status', '--porcelain'))
    rows, skipped = [], []
    for seq in sequences:
        out_dir = results / seq['name']
        if not (DATA / 'datasets' / seq['dataset'] / 'mav0').is_dir():
            if args.ci:
                print(f"{seq['name']}: data not found under {DATA / 'datasets'}", file=sys.stderr)
                return 2
            skipped.append(seq['name'])
            print(f"{seq['name']}: data not found, skipped", file=sys.stderr)
            continue
        repeat_dir = results / (seq['name'] + '_repeat')
        if not args.skip_run:
            print(f"{seq['name']}: running ...", file=sys.stderr, flush=True)
            for d in (out_dir, repeat_dir) if args.ci else (out_dir,):
                if d.exists():
                    shutil.rmtree(d)
                run_sequence(seq, d)
        rows.append(score(seq, out_dir))
        r = rows[-1]
        if args.ci and r['status'] == 'ok':
            r['differs'] = [f for f in DETERMINISTIC_FILES
                            if not (repeat_dir / f).exists()
                            or (out_dir / f).read_bytes() != (repeat_dir / f).read_bytes()]
            r['deterministic'] = not r['differs']
        print(f"{seq['name']}: {r['status']}  ATE VIO {r.get('ate_vio_m')}  "
              f"SLAM {r.get('ate_slam_final_m')}", file=sys.stderr, flush=True)

    baseline = json.loads(baseline_file.read_text()) if baseline_file.exists() else None
    check_lines, failed = compare(rows, baseline, tolerances)

    now = datetime.datetime.now()
    meta = {'date': now.isoformat(timespec='seconds'), 'commit': commit + ('+dirty' if dirty else ''),
            'machine': machine(), 'skipped': skipped}
    header = '| Sequence | Group | ' + ' | '.join(c[1] for c in COLUMNS) + ' |'
    sep = '|' + '---|' * (len(COLUMNS) + 2)
    table = [header, sep]
    for r in rows:
        cells = [fmt(r.get(k), p) for k, _, p, _ in COLUMNS] if r['status'] == 'ok' else \
            ['failed'] + ['–'] * (len(COLUMNS) - 1)
        table.append(f"| {r['name']} | {r['group']} | {' | '.join(cells)} |")
    table += summary_rows([r for r in rows if r['status'] == 'ok'])

    m = meta['machine']
    report = [
        f"# Kestrel {'CI ' if args.ci else ''}regression report — {now:%Y-%m-%d %H:%M}",
        '',
        f"Commit `{meta['commit']}` · {m['cpu']} ({m['cores']} cores, {m['arch']}) · "
        f"baseline: `{baseline['commit'] if baseline else 'none'}`",
        '',
        '## Results',
        '',
        *table,
        '',
        'ATE: RMSE after SE(3) alignment. *VIO* = odometry, *SLAM online* = loop-corrected pose as '
        'available at each instant, *SLAM final* = pose graph after the run. *Wrong loops*: off by '
        '> 0.2 m or > 5° vs. ground truth (`tools/analyze_loops.py`). Timing includes inline loop '
        'closure (offline mode); live operation runs it in a separate thread.',
        '',
        '## Regression check vs. baseline',
        '',
        '| Sequence | Result | Details |',
        '|---|---|---|',
        *check_lines,
        '',
        f"Tolerances: {tolerances}",
    ]
    if args.ci:
        report += ['', 'Determinism: ' + ', '.join(
            f"{r['name']} {'identical' if r.get('deterministic') else 'DIFFERENT'}"
            for r in rows if 'deterministic' in r)
            + ' (two runs, ' + ', '.join(DETERMINISTIC_FILES) + ')']
    if skipped:
        report += ['', f"Skipped (data not found): {', '.join(skipped)}"]
    text = '\n'.join(report) + '\n'

    out = results / 'reports' / f"{now:%Y%m%d_%H%M%S}_{commit}"
    out.mkdir(parents=True, exist_ok=True)
    (out / 'report.md').write_text(text)
    summary = {**meta, 'sequences': {r['name']: r for r in rows}}
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    (results / 'latest.md').write_text(text)
    print(text)
    print(f'report: {out / "report.md"}', file=sys.stderr)

    if args.update_baseline:
        if any(r['status'] != 'ok' or r.get('deterministic') is False for r in rows):
            print('not updating the baseline: a run failed or was not deterministic',
                  file=sys.stderr)
            return 1
        baseline_file.write_text(json.dumps(summary, indent=2) + '\n')
        print(f'baseline updated: {baseline_file}', file=sys.stderr)
        return 0
    return 1 if failed and not args.no_fail else 0


if __name__ == '__main__':
    sys.exit(main())
