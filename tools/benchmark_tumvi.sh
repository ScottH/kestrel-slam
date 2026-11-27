#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Runs Kestrel VIO on every TUM-VI room sequence found under /data/datasets/tumvi and prints a
# Markdown results table (paste into docs/benchmarks.md). Run inside the dev container:
#
#   export KESTREL_DATA_DIR=~/code/kestrel_slam/data     # where the datasets live
#   docker compose run --rm build
#   docker compose run --rm shell /ws/src/kestrel_slam/tools/benchmark_tumvi.sh [rooms...]
#
# Expects /data/datasets/tumvi/room<N>_512_16/mav0/... (TUM-VI "euroc" 512x512 export).
# Results go to /data/results/tumvi_room<N>/ (trajectories, metrics.json, plot).
set -eo pipefail
source /ws/install/setup.bash
CONFIG=${CONFIG:-/ws/src/kestrel_slam/kestrel_core/config/tumvi_512.yaml}
RUN_VIO=/ws/install/kestrel_core/lib/kestrel_core/run_vio
EVALUATE="/opt/evo/bin/python /ws/src/kestrel_slam/tools/evaluate.py"
rooms=("$@")
[[ ${#rooms[@]} -eq 0 ]] && rooms=(1 2 3 4 5 6)

rows=()
for r in "${rooms[@]}"; do
  data=/data/datasets/tumvi/room${r}_512_16
  [[ -d "$data/mav0" ]] || { echo "room$r: not found, skipped" >&2; continue; }
  out=/data/results/tumvi_room$r
  log=$("$RUN_VIO" "$data" "$CONFIG" "$out" 2>&1 | grep -E "^frames|time / frame|solve /")
  $EVALUATE "$out" --plot > /dev/null
  json="$out/metrics.json"
  get() { python3 -c "import json,sys; print(json.load(open('$json'))['$1'])"; }
  resets=$(echo "$log" | sed -nE 's/.*resets ([0-9]+).*/\1/p')
  frame_ms=$(echo "$log" | sed -nE 's/^time \/ frame \[ms\] +median ([0-9.]+).*/\1/p')
  rows+=("| room$r | $(get path_length_m) | $(get ate_rmse_m) | $(get rpe_1m_rmse_m) | $resets | $frame_ms |")
  echo "room$r done" >&2
done

echo "| Sequence | Path [m] | ATE RMSE [m] | RPE 1 m [m] | Resets | ms / frame (median) |"
echo "|---|---|---|---|---|---|"
printf '%s\n' "${rows[@]}"
