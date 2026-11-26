#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Builds the (pure-Python) capture package if needed, then starts a recording session.
# Arguments are launch arguments, e.g.
#   docker compose run --rm capture session:=office_a scenario:=before_change
# Run with `--help` to list them.
set -eo pipefail
cd /ws
if ! ros2 pkg prefix kestrel_capture >/dev/null 2>&1; then
  echo "[capture] building kestrel_capture (first run only)..."
  colcon build --symlink-install --base-paths src/kestrel_slam --packages-select kestrel_capture \
    --event-handlers console_cohesion+ >/dev/null
fi
source /ws/install/setup.bash
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  exec ros2 launch kestrel_capture capture.launch.py --show-args
fi
exec ros2 launch kestrel_capture capture.launch.py "$@"
