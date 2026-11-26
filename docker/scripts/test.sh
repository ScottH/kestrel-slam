#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Builds, then runs every package's tests and prints a summary. Non-zero exit on any failure,
# so this is also what CI runs.
set -euo pipefail
"$(dirname "$0")/build.sh"
# colcon's setup scripts read unset variables, so relax `set -u` while sourcing.
set +u
source /ws/install/setup.bash
set -u
cd /ws
colcon test --base-paths src/kestrel_slam --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --verbose
