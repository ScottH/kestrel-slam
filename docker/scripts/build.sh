#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Builds the Kestrel workspace inside the dev container. Extra args are passed to colcon build,
# e.g.  docker compose run --rm build --packages-select kestrel_core
set -euo pipefail
cd /ws
colcon build \
  --symlink-install \
  --base-paths src/kestrel_slam \
  --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DKESTREL_WARNINGS_AS_ERRORS=ON \
               -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -G Ninja \
  "$@"
