#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Builds the dev image (kestrel/dev:jazzy) with the host's network stack.
#
# Equivalent to `docker compose build`, but some Docker/Compose versions ignore the compose file's
# `build.network: host`, and on machines where Docker's default bridge network has no route to the
# internet (VPNs, some firewalls) package downloads inside the build then fail with DNS errors
# (e.g. curl exit code 6). `docker build --network host` always works. Run from the repo root:
#
#   docker/scripts/build-image.sh            # dev image
#   TARGET=runtime docker/scripts/build-image.sh
set -euo pipefail
cd "$(dirname "$0")/../.."
TARGET=${TARGET:-dev}
TAG=${TAG:-kestrel/${TARGET}:jazzy}
exec docker build --network host \
  --target "${TARGET}" -t "${TAG}" -f docker/Dockerfile \
  --build-arg ROS_DISTRO=jazzy \
  --build-arg USER_UID="${KESTREL_UID:-$(id -u)}" \
  --build-arg USER_GID="${KESTREL_GID:-$(id -g)}" \
  "$@" .
