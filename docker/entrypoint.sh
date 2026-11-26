#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# Container entrypoint: sources the ROS 2 environment, the Orbbec driver and (if built) the Kestrel
# workspace overlay, then runs the requested command.
#
# Environment variables
#   KESTREL_OVERLAY     explicit overlay to source (default: /ws/install if it exists).
#   KESTREL_GRANT_USB   USB vendor id(s) (e.g. "2bc5" = Orbbec). When the container starts as root,
#                       read/write access is granted on matching /dev/bus/usb nodes — exactly what
#                       Orbbec's udev rule would do on the host — so nothing needs installing on the
#                       host. Requires the service to be privileged with /dev mounted.
#   KESTREL_RUN_AS      user to drop to after the root-only setup above (e.g. "kestrel"), so that
#                       files written to bind mounts (rosbags!) are owned by you, not root.
set -eo pipefail

# --- root-only setup ---------------------------------------------------------------------------
if [[ "$(id -u)" == "0" && -n "${KESTREL_GRANT_USB:-}" ]]; then
  for dev in /sys/bus/usb/devices/*; do
    [[ -f "${dev}/idVendor" && -f "${dev}/busnum" && -f "${dev}/devnum" ]] || continue
    vendor="$(cat "${dev}/idVendor")"
    for wanted in ${KESTREL_GRANT_USB}; do
      if [[ "${vendor}" == "${wanted}" ]]; then
        node="$(printf '/dev/bus/usb/%03d/%03d' "$(cat "${dev}/busnum")" "$(cat "${dev}/devnum")")"
        if [[ -e "${node}" ]]; then
          chmod a+rw "${node}"
          echo "[entrypoint] granted access to USB device ${vendor}:$(cat "${dev}/idProduct") at ${node}"
        fi
      fi
    done
  done
fi

if [[ "$(id -u)" == "0" && -n "${KESTREL_RUN_AS:-}" ]]; then
  # Re-run this script as the unprivileged user (the root-only block is skipped the second time).
  run_as="${KESTREL_RUN_AS}"
  unset KESTREL_RUN_AS
  export HOME="$(getent passwd "${run_as}" | cut -d: -f6)"
  exec setpriv --reuid="$(id -u "${run_as}")" --regid="$(id -g "${run_as}")" --init-groups \
    "$0" "$@"
fi

# --- ROS environment -----------------------------------------------------------------------------
# ROS setup scripts reference unset variables, so they must be sourced without `set -u`.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /opt/orbbec_ws/install/setup.bash ]]; then
  source /opt/orbbec_ws/install/setup.bash
fi
overlay="${KESTREL_OVERLAY:-/ws/install}"
if [[ -f "${overlay}/setup.bash" ]]; then
  source "${overlay}/setup.bash"
fi

if [[ $# -eq 0 ]]; then
  set -- bash
fi
exec "$@"
