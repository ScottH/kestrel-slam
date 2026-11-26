# syntax=docker/dockerfile:1.7
# SPDX-License-Identifier: Apache-2.0
#
# Kestrel SLAM container images. Everything (compilers, ROS 2, Eigen/OpenCV/Ceres, the Orbbec
# driver, evaluation tools) lives here, so the host only needs Docker.
#
# Stages
#   base     ROS 2 Jazzy + the libraries kestrel_core needs to build.
#   orbbec   builds the OrbbecSDK_ROS2 driver (pinned commit) into /opt/orbbec_ws.
#   dev      base + driver + developer tools (gdb, clang-format, evo, foxglove bridge, ...).
#            This is the image used by `docker compose run build|test|shell|capture`.
#   builder  dev + a release build of this repository (used to produce `runtime`).
#   runtime  slim image with only what is needed to run on the robot.
#
# Multi-arch: works for linux/amd64 (laptop/desktop) and linux/arm64 (Jetson, Raspberry Pi 5);
# the official ros images and the bundled Orbbec SDK both ship both architectures.

ARG ROS_DISTRO=jazzy

# ================================================================================================
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Build toolchain and the numerical / vision libraries used by kestrel_core.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build ccache git curl ca-certificates \
      libeigen3-dev libopencv-dev libceres-dev libyaml-cpp-dev libgtest-dev \
      python3-colcon-common-extensions python3-pip python3-numpy python3-yaml \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-tf2-eigen \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-visualization-msgs \
      ros-${ROS_DISTRO}-rosbag2-storage-mcap \
      ros-${ROS_DISTRO}-rosbag2-compression-zstd \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# ================================================================================================
FROM base AS orbbec
ARG ROS_DISTRO
# Pinned driver revision (v2-main as of 2026-10-08). Bump deliberately, then re-test capture.
ARG ORBBEC_REPO=https://github.com/orbbec/OrbbecSDK_ROS2.git
ARG ORBBEC_REF=c153462518ad674650bafa4464fda72c27ab797a

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libgflags-dev nlohmann-json3-dev libgoogle-glog-dev libssl-dev libdw-dev libusb-1.0-0-dev \
      ros-${ROS_DISTRO}-image-publisher \
      ros-${ROS_DISTRO}-camera-info-manager \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-diagnostic-msgs \
      ros-${ROS_DISTRO}-statistics-msgs \
      ros-${ROS_DISTRO}-backward-ros \
      ros-${ROS_DISTRO}-rclcpp-components \
      ros-${ROS_DISTRO}-tf2-sensor-msgs \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-robot-state-publisher \
      python3-psutil python3-tabulate

WORKDIR /opt/orbbec_ws/src
RUN git init -q OrbbecSDK_ROS2 && cd OrbbecSDK_ROS2 \
 && git remote add origin ${ORBBEC_REPO} \
 && git fetch -q --depth 1 origin ${ORBBEC_REF} \
 && git checkout -q FETCH_HEAD
WORKDIR /opt/orbbec_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --merge-install --install-base /opt/orbbec_ws/install \
      --packages-up-to orbbec_camera orbbec_description \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf build log src/OrbbecSDK_ROS2/.git

# ================================================================================================
FROM orbbec AS dev
ARG ROS_DISTRO
ARG USERNAME=kestrel
ARG USER_UID=1000
ARG USER_GID=1000

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      gdb valgrind clang-format clangd python3-matplotlib python3-scipy python3-venv \
      sudo less nano unzip wget usbutils \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-rosbag2

# evo (trajectory evaluation: ATE/RPE) lives in a venv so it never fights the system Python.
RUN python3 -m venv --system-site-packages /opt/evo \
 && /opt/evo/bin/pip install --no-cache-dir "evo==1.31.*" \
 && ln -s /opt/evo/bin/evo_ape /opt/evo/bin/evo_rpe /opt/evo/bin/evo_traj /usr/local/bin/

# A non-root user whose UID/GID match the host user, so files written into the bind-mounted
# source tree are owned by you, not by root. (The ubuntu base image ships a uid-1000 user.)
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
 && groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
 && usermod -aG plugdev,video,dialout ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /usr/local/bin/kestrel-entrypoint
# Pre-create the workspace dirs owned by the user: named volumes mounted here inherit ownership.
RUN mkdir -p /ws/src /ws/build /ws/install /ws/log /ws/.ccache /data \
 && chown -R ${USER_UID}:${USER_GID} /ws /data
ENV ROS_DISTRO=${ROS_DISTRO} \
    KESTREL_WS=/ws \
    CCACHE_DIR=/ws/.ccache
USER ${USERNAME}
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/kestrel-entrypoint"]
CMD ["bash"]

# ================================================================================================
FROM dev AS builder
USER root
COPY . /ws/src/kestrel_slam
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && . /opt/orbbec_ws/install/setup.sh \
 && colcon build --merge-install --install-base /opt/kestrel/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DKESTREL_BUILD_TESTS=OFF -DBUILD_TESTING=OFF

# ================================================================================================
FROM ros:${ROS_DISTRO}-ros-core AS runtime
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      libopencv-core406t64 libopencv-imgproc406t64 libopencv-features2d406t64 \
      libopencv-video406t64 libopencv-calib3d406t64 libceres4t64 libyaml-cpp0.8 \
      libgflags2.2 libgoogle-glog0v6t64 libusb-1.0-0 libdw1t64 \
      ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-message-filters ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-image-publisher ros-${ROS_DISTRO}-camera-info-manager \
      ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-backward-ros \
      ros-${ROS_DISTRO}-rclcpp-components ros-${ROS_DISTRO}-tf2-sensor-msgs \
      ros-${ROS_DISTRO}-statistics-msgs ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rosbag2-storage-mcap \
      ros-${ROS_DISTRO}-rosbag2-compression-zstd ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      python3-psutil python3-tabulate python3-yaml
COPY --from=orbbec /opt/orbbec_ws/install /opt/orbbec_ws/install
COPY --from=builder /opt/kestrel/install /opt/kestrel/install
COPY docker/entrypoint.sh /usr/local/bin/kestrel-entrypoint
ENV ROS_DISTRO=${ROS_DISTRO} KESTREL_OVERLAY=/opt/kestrel/install
ENTRYPOINT ["/usr/local/bin/kestrel-entrypoint"]
CMD ["ros2", "launch", "kestrel_capture", "capture.launch.py"]
