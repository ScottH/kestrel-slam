# SPDX-License-Identifier: Apache-2.0
"""Record a SLAM-ready rosbag from an Orbbec Gemini 335L / 335Lg.

What this launches
------------------
1. The Orbbec driver (``orbbec_camera/gemini_330_series.launch.py``) configured for visual-inertial
   SLAM: both IR cameras at the same resolution, hardware-synchronized IMU (gyro + accel fused
   into one ``sensor_msgs/Imu`` topic), optional depth, colour off by default.
2. ``ros2 bag record`` writing MCAP with zstd compression, recording only what SLAM needs.
3. ``kestrel_capture_monitor``, which prints stream health every few seconds and writes a summary
   YAML next to the bag.

Typical use (from the repository root, on the host)::

    docker compose run --rm capture session:=office scenario:=loop
    docker compose run --rm capture session:=office scenario:=after_change notes:="moved 2 shelves"
    docker compose run --rm capture --help          # list all arguments

Output: ``./data/bags/<session>_<scenario>_<YYYYmmdd_HHMMSS>/`` plus ``….capture.yaml``.

About the IR laser projector
----------------------------
The 335L projects an infrared dot pattern to help depth on blank walls. The dots are fixed to the
*camera*, not the world, so a feature tracker locks onto them and reports zero motion — poison for
SLAM. ``laser_mode`` therefore matters:

* ``off``  (default) — clean IR images at full rate; depth comes from passive stereo only.
* ``on``   — best depth, unusable for visual tracking. Only for depth/mapping experiments.
* ``interleave`` — the camera alternates laser-on / laser-off frames. Every frame is recorded and
  the per-frame metadata topics say which is which, so SLAM can track on laser-off frames and map
  with laser-on depth. Effective tracking rate is halved (e.g. 30 → 15 Hz).
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    ('session', 'capture', 'Name of the recording session (e.g. the place: "office", "warehouse").'),
    ('scenario', 'general',
     'What this sequence is for: static_start, loop, dynamic, before_change, after_change, '
     'outdoor, calib_camera, calib_imu, ... (see docs/hardware/capture_guide.md).'),
    ('notes', '', 'Free-text notes stored in the summary (what changed, lighting, ...).'),
    ('output_dir', '/data/bags', 'Directory (inside the container) for recordings; ./data on the host.'),
    ('resolution', '848x480', 'IR (and depth) resolution: 640x400, 848x480 or 1280x800.'),
    ('fps', '30', 'IR / depth frame rate (5, 10, 15, 30, 60; 1280x800 supports up to 30).'),
    ('imu_rate', '200hz', 'IMU rate for both gyro and accel: 50hz, 100hz, 200hz, 500hz, 1khz.'),
    ('laser_mode', 'off', 'IR projector: off | on | interleave (see module docstring).'),
    ('record_depth', 'true', 'Also record the on-board stereo depth stream.'),
    ('record_color', 'false', 'Also record the colour camera (large; not used by Kestrel yet).'),
    ('ir_format', 'Y8', 'IR pixel format (Y8 is 8-bit mono; ANY lets the driver choose).'),
    ('camera_name', 'camera', 'Topic/frame prefix used by the driver.'),
    ('serial_number', '', 'Pick a specific camera when several are connected.'),
    ('duration', '0', 'Stop automatically after this many seconds (0 = until Ctrl-C).'),
    ('static_start_s', '3', 'How long the monitor asks you to hold still at the start.'),
]


def _driver_arguments(ctx) -> dict:
    """Maps our high-level arguments onto the Orbbec driver's launch arguments."""
    get = lambda name: LaunchConfiguration(name).perform(ctx)  # noqa: E731
    width, height = get('resolution').lower().split('x')
    fps = get('fps')
    laser_mode = get('laser_mode')
    if laser_mode not in ('off', 'on', 'interleave'):
        raise RuntimeError(f"laser_mode must be off|on|interleave, got '{laser_mode}'")
    record_depth = get('record_depth').lower() == 'true'
    args = {
        'camera_name': get('camera_name'),
        # Stereo IR — the primary SLAM input. Both cameras must use identical settings.
        'enable_left_ir': 'true', 'enable_right_ir': 'true',
        'left_ir_width': width, 'left_ir_height': height, 'left_ir_fps': fps,
        'right_ir_width': width, 'right_ir_height': height, 'right_ir_fps': fps,
        'left_ir_format': get('ir_format'), 'right_ir_format': get('ir_format'),
        'enable_ir_auto_exposure': 'true',
        # Depth (used by the dense map, never by tracking).
        'enable_depth': 'true' if record_depth else 'false',
        'depth_width': width, 'depth_height': height, 'depth_fps': fps,
        'depth_registration': 'false',   # keep depth in the left-IR frame (no resampling)
        'enable_point_cloud': 'false',   # derived data; saves a lot of bandwidth
        'enable_color': get('record_color'),
        # IMU: one fused topic (<camera>/gyro_accel/sample), same rate for both sensors.
        'enable_accel': 'true', 'enable_gyro': 'true', 'enable_sync_output_accel_gyro': 'true',
        'accel_rate': get('imu_rate'), 'gyro_rate': get('imu_rate'),
        'accel_range': '4g', 'gyro_range': '1000dps',
        # Timing & geometry.
        'enable_frame_sync': 'true',
        'time_domain': 'global',         # device clock mapped to host time, drift-compensated
        'publish_tf': 'true', 'tf_publish_rate': '0.0',  # static TFs (camera↔IMU extrinsics)
        'enable_publish_extrinsic': 'true',
        # Laser projector (see module docstring).
        'enable_laser': 'false' if laser_mode == 'off' else 'true',
        'interleave_frame_enable': 'true' if laser_mode == 'interleave' else 'false',
        'interleave_ae_mode': 'laser',
        'interleave_skip_enable': 'false',  # record both kinds of frames; choose later
    }
    if get('serial_number'):
        args['serial_number'] = get('serial_number')
    return args


def _setup(ctx):
    get = lambda name: LaunchConfiguration(name).perform(ctx)  # noqa: E731
    camera = get('camera_name')
    stamp = time.strftime('%Y%m%d_%H%M%S')
    bag_name = f"{get('session')}_{get('scenario')}_{stamp}"
    output_dir = get('output_dir')
    os.makedirs(output_dir, exist_ok=True)
    bag_path = os.path.join(output_dir, bag_name)

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('orbbec_camera'), 'launch', 'gemini_330_series.launch.py')),
        launch_arguments=_driver_arguments(ctx).items(),
    )

    # Record by regular expression so that driver-version differences in auxiliary topic names
    # (metadata, imu_info, extrinsics) don't silently drop data. Exclude derived/compressed
    # image-transport variants and point clouds, which can be regenerated from the raw data.
    include = (
        rf'^/{camera}/(left_ir|right_ir|depth|color)/(image_raw|camera_info|metadata)$'
        rf'|^/{camera}/(gyro_accel|gyro|accel)/(sample|imu_info)$'
        rf'|^/{camera}/extrinsic/.*'
        r'|^/tf_static$|^/tf$|^/kestrel_capture/.*'
    )
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '--storage', 'mcap', '--storage-preset-profile', 'zstd_fast',
             '--output', bag_path,
             '--regex', include,
             '--exclude-regex', r'.*/(compressed|compressedDepth|theora|zstd)$'],
        output='screen',
        name='rosbag_record',
    )

    fps = float(get('fps'))
    imu_hz = float(get('imu_rate').lower().replace('khz', '000').replace('hz', ''))
    monitor = Node(
        package='kestrel_capture',
        executable='capture_monitor',
        name='kestrel_capture_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_name': camera,
            'fps': fps,
            'imu_rate_hz': imu_hz,
            'record_depth': get('record_depth').lower() == 'true',
            'laser_mode': get('laser_mode'),
            'static_start_s': float(get('static_start_s')),
            'summary_path': bag_path + '.capture.yaml',
            'bag_path': bag_path,
            'session': get('session'),
            'scenario': get('scenario'),
            'notes': get('notes'),
        }],
    )

    actions = [
        LogInfo(msg=f'[kestrel_capture] recording to {bag_path}  (Ctrl-C to stop)'),
        driver,
        monitor,
        recorder,
    ]
    duration = float(get('duration'))
    if duration > 0:
        actions.append(TimerAction(period=duration, actions=[
            LogInfo(msg=f'[kestrel_capture] {duration:.0f} s elapsed, stopping'),
            EmitEvent(event=Shutdown(reason='capture duration reached')),
        ]))
    return actions


def generate_launch_description() -> LaunchDescription:
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGUMENTS]
    return LaunchDescription(declared + [OpaqueFunction(function=_setup)])
