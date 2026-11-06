# SPDX-License-Identifier: Apache-2.0
"""Live health monitor for Gemini 335Lg recording sessions.

While a bag is being recorded this node watches the sensor streams and tells you, in plain
language, whether the data is usable for SLAM:

* stream rates (left/right IR, depth, IMU) versus what was requested,
* hardware stereo sync (left and right IR frames must carry identical timestamps),
* timestamp gaps / reordering (dropped frames, USB bandwidth problems),
* IMU sanity (gravity magnitude, gaps),
* image quality (brightness, saturation, number of trackable corners),
* a guided *static start*: SLAM initializes best if the camera is held still for a few seconds.

At shutdown it writes a YAML summary next to the bag so every recording documents itself.

Why a separate monitor instead of `ros2 topic hz`? Because a bad recording is usually discovered
hours later on the desk. A 10-second glance at this table while recording avoids that.
"""

from __future__ import annotations

import math
import os
import time
from collections import deque
from dataclasses import dataclass, field

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

try:  # OpenCV is only used for the (optional) image-quality statistics.
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - the dev image ships both
    cv2 = None
    np = None


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


@dataclass
class StreamStats:
    """Rolling statistics for one topic."""

    name: str
    expected_hz: float
    window_s: float = 3.0
    count: int = 0
    reordered: int = 0
    max_gap_s: float = 0.0
    gaps_over_threshold: int = 0
    first_stamp: float | None = None
    last_stamp: float | None = None
    arrivals: deque = field(default_factory=deque)

    def add(self, stamp_s: float) -> None:
        now = time.monotonic()
        self.count += 1
        self.arrivals.append(now)
        while self.arrivals and now - self.arrivals[0] > self.window_s:
            self.arrivals.popleft()
        if self.first_stamp is None:
            self.first_stamp = stamp_s
        if self.last_stamp is not None:
            dt = stamp_s - self.last_stamp
            if dt <= 0.0:
                self.reordered += 1
            else:
                self.max_gap_s = max(self.max_gap_s, dt)
                # A gap longer than ~2.5 nominal periods means at least one dropped sample.
                if self.expected_hz > 0 and dt > 2.5 / self.expected_hz:
                    self.gaps_over_threshold += 1
        self.last_stamp = stamp_s

    def rate_hz(self) -> float:
        if len(self.arrivals) < 2:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0 else 0.0

    def ok(self) -> bool:
        return self.expected_hz <= 0 or self.rate_hz() >= 0.9 * self.expected_hz

    def summary(self) -> dict:
        duration = (
            (self.last_stamp - self.first_stamp)
            if self.first_stamp is not None and self.last_stamp is not None
            else 0.0
        )
        return {
            'messages': self.count,
            'duration_s': round(duration, 3),
            'mean_rate_hz': round(self.count / duration, 2) if duration > 0 else 0.0,
            'expected_hz': self.expected_hz,
            'max_gap_s': round(self.max_gap_s, 4),
            'dropped_gaps': self.gaps_over_threshold,
            'reordered_stamps': self.reordered,
        }


class CaptureMonitor(Node):
    """Subscribes to the camera topics and reports recording health."""

    def __init__(self) -> None:
        super().__init__('kestrel_capture_monitor')
        p = self.declare_parameter
        self.camera = p('camera_name', 'camera').value
        self.fps = float(p('fps', 30.0).value)
        self.imu_hz = float(p('imu_rate_hz', 200.0).value)
        self.record_depth = bool(p('record_depth', True).value)
        self.laser_mode = p('laser_mode', 'off').value
        self.still_seconds = float(p('static_start_s', 3.0).value)
        self.report_period = float(p('report_period_s', 2.0).value)
        self.summary_path = p('summary_path', '').value
        self.session_info = {
            'session': p('session', '').value,
            'scenario': p('scenario', '').value,
            'notes': p('notes', '').value,
            'bag': p('bag_path', '').value,
        }

        # In interleave mode only every other IR frame is laser-free; rates are unchanged on the
        # topic, but users should know that tracking will effectively use half of them.
        c = self.camera
        self.left = StreamStats('left_ir', self.fps)
        self.right = StreamStats('right_ir', self.fps)
        self.depth = StreamStats('depth', self.fps if self.record_depth else 0.0)
        self.imu = StreamStats('imu', self.imu_hz)

        self.create_subscription(Image, f'/{c}/left_ir/image_raw', self.on_left, qos_profile_sensor_data)
        self.create_subscription(Image, f'/{c}/right_ir/image_raw', self.on_right, qos_profile_sensor_data)
        if self.record_depth:
            self.create_subscription(Image, f'/{c}/depth/image_raw', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Imu, f'/{c}/gyro_accel/sample', self.on_imu, qos_profile_sensor_data)
        self.events = self.create_publisher(String, '/kestrel_capture/events', 10)

        # Stereo sync bookkeeping: left stamps waiting for their right partner (and vice versa).
        self.pending_left: deque = deque(maxlen=60)
        self.pending_right: deque = deque(maxlen=60)
        self.stereo_pairs = 0
        self.stereo_unmatched = 0

        # IMU sanity + guided static start.
        self.acc_norms: deque = deque(maxlen=400)
        self.gyro_norms: deque = deque(maxlen=400)
        self.still_since: float | None = None
        self.static_done = False

        # Image quality (computed at ~1 Hz to keep CPU low).
        self.last_quality_t = 0.0
        self.quality = {'mean': float('nan'), 'saturated_pct': float('nan'), 'corners': -1}

        self.warnings: set[str] = set()
        self.start_wall = time.time()
        self.create_timer(self.report_period, self.report)
        self.get_logger().info(
            f'Monitoring /{c}/... — hold the camera STILL for {self.still_seconds:.0f} s, '
            'then start moving.'
        )

    # ------------------------------------------------------------------ callbacks
    def on_left(self, msg: Image) -> None:
        t = stamp_to_sec(msg.header.stamp)
        self.left.add(t)
        self.match_stereo(t, self.pending_left, self.pending_right)
        now = time.monotonic()
        if cv2 is not None and now - self.last_quality_t > 1.0:
            self.last_quality_t = now
            self.measure_quality(msg)

    def on_right(self, msg: Image) -> None:
        t = stamp_to_sec(msg.header.stamp)
        self.right.add(t)
        self.match_stereo(t, self.pending_right, self.pending_left)

    def on_depth(self, msg: Image) -> None:
        self.depth.add(stamp_to_sec(msg.header.stamp))

    def on_imu(self, msg: Imu) -> None:
        self.imu.add(stamp_to_sec(msg.header.stamp))
        a, w = msg.linear_acceleration, msg.angular_velocity
        acc = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        gyr = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)
        self.acc_norms.append(acc)
        self.gyro_norms.append(gyr)
        self.update_static_start(acc, gyr)

    # ------------------------------------------------------------------ checks
    def match_stereo(self, t: float, mine: deque, other: deque) -> None:
        """Pairs left/right frames by timestamp; the 335L triggers both sensors together."""
        for i, t_other in enumerate(other):
            if abs(t_other - t) < 1e-3:
                del other[i]
                self.stereo_pairs += 1
                return
        if len(mine) == mine.maxlen:
            self.stereo_unmatched += 1  # the oldest entry never found a partner
        mine.append(t)

    def update_static_start(self, acc: float, gyr: float) -> None:
        if self.static_done:
            return
        now = time.monotonic()
        # Still = barely rotating and accelerometer reading ≈ gravity only.
        if gyr < 0.05 and abs(acc - 9.81) < 0.3:
            if self.still_since is None:
                self.still_since = now
            elif now - self.still_since >= self.still_seconds:
                self.static_done = True
                self.events.publish(String(data='static_start_complete'))
                self.get_logger().info('✔ Static start recorded — start moving now.')
        else:
            self.still_since = None

    def measure_quality(self, msg: Image) -> None:
        if msg.encoding not in ('mono8', '8UC1'):
            return  # 16-bit IR modes: skip quality stats rather than guess the scaling
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width]
        small = cv2.resize(img, (msg.width // 2, msg.height // 2), interpolation=cv2.INTER_AREA)
        corners = cv2.goodFeaturesToTrack(small, maxCorners=500, qualityLevel=0.01, minDistance=8)
        self.quality = {
            'mean': float(img.mean()),
            'saturated_pct': float((img >= 250).mean() * 100.0),
            'corners': 0 if corners is None else int(len(corners)),
        }

    def collect_warnings(self) -> list[str]:
        w = []
        for s in (self.left, self.right, self.depth, self.imu):
            if s.expected_hz > 0 and s.count > 0 and not s.ok():
                w.append(f'{s.name} rate {s.rate_hz():.1f} Hz < expected {s.expected_hz:.0f} Hz')
            if s.expected_hz > 0 and s.count == 0 and time.time() - self.start_wall > 5:
                w.append(f'no {s.name} messages received')
        if self.stereo_unmatched > 0:
            w.append(f'{self.stereo_unmatched} IR frames without a stereo partner')
        if self.acc_norms:
            mean_acc = sum(self.acc_norms) / len(self.acc_norms)
            if self.static_done is False and abs(mean_acc - 9.81) > 1.5 and len(self.acc_norms) > 100:
                w.append(f'IMU |a| averages {mean_acc:.2f} m/s^2 — check IMU units/scaling')
        q = self.quality
        if q['corners'] >= 0 and q['corners'] < 80:
            w.append(f'only {q["corners"]} corners in view — low texture or too dark')
        if not math.isnan(q['saturated_pct']) and q['saturated_pct'] > 10:
            w.append(f'{q["saturated_pct"]:.0f}% of IR pixels saturated — sunlight or laser glare?')
        if not math.isnan(q['mean']) and q['mean'] < 25:
            w.append('IR image very dark — increase exposure or add light')
        return w

    # ------------------------------------------------------------------ reporting
    def report(self) -> None:
        elapsed = time.time() - self.start_wall
        rows = [
            f'{s.name:<8} {s.rate_hz():6.1f} Hz (want {s.expected_hz:4.0f})  '
            f'max gap {s.max_gap_s * 1e3:6.1f} ms  drops {s.gaps_over_threshold}'
            for s in (self.left, self.right, self.depth, self.imu)
            if s.expected_hz > 0
        ]
        q = self.quality
        rows.append(
            f'stereo   pairs {self.stereo_pairs}  unmatched {self.stereo_unmatched}   '
            f'image mean {q["mean"]:.0f}  sat {q["saturated_pct"]:.1f}%  corners {q["corners"]}'
        )
        if not self.static_done:
            rows.append('static   ⏳ hold still to record the static start ...')
        warnings = self.collect_warnings()
        self.warnings.update(warnings)
        status = 'OK' if not warnings else 'CHECK'
        text = f'[{elapsed:6.0f} s] recording — {status}\n  ' + '\n  '.join(rows)
        if warnings:
            text += '\n  ⚠ ' + '\n  ⚠ '.join(warnings)
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def write_summary(self) -> None:
        summary = {
            **self.session_info,
            'laser_mode': self.laser_mode,
            'recorded_at': time.strftime('%Y-%m-%dT%H:%M:%S%z', time.localtime(self.start_wall)),
            'wall_duration_s': round(time.time() - self.start_wall, 1),
            'static_start_complete': self.static_done,
            'streams': {s.name: s.summary() for s in (self.left, self.right, self.depth, self.imu)},
            'stereo': {'pairs': self.stereo_pairs, 'unmatched': self.stereo_unmatched},
            'warnings_seen': sorted(self.warnings),
        }
        text = yaml.safe_dump(summary, sort_keys=False)
        if self.summary_path:
            os.makedirs(os.path.dirname(self.summary_path) or '.', exist_ok=True)
            with open(self.summary_path, 'w', encoding='utf-8') as f:
                f.write('# Kestrel capture summary (written by kestrel_capture_monitor)\n')
                f.write(text)
        print('\n===== capture summary =====\n' + text, flush=True)


def main() -> None:
    rclpy.init()
    node = CaptureMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.write_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
