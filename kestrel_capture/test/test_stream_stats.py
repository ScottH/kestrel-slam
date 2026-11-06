# SPDX-License-Identifier: Apache-2.0
"""Unit tests for the capture monitor's bookkeeping (no camera or ROS graph required)."""

from collections import deque

from kestrel_capture.monitor import StreamStats


def test_gap_and_reorder_detection():
    s = StreamStats('imu', expected_hz=200.0)
    t = 100.0
    for _ in range(10):
        s.add(t)
        t += 0.005
    s.add(t + 0.05)        # 50 ms gap → ~9 missing samples
    s.add(t + 0.04)        # stamp goes backwards
    summary = s.summary()
    assert summary['messages'] == 12
    assert summary['dropped_gaps'] == 1
    assert summary['reordered_stamps'] == 1
    assert abs(summary['max_gap_s'] - 0.055) < 1e-9


def test_rate_unknown_with_single_sample():
    s = StreamStats('left_ir', expected_hz=30.0)
    s.add(1.0)
    assert s.rate_hz() == 0.0


def test_stereo_matching_logic():
    # Re-implements CaptureMonitor.match_stereo's contract on plain deques: identical stamps pair
    # up regardless of arrival order, and nothing is left pending.
    from kestrel_capture.monitor import CaptureMonitor

    class Fake:
        stereo_pairs = 0
        stereo_unmatched = 0

    fake = Fake()
    left, right = deque(maxlen=60), deque(maxlen=60)
    CaptureMonitor.match_stereo(fake, 1.000, left, right)   # left arrives first
    CaptureMonitor.match_stereo(fake, 1.000, right, left)   # its partner
    CaptureMonitor.match_stereo(fake, 1.033, right, left)   # right arrives first this time
    CaptureMonitor.match_stereo(fake, 1.033, left, right)
    assert fake.stereo_pairs == 2
    assert not left and not right
