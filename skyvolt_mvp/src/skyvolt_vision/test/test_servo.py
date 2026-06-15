"""Closed-loop visual-servoing tests (Phase 3)."""
import math

import numpy as np
import pytest

from skyvolt_vision.port_pose import Pose6DoF, ChargingPort
from skyvolt_vision.servo import (
    VisualServo, ServoPhase, simulate_insertion,
)


def _pose(x, y, z):
    return Pose6DoF(R=np.eye(3), t=np.array([x, y, z], dtype=float))


def test_servo_aligns_then_docks():
    """From a lateral offset, the servo converges and seats the plug."""
    servo = VisualServo()
    res = simulate_insertion(servo, _pose(0.03, -0.02, 0.30))
    assert res.docked
    assert res.phase == ServoPhase.DOCKED
    assert res.final_lateral_m <= servo.align_tol_m + 1e-6
    assert res.final_depth_m <= servo.seat_depth_m + 1e-6


def test_already_aligned_goes_straight_to_insert():
    servo = VisualServo()
    res = simulate_insertion(servo, _pose(0.0, 0.0, 0.25))
    assert res.docked


def test_phase_progression_is_forward_only():
    """ALIGN -> INSERT -> DOCKED, never backwards."""
    servo = VisualServo()
    pose = _pose(0.04, 0.01, 0.30)
    order = {ServoPhase.ALIGN: 0, ServoPhase.INSERT: 1, ServoPhase.DOCKED: 2}
    rank = 0
    from skyvolt_vision.servo import _advance_plant
    for _ in range(5000):
        cmd = servo.step(pose, 0.02)
        assert order[cmd.phase] >= rank
        rank = order[cmd.phase]
        if servo.phase == ServoPhase.DOCKED:
            break
        pose = _advance_plant(pose, cmd, 0.02)
    assert servo.phase == ServoPhase.DOCKED


def test_lateral_command_is_bounded():
    servo = VisualServo(max_lateral_mps=0.05)
    cmd = servo.step(_pose(10.0, 10.0, 0.3), 0.02)   # huge offset
    assert abs(cmd.vx) <= 0.05 + 1e-9
    assert abs(cmd.vy) <= 0.05 + 1e-9


def test_alignment_error_decreases_monotonically():
    servo = VisualServo()
    from skyvolt_vision.servo import _advance_plant
    pose = _pose(0.05, 0.0, 0.30)
    prev = math.hypot(pose.t[0], pose.t[1])
    while servo.phase == ServoPhase.ALIGN:
        cmd = servo.step(pose, 0.02)
        if servo.phase != ServoPhase.ALIGN:
            break
        pose = _advance_plant(pose, cmd, 0.02)
        lat = math.hypot(pose.t[0], pose.t[1])
        assert lat <= prev + 1e-9
        prev = lat


def test_latch_fires_exactly_once_at_dock():
    servo = VisualServo()
    from skyvolt_vision.servo import _advance_plant
    pose = _pose(0.0, 0.0, 0.10)
    engages = 0
    for _ in range(2000):
        cmd = servo.step(pose, 0.02)
        engages += int(cmd.engage)
        if servo.phase == ServoPhase.DOCKED:
            break
        pose = _advance_plant(pose, cmd, 0.02)
    assert engages == 1


def test_faults_if_alignment_cannot_converge_in_time():
    """A tiny tick budget can't finish alignment -> FAULT (no spurious dock)."""
    servo = VisualServo(max_ticks=3)
    res = simulate_insertion(servo, _pose(0.20, 0.0, 0.30))
    assert not res.docked
    assert res.phase == ServoPhase.FAULT


def test_terminal_states_are_sticky():
    servo = VisualServo()
    simulate_insertion(servo, _pose(0.0, 0.0, 0.10))
    assert servo.phase == ServoPhase.DOCKED
    cmd = servo.step(_pose(0.5, 0.5, 0.5), 0.02)   # any later input
    assert cmd.phase == ServoPhase.DOCKED
    assert cmd.vx == 0.0 and cmd.reach_vel == 0.0


def test_end_to_end_vision_then_servo():
    """Detect the port from RGB-D points, then servo to a dock."""
    port = ChargingPort()
    # Port observed 28 cm ahead, 3 cm off-axis.
    t = np.array([0.03, 0.0, 0.28])
    observed = port.model_points + t
    fit = port.detect(observed)
    res = simulate_insertion(VisualServo(), fit.pose)
    assert res.docked
