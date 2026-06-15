"""Unit tests for charging-port 6-DoF pose estimation (Phase 3)."""
import math

import numpy as np
import pytest

from skyvolt_vision.port_pose import (
    Pose6DoF, ChargingPort, estimate_pose, approach_distance, lateral_offset,
)


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def test_recovers_known_transform_exactly():
    port = ChargingPort()
    R = _rot_z(0.4) @ _rot_x(-0.2)
    t = np.array([0.10, -0.05, 0.30])
    observed = port.model_points @ R.T + t
    fit = port.detect(observed)
    assert fit.rmse_m == pytest.approx(0.0, abs=1e-9)
    assert np.allclose(fit.pose.R, R, atol=1e-9)
    assert np.allclose(fit.pose.t, t, atol=1e-9)


def test_pure_translation():
    port = ChargingPort()
    t = np.array([0.0, 0.0, 0.25])
    fit = port.detect(port.model_points + t)
    assert np.allclose(fit.pose.R, np.eye(3), atol=1e-9)
    assert np.allclose(fit.pose.t, t, atol=1e-9)


def test_round_trip_apply_then_estimate():
    R = _rot_x(0.5)
    t = np.array([0.2, 0.1, 0.4])
    pose = Pose6DoF(R=R, t=t)
    model = ChargingPort(size_m=0.08).model_points
    fit = estimate_pose(model, pose.apply(model))
    assert np.allclose(fit.pose.apply(model), pose.apply(model), atol=1e-9)


def test_rmse_small_under_measurement_noise():
    port = ChargingPort()
    rng = np.random.default_rng(0)
    R = _rot_z(0.3)
    t = np.array([0.05, 0.02, 0.28])
    noisy = port.model_points @ R.T + t + rng.normal(0, 1e-3, size=(4, 3))
    fit = port.detect(noisy)
    assert fit.rmse_m < 3e-3                       # ~ the noise level
    assert np.allclose(fit.pose.t, t, atol=5e-3)   # pose still close


def test_result_is_a_proper_rotation_not_reflection():
    """The reflection guard must keep det(R) = +1 even for tricky data."""
    port = ChargingPort()
    R = _rot_x(math.pi) @ _rot_z(0.7)              # 180-deg flip + yaw
    observed = port.model_points @ R.T + np.array([0, 0, 0.3])
    fit = port.detect(observed)
    assert np.linalg.det(fit.pose.R) == pytest.approx(1.0, abs=1e-9)


def test_rpy_matches_known_yaw():
    pose = Pose6DoF(R=_rot_z(0.5), t=np.zeros(3))
    roll, pitch, yaw = pose.rpy
    assert yaw == pytest.approx(0.5, abs=1e-9)
    assert roll == pytest.approx(0.0, abs=1e-9)
    assert pitch == pytest.approx(0.0, abs=1e-9)


def test_approach_and_lateral_feed_the_manipulator():
    """Estimated pose -> reach distance + alignment error for the servo loop."""
    pose = Pose6DoF(R=np.eye(3), t=np.array([0.03, -0.04, 0.30]))
    assert approach_distance(pose) == pytest.approx(0.30)
    assert lateral_offset(pose) == pytest.approx(0.05)  # 3-4-5


def test_rejects_too_few_points():
    with pytest.raises(ValueError):
        estimate_pose(np.zeros((2, 3)), np.zeros((2, 3)))


def test_rejects_mismatched_shapes():
    with pytest.raises(ValueError):
        estimate_pose(np.zeros((4, 3)), np.zeros((5, 3)))
