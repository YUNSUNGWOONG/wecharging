"""RGB-D feature front-end tests (Phase 3 sim integration).

Synthesize an RGB-D frame of the dark socket at a known camera-frame pose, then
check the detector recovers the 3D feature points / approach distance that the
servo loop needs.
"""
import math

import numpy as np
import pytest

from skyvolt_vision.detector import (
    CameraIntrinsics, deproject, detect_socket_corners, detect_port_pose,
)
from skyvolt_vision.port_pose import approach_distance, lateral_offset
from skyvolt_vision.servo import VisualServo, simulate_insertion

W, H, HFOV = 320, 240, 1.2
INTR = CameraIntrinsics.from_fov(W, H, HFOV)
HALF = 0.03   # 60 mm socket


def _synth_rgbd(cx_m, cy_m, z_m, half=HALF):
    """Render a dark fronto-parallel socket quad centered at camera-frame
    (cx_m, cy_m, z_m). Returns (rgb, depth)."""
    rgb = np.full((H, W, 3), 200, dtype=np.uint8)        # pale background
    depth = np.full((H, W), 5.0, dtype=np.float32)        # far background
    us, vs = [], []
    for dx in (-half, half):
        for dy in (-half, half):
            X, Y, Z = cx_m + dx, cy_m + dy, z_m
            us.append(INTR.cx + INTR.fx * X / Z)
            vs.append(INTR.cy + INTR.fy * Y / Z)
    umin, umax = int(math.floor(min(us))), int(math.ceil(max(us)))
    vmin, vmax = int(math.floor(min(vs))), int(math.ceil(max(vs)))
    rgb[vmin:vmax + 1, umin:umax + 1] = 20                # dark socket
    depth[vmin:vmax + 1, umin:umax + 1] = z_m
    return rgb, depth


def test_intrinsics_from_fov():
    intr = CameraIntrinsics.from_fov(320, 240, 1.2)
    assert intr.cx == 160 and intr.cy == 120
    assert intr.fx == pytest.approx((320 / 2) / math.tan(0.6))
    assert intr.fx == intr.fy


def test_deproject_inverts_projection():
    X, Y, Z = 0.04, -0.02, 0.30
    u = INTR.cx + INTR.fx * X / Z
    v = INTR.cy + INTR.fy * Y / Z
    p = deproject(u, v, Z, INTR)
    assert np.allclose(p, [X, Y, Z], atol=1e-9)


def test_detects_four_socket_corners():
    rgb, _ = _synth_rgbd(0.0, 0.0, 0.30)
    corners = detect_socket_corners(rgb)
    assert corners.shape == (4, 2)
    # centered port -> corners straddle the image centre
    assert corners[:, 0].min() < INTR.cx < corners[:, 0].max()


def test_raises_without_a_dark_region():
    rgb = np.full((H, W, 3), 200, dtype=np.uint8)
    with pytest.raises(ValueError):
        detect_socket_corners(rgb)


def test_recovers_approach_and_lateral_from_image():
    rgb, depth = _synth_rgbd(0.03, -0.02, 0.30)
    fit = detect_port_pose(rgb, depth, INTR)
    assert approach_distance(fit.pose) == pytest.approx(0.30, abs=5e-3)
    assert lateral_offset(fit.pose) == pytest.approx(math.hypot(0.03, 0.02),
                                                     abs=5e-3)
    assert fit.rmse_m < 5e-3


def test_end_to_end_image_to_dock():
    """RGB-D image -> detect -> pose -> visual servo -> dock."""
    rgb, depth = _synth_rgbd(0.03, -0.02, 0.30)
    fit = detect_port_pose(rgb, depth, INTR)
    res = simulate_insertion(VisualServo(), fit.pose)
    assert res.docked
