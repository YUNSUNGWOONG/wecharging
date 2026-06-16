"""RGB-D charging-port feature front-end (Phase 3 sim integration).

The missing link between the Gazebo RGB-D camera and the pose estimator: find
the charging-port fiducial in the image and turn it into the 3D feature points
that port_pose.estimate_pose consumes.

Pipeline:  RGB + depth  ->  detect dark socket quad (4 corners)  ->  deproject
each corner with its depth + camera intrinsics  ->  3D points (camera frame).

The port face is a dark recessed socket on a pale body, so a simple intensity
threshold isolates it; the four extreme corners of that region are the
fiducial. numpy-only (no OpenCV), tested on synthetic RGB-D so it runs in CI;
on the real Gazebo camera it consumes /r0/rgbd/image + /depth_image.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .port_pose import ChargingPort, PoseFit, estimate_pose


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_fov(cls, width: int, height: int, hfov_rad: float) -> "CameraIntrinsics":
        fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        return cls(fx=fx, fy=fx, cx=width / 2.0, cy=height / 2.0)


def deproject(u: float, v: float, depth: float,
              intr: CameraIntrinsics) -> np.ndarray:
    """Pixel (u, v) + depth -> 3D point in the camera optical frame
    (x right, y down, z forward)."""
    return np.array([(u - intr.cx) * depth / intr.fx,
                     (v - intr.cy) * depth / intr.fy,
                     depth])


def detect_socket_corners(rgb: np.ndarray, dark_max: int = 60) -> np.ndarray:
    """Find the 4 corners (pixels) of the dark socket region.

    Returns a (4, 2) array of (u, v) ordered TL, TR, BR, BL. Raises if the dark
    region is too small to be a fiducial.
    """
    gray = rgb[:, :, :3].mean(axis=2)
    ys, xs = np.nonzero(gray <= dark_max)
    if xs.size < 4:
        raise ValueError("no socket-like dark region found")
    xs = xs.astype(float)
    ys = ys.astype(float)
    s, d = xs + ys, xs - ys
    tl = (xs[np.argmin(s)], ys[np.argmin(s)])
    br = (xs[np.argmax(s)], ys[np.argmax(s)])
    tr = (xs[np.argmax(d)], ys[np.argmax(d)])
    bl = (xs[np.argmin(d)], ys[np.argmin(d)])
    return np.array([tl, tr, br, bl])


def _depth_at(depth: np.ndarray, u: float, v: float) -> float:
    return float(depth[int(round(v)), int(round(u))])


def detect_port_points(rgb: np.ndarray, depth: np.ndarray,
                       intr: CameraIntrinsics, dark_max: int = 60) -> np.ndarray:
    """Detect the socket corners and deproject them to 3D camera-frame points."""
    corners = detect_socket_corners(rgb, dark_max=dark_max)
    return np.array([deproject(u, v, _depth_at(depth, u, v), intr)
                     for u, v in corners])


def detect_port_pose(rgb: np.ndarray, depth: np.ndarray,
                     intr: CameraIntrinsics,
                     port: ChargingPort | None = None,
                     dark_max: int = 60) -> PoseFit:
    """Full front-end: image -> 3D feature points -> port pose estimate.

    Note: the translation (approach distance, lateral offset that the servo
    uses) is recovered robustly; full orientation needs an asymmetric marker to
    resolve the square's 4-fold ambiguity (future refinement).
    """
    port = port or ChargingPort()
    observed = detect_port_points(rgb, depth, intr, dark_max=dark_max)
    return estimate_pose(port.model_points, observed)
