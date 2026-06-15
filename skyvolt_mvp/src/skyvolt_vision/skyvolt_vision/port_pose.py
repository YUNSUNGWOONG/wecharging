"""Charging-port 6-DoF pose estimation (Phase 3, paper 2: Z-Axis Charging).

To insert the plug, the robot must know where the vehicle's charging port is.
An RGB-D sensor gives the 3D positions of a few known port features (fiducial
corners / socket landmarks). Given the same features' positions in the port's
own frame (the model), the port pose = the rigid transform that best maps model
points onto the observed points.

This module is the tested IP: a closed-form least-squares rigid-fit (the Kabsch
/ Umeyama algorithm) with a reflection guard, returning the pose and a fit
residual. Perception front-end (detecting the features in an image) and the
Gazebo RGB-D sensor are separate; this core runs pure-Python + numpy in CI.

The estimated pose then drives the lead-screw manipulator (skyvolt_manipulator):
the approach distance becomes the telescope reach, the lateral offset is the
alignment error a visual-servo loop must drive to zero.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Pose6DoF:
    """Rigid pose: observed = R @ model + t (camera/plug frame)."""
    R: np.ndarray   # 3x3 rotation
    t: np.ndarray   # 3, translation (m)

    def apply(self, points: np.ndarray) -> np.ndarray:
        """Map model-frame points (N x 3) into the observed frame."""
        return np.asarray(points) @ self.R.T + self.t

    @property
    def xyz(self) -> tuple[float, float, float]:
        return tuple(float(v) for v in self.t)

    @property
    def rpy(self) -> tuple[float, float, float]:
        """Roll, pitch, yaw (rad), ZYX convention."""
        R = self.R
        yaw = math.atan2(R[1, 0], R[0, 0])
        pitch = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
        roll = math.atan2(R[2, 1], R[2, 2])
        return roll, pitch, yaw


@dataclass
class PoseFit:
    pose: Pose6DoF
    rmse_m: float        # RMS residual of the fit (m)
    n_points: int


def estimate_pose(model_points: np.ndarray,
                  observed_points: np.ndarray) -> PoseFit:
    """Best-fit rigid transform mapping model points onto observed points.

    Kabsch/Umeyama: needs >= 3 non-degenerate correspondences. Returns the pose
    and the RMS residual (a small residual means a confident detection).
    """
    M = np.asarray(model_points, dtype=float)
    O = np.asarray(observed_points, dtype=float)
    if M.shape != O.shape or M.ndim != 2 or M.shape[1] != 3:
        raise ValueError("model/observed must be matching (N, 3) arrays")
    if M.shape[0] < 3:
        raise ValueError("need at least 3 point correspondences")

    cm = M.mean(axis=0)
    co = O.mean(axis=0)
    H = (M - cm).T @ (O - co)
    U, _, Vt = np.linalg.svd(H)
    # Reflection guard: force a proper rotation (det = +1).
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = co - R @ cm

    pose = Pose6DoF(R=R, t=t)
    resid = O - pose.apply(M)
    rmse = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))
    return PoseFit(pose=pose, rmse_m=rmse, n_points=M.shape[0])


@dataclass
class ChargingPort:
    """Known charging-port feature geometry in the port frame.

    Default: four fiducial corners of a square socket face (side `size_m`)
    centered on the port origin, lying in the port's x-y plane. The +z axis is
    the insertion (approach) axis.
    """
    size_m: float = 0.06   # 60 mm fiducial square

    @property
    def model_points(self) -> np.ndarray:
        h = self.size_m / 2.0
        return np.array([
            [-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0],
        ])

    def detect(self, observed_points: np.ndarray) -> PoseFit:
        """Estimate the port pose from observed feature positions."""
        return estimate_pose(self.model_points, observed_points)


def approach_distance(pose: Pose6DoF) -> float:
    """Distance to the port along the insertion (approach) axis: the telescope
    reach the manipulator must extend (port-origin z in the plug frame)."""
    return float(pose.t[2])


def lateral_offset(pose: Pose6DoF) -> float:
    """In-plane misalignment to the port axis — the error a visual-servo loop
    drives to zero before insertion."""
    return float(math.hypot(pose.t[0], pose.t[1]))
