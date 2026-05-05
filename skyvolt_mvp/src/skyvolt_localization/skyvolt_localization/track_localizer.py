"""Track-arclength Kalman filter.

State:    x = [s, ds]  (arclength m, arclength velocity m/s)
Process:  constant-velocity with Q on acceleration
Measurements:
    - wheel-encoder odom: linear z = ds
    - RFID tag (cue/positioning) at known arclength: z = s
    - photoeye docking: z = s_dock (very low variance, treated as a sharp
      measurement)

Kept dependency-free except for numpy so it can run in CI without ROS.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class RfidObservation:
    """One RFID read against a pre-mapped tag."""
    tag_arclength_m: float
    sigma_m: float = 0.025  # paper: 50 mm dia. cone -> ~25 mm 1-sigma
    kind: str = "cue"        # "cue" | "positioning" | "docking"
    tag_id: str = ""


@dataclass
class PhotoeyeObservation:
    """Binary photoeye trigger at a docking-card arclength."""
    docking_arclength_m: float
    sigma_m: float = 0.001  # paper: focused beam, ~1 mm
    triggered: bool = True
    docking_card_id: str = ""


@dataclass
class TrackLocalizer:
    """Constant-velocity KF on (s, ds).

    Reset/init:
        loc = TrackLocalizer(initial_s=0.0, initial_ds=0.0)
    Cycle each tick:
        loc.predict(dt)
        loc.update_odom(measured_ds)
        if rfid: loc.update_rfid(rfid)
        if photo: loc.update_photoeye(photo)
        s_hat, ds_hat = loc.state
    """

    initial_s: float = 0.0
    initial_ds: float = 0.0
    # Process noise: how much we let velocity drift per second.
    # Tuned conservatively; refine with measured wheel data in Phase 1.
    process_accel_sigma: float = 0.5  # m/s^2
    odom_sigma: float = 0.05          # m/s, encoder noise

    _x: np.ndarray = field(init=False)
    _P: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self._x = np.array([self.initial_s, self.initial_ds], dtype=float)
        # Initial uncertainty: ±0.5 m on s, ±0.5 m/s on ds.
        self._P = np.diag([0.5**2, 0.5**2])

    # ------------------------------------------------------------------
    # Public API
    @property
    def state(self) -> tuple[float, float]:
        return float(self._x[0]), float(self._x[1])

    @property
    def covariance(self) -> np.ndarray:
        return self._P.copy()

    def predict(self, dt: float) -> None:
        if dt <= 0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        # Process noise from accel: q = [[dt^4/4, dt^3/2],[dt^3/2, dt^2]] * sigma_a^2
        sa2 = self.process_accel_sigma ** 2
        Q = np.array([
            [dt**4 / 4.0, dt**3 / 2.0],
            [dt**3 / 2.0, dt**2],
        ]) * sa2
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q

    def update_odom(self, measured_ds: float) -> None:
        """Use a wheel-encoder velocity measurement."""
        H = np.array([[0.0, 1.0]])
        R = np.array([[self.odom_sigma ** 2]])
        self._kf_update(np.array([measured_ds]), H, R)

    def update_rfid(self, obs: RfidObservation) -> None:
        """Use an RFID tag detection: arclength is known to the map."""
        H = np.array([[1.0, 0.0]])
        R = np.array([[obs.sigma_m ** 2]])
        self._kf_update(np.array([obs.tag_arclength_m]), H, R)

    def update_photoeye(self, obs: PhotoeyeObservation) -> None:
        """Photoeye: very low-variance arclength fix when triggered."""
        if not obs.triggered:
            return
        H = np.array([[1.0, 0.0]])
        R = np.array([[obs.sigma_m ** 2]])
        self._kf_update(np.array([obs.docking_arclength_m]), H, R)

    # ------------------------------------------------------------------
    # Internal
    def _kf_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        y = z - H @ self._x
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        I = np.eye(self._P.shape[0])
        # Joseph form for numerical stability.
        IKH = I - K @ H
        self._P = IKH @ self._P @ IKH.T + K @ R @ K.T
