"""Track-arclength Kalman filter.

State:    x = [s, ds]  (arclength m, arclength velocity m/s)
Process:  constant-velocity with Q on acceleration, OR (Phase 1) IMU-driven
          constant-acceleration when a tangential accelerometer reading is
          supplied to predict() as a control input.
Measurements:
    - wheel-encoder odom: linear z = ds
    - IMU gyro (in-curve): z = ds, derived from yaw_rate * curve_radius
    - RFID tag (cue/positioning) at known arclength: z = s
    - photoeye docking: z = s_dock (very low variance, treated as a sharp
      measurement)

IMU + wheel-encoder fusion (Phase 1):
    * accelerometer -> control input in predict(); the measured tangential
      acceleration drives the mean instead of assuming zero accel. This sharpens
      the estimate through speed changes (curve entry/exit, approach ramps).
    * gyro -> in-curve velocity measurement (update_gyro); on a curve of radius R
      the body yaw rate omega gives v = omega * R, an encoder-independent speed
      cue that is robust to wheel slip the encoder cannot see.

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
class GyroObservation:
    """IMU yaw-rate read on a known-radius curve section.

    On a curve of radius R the arclength speed relates to the body yaw rate by
    v = yaw_rate * R, giving an encoder-independent velocity measurement. On
    straight track (radius -> inf / None) the read carries no speed information
    and is ignored.
    """
    yaw_rate_rps: float
    curve_radius_m: Optional[float]   # track radius at current s; None on straight
    sigma_rps: float = 0.01           # gyro angular-rate noise (1-sigma)


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
    # Default IMU accelerometer 1-sigma, used when predict() gets an accel
    # reading but no per-sample sigma. Smaller than process_accel_sigma because
    # a measured acceleration is far more informative than "unknown accel".
    imu_accel_sigma: float = 0.1      # m/s^2

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

    def predict(self, dt: float,
                accel_mps2: Optional[float] = None,
                accel_sigma_mps2: Optional[float] = None) -> None:
        """Propagate the estimate by dt.

        accel_mps2: optional IMU tangential acceleration. When given, it is used
            as a control input (constant-acceleration model) instead of assuming
            zero acceleration with broad process noise. accel_sigma_mps2
            overrides the default IMU noise for this sample.
        """
        if dt <= 0:
            return
        F = np.array([[1.0, dt], [0.0, 1.0]])
        # Control matrix for an acceleration input.
        B = np.array([0.5 * dt ** 2, dt])
        if accel_mps2 is None:
            # Constant-velocity: the unknown acceleration is the process noise.
            sigma_a = self.process_accel_sigma
            self._x = F @ self._x
        else:
            # IMU-driven: the measured acceleration drives the mean; its
            # measurement uncertainty becomes the process noise.
            sigma_a = (self.imu_accel_sigma if accel_sigma_mps2 is None
                       else accel_sigma_mps2)
            self._x = F @ self._x + B * float(accel_mps2)
        # Q = B B^T sigma_a^2 = [[dt^4/4, dt^3/2],[dt^3/2, dt^2]] sigma_a^2
        Q = np.outer(B, B) * sigma_a ** 2
        self._P = F @ self._P @ F.T + Q

    def update_odom(self, measured_ds: float) -> None:
        """Use a wheel-encoder velocity measurement."""
        H = np.array([[0.0, 1.0]])
        R = np.array([[self.odom_sigma ** 2]])
        self._kf_update(np.array([measured_ds]), H, R)

    def update_gyro(self, obs: GyroObservation) -> None:
        """Fuse an in-curve IMU yaw-rate read as a velocity measurement.

        v = yaw_rate * R. No-op on straight track (radius None/inf/<=0), where
        the yaw rate carries no arclength-speed information.
        """
        R_m = obs.curve_radius_m
        if R_m is None or not np.isfinite(R_m) or R_m <= 0.0:
            return
        z = obs.yaw_rate_rps * R_m
        H = np.array([[0.0, 1.0]])
        R = np.array([[(obs.sigma_rps * R_m) ** 2]])
        self._kf_update(np.array([z]), H, R)

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
