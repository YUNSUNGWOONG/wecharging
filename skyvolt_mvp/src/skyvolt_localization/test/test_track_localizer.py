"""Unit tests for the track-arclength Kalman filter."""
import numpy as np
import pytest

from skyvolt_localization.track_localizer import (
    TrackLocalizer, RfidObservation, PhotoeyeObservation, GyroObservation,
)


def test_predict_advances_state_by_velocity():
    loc = TrackLocalizer(initial_s=0.0, initial_ds=1.5)
    loc.predict(1.0)
    s, ds = loc.state
    assert s == pytest.approx(1.5, abs=1e-6)
    assert ds == pytest.approx(1.5, abs=1e-6)


def test_odom_update_pulls_velocity_estimate():
    loc = TrackLocalizer(initial_s=0.0, initial_ds=0.0)
    loc.predict(0.1)
    for _ in range(50):
        loc.predict(0.02)
        loc.update_odom(1.0)
    _, ds = loc.state
    assert ds == pytest.approx(1.0, abs=0.05)


def test_rfid_update_anchors_arclength():
    loc = TrackLocalizer(initial_s=2.0, initial_ds=1.0)
    # Drift the prior way off, then anchor.
    for _ in range(5):
        loc.predict(0.1)
    loc.update_rfid(RfidObservation(tag_arclength_m=4.0, sigma_m=0.005))
    s, _ = loc.state
    # Tight anchor should pull the estimate near 4.0
    assert s == pytest.approx(4.0, abs=0.05)


def test_photoeye_anchor_dominates_when_triggered():
    loc = TrackLocalizer(initial_s=0.0, initial_ds=0.0)
    loc.update_photoeye(PhotoeyeObservation(
        docking_arclength_m=10.123, sigma_m=0.0005, triggered=True,
    ))
    s, _ = loc.state
    assert s == pytest.approx(10.123, abs=0.01)


def test_covariance_remains_positive_definite():
    loc = TrackLocalizer()
    rng = np.random.default_rng(42)
    for _ in range(200):
        loc.predict(0.02)
        loc.update_odom(rng.normal(1.0, 0.05))
        if rng.random() < 0.05:
            loc.update_rfid(RfidObservation(
                tag_arclength_m=loc.state[0] + rng.normal(0, 0.02)))
    eigs = np.linalg.eigvalsh(loc.covariance)
    assert (eigs > 0).all()


# ---------------------------------------------------------------------------
# Phase 1: IMU + wheel-encoder fusion
def test_predict_without_accel_is_unchanged():
    """Backward compat: predict(dt) must still be pure constant-velocity."""
    loc = TrackLocalizer(initial_s=1.0, initial_ds=2.0)
    loc.predict(0.5)
    s, ds = loc.state
    assert s == pytest.approx(1.0 + 2.0 * 0.5, abs=1e-9)
    assert ds == pytest.approx(2.0, abs=1e-9)


def test_predict_with_imu_accel_follows_kinematics():
    """A supplied accel drives the mean: s += v*dt + a*dt^2/2, ds += a*dt."""
    loc = TrackLocalizer(initial_s=0.0, initial_ds=1.0)
    a, dt = 2.0, 0.5
    loc.predict(dt, accel_mps2=a)
    s, ds = loc.state
    assert s == pytest.approx(1.0 * dt + 0.5 * a * dt ** 2, abs=1e-9)
    assert ds == pytest.approx(1.0 + a * dt, abs=1e-9)


def test_imu_accel_beats_constant_velocity_on_a_ramp():
    """Through a real acceleration ramp, the IMU-fed filter should track the
    true velocity better than the constant-velocity filter on the same noisy
    encoder stream."""
    rng = np.random.default_rng(7)
    dt, a = 0.02, 1.0           # accelerate at 1 m/s^2
    enc_sigma = 0.10            # noisy encoder
    with_imu = TrackLocalizer(initial_s=0.0, initial_ds=0.0)
    no_imu = TrackLocalizer(initial_s=0.0, initial_ds=0.0)
    v_true = 0.0
    err_imu = err_plain = 0.0
    n = 200
    for _ in range(n):
        v_true += a * dt
        enc = v_true + rng.normal(0, enc_sigma)
        acc = a + rng.normal(0, 0.05)     # decent IMU
        with_imu.predict(dt, accel_mps2=acc)
        with_imu.update_odom(enc)
        no_imu.predict(dt)
        no_imu.update_odom(enc)
        err_imu += abs(with_imu.state[1] - v_true)
        err_plain += abs(no_imu.state[1] - v_true)
    assert err_imu < err_plain   # IMU fusion reduces velocity error


def test_gyro_anchors_velocity_on_curve():
    """A tight in-curve gyro read pulls ds toward yaw_rate * radius."""
    loc = TrackLocalizer(initial_s=2.0, initial_ds=0.0)
    # radius 0.8 m, yaw rate 1.0 rad/s -> v = 0.8 m/s
    loc.update_gyro(GyroObservation(yaw_rate_rps=1.0, curve_radius_m=0.8,
                                    sigma_rps=1e-3))
    _, ds = loc.state
    assert ds == pytest.approx(0.8, abs=0.02)


def test_gyro_is_noop_on_straight_track():
    """No curve radius -> the yaw rate carries no speed info -> state untouched."""
    loc = TrackLocalizer(initial_s=2.0, initial_ds=1.0)
    before = loc.state
    loc.update_gyro(GyroObservation(yaw_rate_rps=0.0, curve_radius_m=None))
    loc.update_gyro(GyroObservation(yaw_rate_rps=5.0, curve_radius_m=float("inf")))
    assert loc.state == before
