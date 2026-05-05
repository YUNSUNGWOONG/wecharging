"""Unit tests for the track-arclength Kalman filter."""
import numpy as np
import pytest

from skyvolt_localization.track_localizer import (
    TrackLocalizer, RfidObservation, PhotoeyeObservation,
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
