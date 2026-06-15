"""Property-based fault-injection tests for the track-arclength Kalman filter.

Phase 1 (Hardening Localization): instead of a handful of hand-picked inputs,
Hypothesis generates adversarial sequences of predict/odom/RFID/photoeye calls
and checks invariants that must hold for *any* finite input — the kind of edge
cases (degenerate dt, far-off measurements, tight/loose sigmas) that a fielded
localizer will eventually see.
"""
from __future__ import annotations

import numpy as np
from hypothesis import given, settings, strategies as st

from skyvolt_localization.track_localizer import (
    TrackLocalizer, RfidObservation, PhotoeyeObservation, GyroObservation,
)

# ---------------------------------------------------------------------------
# Strategies: one "op" is a single localizer call. A test replays a list of ops.
finite = lambda lo, hi: st.floats(  # noqa: E731
    min_value=lo, max_value=hi, allow_nan=False, allow_infinity=False)

_dt = finite(1e-3, 1.0)
_meas = finite(-1e4, 1e4)
_sigma = finite(1e-4, 1.0)
_accel = finite(-20.0, 20.0)
_yaw = finite(-10.0, 10.0)
_radius = finite(0.1, 50.0)

ops = st.lists(
    st.one_of(
        st.tuples(st.just("predict"), _dt),
        st.tuples(st.just("predict_imu"), _dt, _accel),     # IMU control input
        st.tuples(st.just("odom"), _meas),
        st.tuples(st.just("gyro"), _yaw, _radius, _sigma),  # in-curve velocity
        st.tuples(st.just("rfid"), _meas, _sigma),
        st.tuples(st.just("photoeye"), _meas, _sigma),
    ),
    min_size=0, max_size=200,
)


def _apply(loc: TrackLocalizer, op) -> None:
    kind = op[0]
    if kind == "predict":
        loc.predict(op[1])
    elif kind == "predict_imu":
        loc.predict(op[1], accel_mps2=op[2])
    elif kind == "odom":
        loc.update_odom(op[1])
    elif kind == "gyro":
        loc.update_gyro(GyroObservation(
            yaw_rate_rps=op[1], curve_radius_m=op[2], sigma_rps=op[3]))
    elif kind == "rfid":
        loc.update_rfid(RfidObservation(tag_arclength_m=op[1], sigma_m=op[2]))
    elif kind == "photoeye":
        loc.update_photoeye(PhotoeyeObservation(
            docking_arclength_m=op[1], sigma_m=op[2], triggered=True))


# ---------------------------------------------------------------------------
@given(ops)
@settings(max_examples=300)
def test_covariance_stays_symmetric_and_psd(op_list):
    """P must remain symmetric and positive-semidefinite no matter the input."""
    loc = TrackLocalizer()
    for op in op_list:
        _apply(loc, op)
        P = loc.covariance
        assert np.allclose(P, P.T, atol=1e-9), "covariance lost symmetry"
        eigs = np.linalg.eigvalsh(P)
        assert eigs.min() > -1e-9, f"covariance not PSD: min eig {eigs.min()}"


@given(ops)
@settings(max_examples=300)
def test_state_and_covariance_stay_finite(op_list):
    """Finite inputs must never produce NaN/Inf in the estimate or covariance."""
    loc = TrackLocalizer()
    for op in op_list:
        _apply(loc, op)
    s, ds = loc.state
    assert np.isfinite(s) and np.isfinite(ds)
    assert np.isfinite(loc.covariance).all()


@given(prefix=ops, z=_meas, sigma=_sigma)
@settings(max_examples=300)
def test_measurement_never_increases_observed_variance(prefix, z, sigma):
    """A KF correction can only shrink (never inflate) the variance of the
    dimension it measures. RFID/photoeye observe position s -> P[0,0]."""
    loc = TrackLocalizer()
    for op in prefix:
        _apply(loc, op)
    before = loc.covariance[0, 0]
    loc.update_rfid(RfidObservation(tag_arclength_m=z, sigma_m=sigma))
    after = loc.covariance[0, 0]
    assert after <= before + 1e-9


@given(prefix=ops, z=_meas)
@settings(max_examples=300)
def test_tight_anchor_pulls_estimate_toward_measurement(prefix, z):
    """A near-zero-sigma RFID fix must move s toward the measurement, not away."""
    loc = TrackLocalizer()
    for op in prefix:
        _apply(loc, op)
    s_before = loc.state[0]
    loc.update_rfid(RfidObservation(tag_arclength_m=z, sigma_m=1e-4))
    s_after = loc.state[0]
    assert abs(s_after - z) <= abs(s_before - z) + 1e-6


@given(op_list=ops, bad_dt=st.floats(min_value=-100.0, max_value=0.0,
                                     allow_nan=False, allow_infinity=False))
@settings(max_examples=200)
def test_nonpositive_dt_predict_is_noop(op_list, bad_dt):
    """predict(dt<=0) must leave the estimate and covariance untouched."""
    loc = TrackLocalizer()
    for op in op_list:
        _apply(loc, op)
    x_before, P_before = loc.state, loc.covariance
    loc.predict(bad_dt)
    assert loc.state == x_before
    assert np.array_equal(loc.covariance, P_before)
