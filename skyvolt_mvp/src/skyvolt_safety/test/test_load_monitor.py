"""Rail load + track-tilt safety monitor tests (Phase 4)."""
import pytest

from skyvolt_safety.load_monitor import (
    LoadMonitor, LoadState, SafetyAction,
)


def test_balanced_load_is_nominal():
    m = LoadMonitor(rated_load_n=2000.0)
    r = m.assess({"n1": 400, "n2": 400, "s1": 400, "s2": 400}, tilt_deg=0.2)
    assert r.state == LoadState.NOMINAL
    assert r.action == SafetyAction.OK
    assert r.total_n == pytest.approx(1600)
    assert r.max_imbalance == pytest.approx(0.0)


def test_overload_trips_estop():
    m = LoadMonitor(rated_load_n=2000.0)
    r = m.assess([600, 600, 600, 600])     # total 2400 > 2000
    assert r.state == LoadState.OVERLOAD
    assert r.action == SafetyAction.ESTOP


def test_excess_tilt_trips_estop():
    m = LoadMonitor(tilt_limit_deg=2.0)
    r = m.assess([400, 400, 400, 400], tilt_deg=3.5)
    assert r.state == LoadState.TILT_FAULT
    assert r.action == SafetyAction.ESTOP


def test_tilt_sign_does_not_matter():
    m = LoadMonitor(tilt_limit_deg=2.0)
    assert m.assess([400, 400], tilt_deg=-3.0).action == SafetyAction.ESTOP


def test_imbalance_holds_the_fleet():
    m = LoadMonitor(imbalance_tol=0.40)
    # mean 400; one support at 700 -> deviation 0.75 > 0.40
    r = m.assess({"n1": 700, "n2": 300, "s1": 300, "s2": 300})
    assert r.state == LoadState.IMBALANCED
    assert r.action == SafetyAction.HOLD
    assert r.max_imbalance == pytest.approx(0.75, abs=1e-3)


def test_overload_takes_priority_over_imbalance():
    m = LoadMonitor(rated_load_n=1000.0, imbalance_tol=0.40)
    r = m.assess([900, 100, 100, 100])     # total 1200 > 1000 AND imbalanced
    assert r.state == LoadState.OVERLOAD
    assert r.action == SafetyAction.ESTOP


def test_imbalance_metric_is_max_deviation_fraction():
    m = LoadMonitor()
    # mean 500; max deviation 100 -> 0.2
    r = m.assess([600, 400, 500, 500])
    assert r.max_imbalance == pytest.approx(0.2)


def test_handles_zero_and_empty_loads():
    m = LoadMonitor()
    assert m.assess([]).state == LoadState.NOMINAL
    assert m.assess([0, 0, 0, 0]).action == SafetyAction.OK
