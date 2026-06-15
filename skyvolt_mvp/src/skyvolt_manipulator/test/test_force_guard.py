"""Force/torque misalignment-detection tests (Phase 3)."""
import math

import pytest

from skyvolt_manipulator.force_guard import (
    InsertionForceGuard, Wrench, ContactState, GuardAction,
)


def test_no_contact_is_free_and_continue():
    g = InsertionForceGuard()
    r = g.assess(Wrench(fz=0.2))           # below contact threshold
    assert r.state == ContactState.FREE
    assert r.action == GuardAction.CONTINUE


def test_clean_axial_insertion_is_seating():
    g = InsertionForceGuard()
    r = g.assess(Wrench(fz=10.0, fx=0.5, fy=0.5))   # mostly axial
    assert r.state == ContactState.SEATING
    assert r.action == GuardAction.CONTINUE
    assert r.lateral_ratio < g.lateral_ratio_tol


def test_lateral_binding_is_misaligned_correct():
    g = InsertionForceGuard()
    # lateral force is half the axial -> ratio 0.5 > 0.30
    r = g.assess(Wrench(fz=10.0, fx=5.0))
    assert r.state == ContactState.MISALIGNED
    assert r.action == GuardAction.CORRECT
    assert r.lateral_ratio == pytest.approx(0.5)


def test_cocking_moment_is_misaligned():
    g = InsertionForceGuard()
    r = g.assess(Wrench(fz=10.0, mx=0.8))   # axial fine, but cocking moment high
    assert r.state == ContactState.MISALIGNED
    assert r.action == GuardAction.CORRECT


def test_overload_force_aborts():
    g = InsertionForceGuard(force_limit_n=50.0)
    r = g.assess(Wrench(fz=60.0))
    assert r.state == ContactState.OVERLOAD
    assert r.action == GuardAction.ABORT


def test_overload_moment_aborts():
    g = InsertionForceGuard(moment_limit_nm=5.0)
    r = g.assess(Wrench(fz=10.0, mx=6.0))
    assert r.state == ContactState.OVERLOAD
    assert r.action == GuardAction.ABORT


def test_overload_takes_priority_over_misalignment():
    """A huge lateral force is both misaligned and overloaded -> ABORT wins."""
    g = InsertionForceGuard()
    r = g.assess(Wrench(fz=10.0, fx=100.0))
    assert r.state == ContactState.OVERLOAD
    assert r.action == GuardAction.ABORT


def test_lateral_components_reported():
    g = InsertionForceGuard()
    r = g.assess(Wrench(fz=10.0, fx=3.0, fy=4.0, mx=0.1, my=0.2))
    assert r.lateral_force_n == pytest.approx(5.0)            # 3-4-5
    assert r.lateral_moment_nm == pytest.approx(math.hypot(0.1, 0.2))
