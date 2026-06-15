"""Unit tests for the lead-screw manipulator kinematics (Phase 3)."""
import math

import pytest

from skyvolt_manipulator.manipulator import (
    LeadScrew, TelescopicManipulator, InsertionPlan,
)


# --- LeadScrew geometry ----------------------------------------------------
def test_displacement_is_linear_in_revolutions():
    s = LeadScrew(lead_m=0.01, stroke_m=0.32)
    assert s.displacement(0) == pytest.approx(0.0)
    assert s.displacement(10) == pytest.approx(0.10)     # 10 rev * 10 mm
    assert s.displacement(16) == pytest.approx(0.16)


def test_displacement_clamps_to_stroke():
    s = LeadScrew(lead_m=0.01, stroke_m=0.32)
    assert s.displacement(1000) == pytest.approx(0.32)   # past the end
    assert s.displacement(-5) == pytest.approx(0.0)      # before the start


def test_revolutions_and_displacement_are_inverse():
    s = LeadScrew(lead_m=0.01, stroke_m=0.32)
    for d in (0.0, 0.05, 0.2, 0.32):
        assert s.displacement(s.revolutions_for(d)) == pytest.approx(d)


def test_motor_angle_matches_revolutions():
    s = LeadScrew(lead_m=0.01, stroke_m=0.32)
    # 0.05 m = 5 rev = 10*pi rad
    assert s.motor_angle_for(0.05) == pytest.approx(5 * 2 * math.pi)


# --- rates -----------------------------------------------------------------
def test_linear_speed_scales_with_rpm_and_lead():
    s = LeadScrew(lead_m=0.01, stroke_m=0.32, max_rpm=600)
    assert s.linear_speed(600) == pytest.approx(0.1)     # 600/60 * 0.01
    assert s.max_linear_speed == pytest.approx(0.1)
    assert s.linear_speed(0) == 0.0


def test_travel_time_uses_distance_over_speed():
    s = LeadScrew(lead_m=0.01, stroke_m=0.32, max_rpm=600)  # 0.1 m/s
    assert s.travel_time(0.0, 0.2) == pytest.approx(2.0)
    assert s.travel_time(0.2, 0.0) == pytest.approx(2.0)    # symmetric
    assert s.travel_time(0.0, 0.1, rpm=300) == pytest.approx(2.0)  # half speed


def test_zero_speed_travel_time_is_infinite():
    s = LeadScrew(lead_m=0.01, stroke_m=0.32)
    assert math.isinf(s.travel_time(0.0, 0.1, rpm=0))


# --- force / mechanical advantage ------------------------------------------
def test_smaller_lead_gives_more_thrust():
    coarse = LeadScrew(lead_m=0.010, stroke_m=0.32, efficiency=1.0)
    fine = LeadScrew(lead_m=0.002, stroke_m=0.04, efficiency=1.0)
    # Same torque, finer lead -> more axial force.
    assert fine.thrust(1.0) > coarse.thrust(1.0)
    # F = 2*pi*eta*T / lead
    assert coarse.thrust(1.0) == pytest.approx(2 * math.pi / 0.010)


def test_efficiency_reduces_thrust():
    ideal = LeadScrew(lead_m=0.01, stroke_m=0.32, efficiency=1.0)
    real = LeadScrew(lead_m=0.01, stroke_m=0.32, efficiency=0.8)
    assert real.thrust(1.0) == pytest.approx(0.8 * ideal.thrust(1.0))


def test_feasible_respects_stroke():
    s = LeadScrew(lead_m=0.01, stroke_m=0.32)
    assert s.feasible(0.0) and s.feasible(0.32) and s.feasible(0.16)
    assert not s.feasible(0.33)
    assert not s.feasible(-0.01)


# --- TelescopicManipulator insertion planning ------------------------------
def test_insertion_plan_is_feasible_and_timed():
    m = TelescopicManipulator()
    plan = m.plan_insertion(reach_m=0.30, engage_m=0.03)
    assert isinstance(plan, InsertionPlan) and plan.feasible
    # reach: 0.30 m @ 0.1 m/s = 3.0 s; engage: 0.03 @ 0.05 m/s = 0.6 s
    assert plan.reach_time_s == pytest.approx(3.0)
    assert plan.engage_time_s == pytest.approx(0.6)
    assert plan.total_time_s == pytest.approx(3.6)
    assert plan.reach_rev == pytest.approx(0.30 / 0.010)


def test_insertion_rejects_out_of_stroke_reach():
    m = TelescopicManipulator()
    plan = m.plan_insertion(reach_m=0.50, engage_m=0.03)   # > 0.32 m stroke
    assert not plan.feasible
    assert "reach" in plan.reason


def test_insertion_rejects_out_of_stroke_engage():
    m = TelescopicManipulator()
    plan = m.plan_insertion(reach_m=0.30, engage_m=0.10)   # > 0.04 m stroke
    assert not plan.feasible
    assert "engage" in plan.reason


def test_default_axes_match_urdf_velocity_limits():
    m = TelescopicManipulator()
    assert m.reach.max_linear_speed == pytest.approx(0.1)    # URDF telescope_y
    assert m.engage.max_linear_speed == pytest.approx(0.05)  # URDF latch_x
    assert m.reach.stroke_m == pytest.approx(0.32)
    assert m.engage.stroke_m == pytest.approx(0.04)
