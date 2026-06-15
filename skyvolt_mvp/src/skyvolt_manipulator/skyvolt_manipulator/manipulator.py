"""Lead-screw telescopic manipulator kinematics (Phase 3).

The SkyvoltRobot charging plug is driven by two lead-screw actuators
(paper sec 4.2.3 / 5):

    * Y-axis telescope  — extends the plug toward the vehicle port (320 mm stroke)
    * X-axis latch      — engages/locks the connector (40 mm stroke)

A lead screw turns motor rotation into linear travel: one revolution advances
the nut by the screw *lead*. That gives a clean linear map between motor angle
and stroke, a torque->thrust mechanical advantage fixed by the lead, and a hard
travel limit at the stroke. Kept pure-Python (stdlib only) so it runs in CI
without ROS, like the localization and fleet cores.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

TWO_PI = 2.0 * math.pi


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


@dataclass(frozen=True)
class LeadScrew:
    """A single lead-screw linear actuator."""
    lead_m: float              # axial advance per revolution (m/rev)
    stroke_m: float            # usable travel (m)
    max_rpm: float = 600.0     # motor speed limit (rev/min)
    efficiency: float = 0.9    # 0..1 mechanical efficiency

    # -- geometry (motor rotation <-> linear travel) -----------------------
    def displacement(self, revolutions: float) -> float:
        """Linear position for a (fractional) revolution count, clamped to the
        physical stroke."""
        return _clamp(revolutions * self.lead_m, 0.0, self.stroke_m)

    def revolutions_for(self, displacement_m: float) -> float:
        """Revolutions needed to reach a displacement (unclamped)."""
        return displacement_m / self.lead_m

    def motor_angle_for(self, displacement_m: float) -> float:
        """Motor shaft angle (rad) for a displacement."""
        return TWO_PI * displacement_m / self.lead_m

    # -- rates -------------------------------------------------------------
    def linear_speed(self, rpm: float) -> float:
        """Linear speed (m/s) at a given motor speed (rev/min)."""
        return rpm / 60.0 * self.lead_m

    @property
    def max_linear_speed(self) -> float:
        return self.linear_speed(self.max_rpm)

    def travel_time(self, d_from: float, d_to: float,
                    rpm: float | None = None) -> float:
        """Seconds to move between two displacements (default: max speed)."""
        v = self.linear_speed(self.max_rpm if rpm is None else rpm)
        return math.inf if v <= 0 else abs(d_to - d_from) / v

    # -- force (mechanical advantage) --------------------------------------
    def thrust(self, motor_torque_nm: float) -> float:
        """Axial force (N) produced by a motor torque. Smaller lead -> more
        force (and slower travel) for the same torque."""
        return TWO_PI * self.efficiency * motor_torque_nm / self.lead_m

    def feasible(self, displacement_m: float, tol: float = 1e-9) -> bool:
        return -tol <= displacement_m <= self.stroke_m + tol


@dataclass(frozen=True)
class InsertionPlan:
    """Result of planning a plug insertion."""
    feasible: bool
    reach_rev: float = 0.0
    engage_rev: float = 0.0
    reach_time_s: float = 0.0
    engage_time_s: float = 0.0
    total_time_s: float = 0.0
    reason: str = ""


@dataclass
class TelescopicManipulator:
    """Two-axis lead-screw plug manipulator: telescope (reach) then latch
    (engage). Defaults match the URDF joint limits (0.32 m @ 0.1 m/s reach,
    0.04 m @ 0.05 m/s engage)."""
    reach: LeadScrew = field(default_factory=lambda: LeadScrew(
        lead_m=0.010, stroke_m=0.32, max_rpm=600.0))   # 0.1 m/s max
    engage: LeadScrew = field(default_factory=lambda: LeadScrew(
        lead_m=0.004, stroke_m=0.04, max_rpm=750.0))   # 0.05 m/s max

    def plan_insertion(self, reach_m: float, engage_m: float,
                       rpm: float | None = None) -> InsertionPlan:
        """Plan a sequential insertion: extend the telescope to `reach_m`, then
        drive the latch to `engage_m`. Returns an infeasible plan (with reason)
        if either target is outside its actuator's stroke."""
        if not self.reach.feasible(reach_m):
            return InsertionPlan(False, reason=f"reach {reach_m:.3f} m exceeds "
                                 f"telescope stroke {self.reach.stroke_m:.3f} m")
        if not self.engage.feasible(engage_m):
            return InsertionPlan(False, reason=f"engage {engage_m:.3f} m exceeds "
                                 f"latch stroke {self.engage.stroke_m:.3f} m")
        rt = self.reach.travel_time(0.0, reach_m, rpm)
        et = self.engage.travel_time(0.0, engage_m, rpm)
        return InsertionPlan(
            feasible=True,
            reach_rev=self.reach.revolutions_for(reach_m),
            engage_rev=self.engage.revolutions_for(engage_m),
            reach_time_s=rt, engage_time_s=et, total_time_s=rt + et)
