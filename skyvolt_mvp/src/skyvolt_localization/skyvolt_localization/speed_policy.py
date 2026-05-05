"""Two-stage deceleration FSM (paper section 3, Fig.3).

States:
    IDLE        - no goal assigned
    CRUISE      - moving at v_cruise, waiting for cue card
    APPROACH_1  - decelerated to v_approach1, waiting for positioning card
    APPROACH_2  - decelerated to v_approach2, waiting for photoeye
    DOCKED      - stopped at target
    FAULT       - unrecoverable: missed cards, overshoot, sensor stuck

Kept independent of ROS so it can be unit-tested and replayed offline.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional


class DockingState(IntEnum):
    IDLE = 0
    CRUISE = 1
    APPROACH_1 = 2
    APPROACH_2 = 3
    DOCKED = 4
    FAULT = 255


@dataclass
class SpeedProfile:
    """Speed targets per phase. Paper defaults."""
    cruise_straight_mps: float = 1.5
    cruise_curve_mps: float = 1.0
    approach1_mps: float = 0.8
    approach2_mps: float = 0.2
    # Hard accel limit for jerk-bounded ramps.
    max_decel_mps2: float = 1.5


@dataclass
class SpeedPolicyFSM:
    """Drives commanded velocity from localizer fused estimate.

    Usage:
        fsm = SpeedPolicyFSM()
        fsm.set_goal(target_arclength=12.34)
        for tick in time:
            cmd_v = fsm.step(s_hat, ds_hat, on_cue, on_pos, photoeye, dt, on_curve=False)
    """

    profile: SpeedProfile = field(default_factory=SpeedProfile)
    # Distance after which a missing expected card is considered FAULT.
    overshoot_tol_m: float = 0.30

    state: DockingState = DockingState.IDLE
    target_arclength: Optional[float] = None
    cue_arclength: Optional[float] = None
    pos_arclength: Optional[float] = None
    last_cmd_v: float = 0.0
    fault_reason: str = ""

    def set_goal(self, target_arclength: float,
                 cue_arclength: Optional[float] = None,
                 pos_arclength: Optional[float] = None) -> None:
        """Assign a docking goal.

        cue_arclength/pos_arclength are pre-mapped tag positions; if known
        they are used as overshoot watchdogs. If unknown, the FSM relies
        purely on event triggers.
        """
        self.target_arclength = float(target_arclength)
        self.cue_arclength = cue_arclength
        self.pos_arclength = pos_arclength
        self.state = DockingState.CRUISE
        self.fault_reason = ""

    def reset(self) -> None:
        self.state = DockingState.IDLE
        self.target_arclength = None
        self.cue_arclength = None
        self.pos_arclength = None
        self.last_cmd_v = 0.0
        self.fault_reason = ""

    # ------------------------------------------------------------------
    def step(
        self,
        s_hat: float,
        ds_hat: float,
        on_cue: bool,
        on_pos: bool,
        photoeye: bool,
        dt: float,
        on_curve: bool = False,
    ) -> float:
        """Advance one tick, return commanded velocity (m/s)."""
        # Always honor photoeye; emergency stop semantics.
        if photoeye and self.state in (
            DockingState.APPROACH_2, DockingState.APPROACH_1, DockingState.CRUISE,
        ):
            self.state = DockingState.DOCKED
            self.last_cmd_v = self._ramp(self.last_cmd_v, 0.0, dt)
            return self.last_cmd_v

        if self.state == DockingState.IDLE or self.state == DockingState.DOCKED:
            self.last_cmd_v = self._ramp(self.last_cmd_v, 0.0, dt)
            return self.last_cmd_v

        if self.state == DockingState.FAULT:
            self.last_cmd_v = self._ramp(self.last_cmd_v, 0.0, dt)
            return self.last_cmd_v

        # Overshoot watchdogs.
        if (self.state == DockingState.CRUISE
                and self.cue_arclength is not None
                and s_hat > self.cue_arclength + self.overshoot_tol_m
                and not on_cue):
            self._fault("missed cue card")
            return self.last_cmd_v
        if (self.state == DockingState.APPROACH_1
                and self.pos_arclength is not None
                and s_hat > self.pos_arclength + self.overshoot_tol_m
                and not on_pos):
            self._fault("missed positioning card")
            return self.last_cmd_v
        if (self.target_arclength is not None
                and s_hat > self.target_arclength + self.overshoot_tol_m):
            self._fault("overshot target")
            return self.last_cmd_v

        # Forward transitions.
        if self.state == DockingState.CRUISE and on_cue:
            self.state = DockingState.APPROACH_1
        elif self.state == DockingState.APPROACH_1 and on_pos:
            self.state = DockingState.APPROACH_2

        target_v = self._target_speed(on_curve)
        self.last_cmd_v = self._ramp(self.last_cmd_v, target_v, dt)
        return self.last_cmd_v

    # ------------------------------------------------------------------
    def _target_speed(self, on_curve: bool) -> float:
        p = self.profile
        if self.state == DockingState.CRUISE:
            return p.cruise_curve_mps if on_curve else p.cruise_straight_mps
        if self.state == DockingState.APPROACH_1:
            return p.approach1_mps
        if self.state == DockingState.APPROACH_2:
            return p.approach2_mps
        return 0.0

    def _ramp(self, current: float, target: float, dt: float) -> float:
        if dt <= 0:
            return current
        max_step = self.profile.max_decel_mps2 * dt
        if target > current:
            return min(current + max_step, target)
        return max(current - max_step, target)

    def _fault(self, reason: str) -> None:
        self.state = DockingState.FAULT
        self.fault_reason = reason
        self.last_cmd_v = 0.0
