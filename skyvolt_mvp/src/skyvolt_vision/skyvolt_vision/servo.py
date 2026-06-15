"""Closed-loop visual servoing for plug insertion (Phase 3, paper 2).

Ties the vision pose estimate (skyvolt_vision.port_pose) to the lead-screw
manipulator: observe the charging-port pose, drive the in-plane misalignment to
zero, then extend the telescope to insert and latch. A two-phase controller:

    ALIGN   — proportional lateral correction until |offset| < tolerance
    INSERT  — advance the telescope reach until the port is seated, then latch
    DOCKED  — terminal

The controller is pure-Python and is tested in closed loop against a simple
kinematic plant (commanding a lateral velocity shifts the observed port pose).
The reach/insert parameters come from the manipulator stroke; the lateral
offset is exactly skyvolt_vision.lateral_offset of the estimated pose.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum

from .port_pose import Pose6DoF


class ServoPhase(IntEnum):
    ALIGN = 0
    INSERT = 1
    DOCKED = 2
    FAULT = 255


@dataclass
class ServoCommand:
    """One control output: lateral plug velocity + telescope advance + latch."""
    vx: float            # lateral correction, plug +x (m/s)
    vy: float            # lateral correction, plug +y (m/s)
    reach_vel: float     # telescope advance (m/s)
    engage: bool         # latch the connector
    phase: ServoPhase


def _clamp(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


@dataclass
class VisualServo:
    """Proportional align-then-insert visual servo controller."""
    align_tol_m: float = 0.002       # 2 mm in-plane alignment tolerance
    kp: float = 3.0                  # proportional gain (1/s)
    max_lateral_mps: float = 0.05    # lateral correction speed cap
    insert_mps: float = 0.05         # telescope advance speed
    seat_depth_m: float = 0.02       # remaining approach distance at which we latch
    max_ticks: int = 2000            # give up (FAULT) if alignment never converges

    phase: ServoPhase = ServoPhase.ALIGN
    _ticks: int = field(default=0, repr=False)

    def step(self, pose: Pose6DoF, dt: float) -> ServoCommand:
        """Advance one control tick from the current port-pose observation."""
        if self.phase in (ServoPhase.DOCKED, ServoPhase.FAULT):
            return ServoCommand(0.0, 0.0, 0.0, False, self.phase)

        self._ticks += 1
        if self._ticks > self.max_ticks:
            self.phase = ServoPhase.FAULT
            return ServoCommand(0.0, 0.0, 0.0, False, self.phase)

        x, y, z = float(pose.t[0]), float(pose.t[1]), float(pose.t[2])
        lateral = math.hypot(x, y)

        if self.phase == ServoPhase.ALIGN:
            if lateral <= self.align_tol_m:
                self.phase = ServoPhase.INSERT
            else:
                # Move the plug toward the port to shrink the offset.
                return ServoCommand(
                    vx=_clamp(self.kp * x, self.max_lateral_mps),
                    vy=_clamp(self.kp * y, self.max_lateral_mps),
                    reach_vel=0.0, engage=False, phase=ServoPhase.ALIGN)

        # INSERT
        if z <= self.seat_depth_m:
            self.phase = ServoPhase.DOCKED
            return ServoCommand(0.0, 0.0, 0.0, True, ServoPhase.DOCKED)
        return ServoCommand(0.0, 0.0, self.insert_mps, False, ServoPhase.INSERT)


@dataclass
class ServoResult:
    docked: bool
    phase: ServoPhase
    ticks: int
    final_lateral_m: float
    final_depth_m: float


def _advance_plant(pose: Pose6DoF, cmd: ServoCommand, dt: float) -> Pose6DoF:
    """Kinematic plant: a lateral plug velocity shifts the observed port
    laterally; telescope reach closes the approach distance."""
    t = pose.t.astype(float).copy()
    t[0] -= cmd.vx * dt
    t[1] -= cmd.vy * dt
    t[2] -= cmd.reach_vel * dt
    return Pose6DoF(R=pose.R, t=t)


def simulate_insertion(servo: VisualServo, initial_pose: Pose6DoF,
                       dt: float = 0.02, max_steps: int = 5000) -> ServoResult:
    """Run the controller in closed loop against the kinematic plant."""
    pose = initial_pose
    steps = 0
    for steps in range(1, max_steps + 1):
        cmd = servo.step(pose, dt)
        if servo.phase in (ServoPhase.DOCKED, ServoPhase.FAULT):
            break
        pose = _advance_plant(pose, cmd, dt)
    return ServoResult(
        docked=servo.phase == ServoPhase.DOCKED,
        phase=servo.phase, ticks=steps,
        final_lateral_m=math.hypot(float(pose.t[0]), float(pose.t[1])),
        final_depth_m=float(pose.t[2]))
