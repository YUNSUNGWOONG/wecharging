"""Force/torque misalignment detection during plug insertion (Phase 3).

A wrist F/T sensor sees the contact wrench while the lead-screw manipulator
pushes the plug into the socket. A well-aligned insertion is almost pure axial
force (along the approach axis); a misaligned plug binds against the socket rim,
showing up as lateral force and/or a cocking moment. This guard classifies the
contact and recommends an action — so the visual-servo loop can back off and
re-align (CORRECT) or stop before damage (ABORT).

Pure-Python (stdlib only); the F/T sensor itself / Gazebo plugin is separate.
Convention: z is the insertion (approach) axis.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum


class ContactState(IntEnum):
    FREE = 0          # not in contact yet (approaching)
    SEATING = 1       # clean axial insertion
    MISALIGNED = 2    # binding: lateral force / cocking moment
    OVERLOAD = 3      # force or moment past the safe limit


class GuardAction(IntEnum):
    CONTINUE = 0      # keep inserting
    CORRECT = 1       # back off and re-align
    ABORT = 2         # stop now


@dataclass(frozen=True)
class Wrench:
    """Contact wrench at the plug (sensor frame); z = insertion axis."""
    fx: float = 0.0   # N (lateral)
    fy: float = 0.0   # N (lateral)
    fz: float = 0.0   # N (axial)
    mx: float = 0.0   # N·m (cocking)
    my: float = 0.0   # N·m (cocking)
    mz: float = 0.0   # N·m (about insertion axis)


@dataclass(frozen=True)
class GuardReading:
    state: ContactState
    action: GuardAction
    lateral_ratio: float    # |F_lateral| / |F_axial|
    lateral_force_n: float
    lateral_moment_nm: float


@dataclass
class InsertionForceGuard:
    """Classifies the insertion contact wrench and recommends an action."""
    contact_force_n: float = 1.0       # axial below this -> not yet seated (FREE)
    lateral_ratio_tol: float = 0.30    # |F_lat|/|F_ax| above this -> misaligned
    moment_tol_nm: float = 0.50        # cocking moment above this -> misaligned
    force_limit_n: float = 50.0        # total force above this -> overload
    moment_limit_nm: float = 5.0       # any moment above this -> overload

    def assess(self, w: Wrench) -> GuardReading:
        f_ax = abs(w.fz)
        f_lat = math.hypot(w.fx, w.fy)
        m_lat = math.hypot(w.mx, w.my)
        total_f = math.sqrt(w.fx ** 2 + w.fy ** 2 + w.fz ** 2)
        ratio = f_lat / f_ax if f_ax > 1e-9 else math.inf

        if total_f > self.force_limit_n or m_lat > self.moment_limit_nm:
            state, action = ContactState.OVERLOAD, GuardAction.ABORT
        elif f_ax < self.contact_force_n:
            state, action = ContactState.FREE, GuardAction.CONTINUE
        elif ratio > self.lateral_ratio_tol or m_lat > self.moment_tol_nm:
            state, action = ContactState.MISALIGNED, GuardAction.CORRECT
        else:
            state, action = ContactState.SEATING, GuardAction.CONTINUE

        return GuardReading(
            state=state, action=action,
            lateral_ratio=ratio, lateral_force_n=f_lat, lateral_moment_nm=m_lat)
