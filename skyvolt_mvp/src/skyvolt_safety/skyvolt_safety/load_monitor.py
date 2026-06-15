"""Rail load + track-tilt safety monitor (Phase 4 — Operations & safety).

Each rail support is instrumented with a load cell; an inclinometer reports the
track tilt. This monitor watches for structural danger while robots run on the
suspended track:

    * OVERLOAD   — total supported load exceeds the rated capacity
    * TILT_FAULT — track tilt past the safe limit (sagging / support failure)
    * IMBALANCED — one support carries a disproportionate share (a jammed robot
                   or a degrading support)

and recommends an operations action: OK / HOLD (pause the fleet and inspect) /
ESTOP (cut power — structural danger). Pure-Python (stdlib) so it runs in CI;
the load-cell / inclinometer drivers are separate.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Mapping


class LoadState(IntEnum):
    NOMINAL = 0
    IMBALANCED = 1
    OVERLOAD = 2
    TILT_FAULT = 3


class SafetyAction(IntEnum):
    OK = 0
    HOLD = 1      # pause fleet, inspect
    ESTOP = 2     # cut power immediately


@dataclass(frozen=True)
class LoadReading:
    state: LoadState
    action: SafetyAction
    total_n: float
    max_imbalance: float    # largest per-support deviation from the mean (fraction)
    tilt_deg: float


@dataclass
class LoadMonitor:
    """Assess rail-support load cells + track tilt against safety limits."""
    rated_load_n: float = 2000.0     # total structural capacity
    imbalance_tol: float = 0.40      # max per-support deviation from mean (frac)
    tilt_limit_deg: float = 2.0      # max safe track tilt

    def assess(self, support_loads_n: Mapping[str, float] | list[float],
               tilt_deg: float = 0.0) -> LoadReading:
        loads = (list(support_loads_n.values())
                 if isinstance(support_loads_n, Mapping)
                 else list(support_loads_n))
        total = sum(loads)
        n = len(loads)
        mean = total / n if n else 0.0
        imbalance = (max(abs(x - mean) for x in loads) / mean
                     if mean > 1e-9 else 0.0)

        # Structural-danger conditions trip an e-stop and take priority.
        if total > self.rated_load_n:
            state, action = LoadState.OVERLOAD, SafetyAction.ESTOP
        elif abs(tilt_deg) > self.tilt_limit_deg:
            state, action = LoadState.TILT_FAULT, SafetyAction.ESTOP
        elif imbalance > self.imbalance_tol:
            state, action = LoadState.IMBALANCED, SafetyAction.HOLD
        else:
            state, action = LoadState.NOMINAL, SafetyAction.OK

        return LoadReading(state=state, action=action, total_n=total,
                           max_imbalance=imbalance, tilt_deg=tilt_deg)
