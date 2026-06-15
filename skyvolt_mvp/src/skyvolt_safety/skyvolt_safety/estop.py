"""Emergency-stop chain with software fallback (Phase 4 — Operations & safety).

Aggregates every stop source — the hardware e-stop button, software faults
(load/tilt ESTOP, servo FAULT, ...), and a comms-loss watchdog — into one
latching chain:

    * ANY source asserting trips the chain ESTOPPED and cuts power.
    * The trip LATCHES: clearing the sources does NOT re-enable power.
    * reset() re-arms only when *all* sources are clear (an explicit operator
      acknowledgement); otherwise it refuses.

The comms watchdog is the software fallback: if heartbeats go stale (or never
arrived) the chain trips, so a wedged controller cannot leave the track live.
Pure-Python (stdlib); the hardware GPIO / ROS wiring is separate.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

COMMS_SOURCE = "comms_loss"


class EStopState(IntEnum):
    ARMED = 0        # normal — power enabled
    ESTOPPED = 1     # latched — power cut


@dataclass
class EStopChain:
    """Latching e-stop aggregator with a heartbeat (comms-loss) watchdog."""
    heartbeat_timeout_s: float = 1.0
    state: EStopState = EStopState.ARMED
    _active: set = field(default_factory=set, repr=False)
    _last_heartbeat_s: Optional[float] = field(default=None, repr=False)

    # -- stop sources ------------------------------------------------------
    def set_source(self, name: str, asserted: bool) -> None:
        """Assert/clear a named stop source. Asserting latches the trip."""
        if asserted:
            self._active.add(name)
            if self.state == EStopState.ARMED:
                self.state = EStopState.ESTOPPED      # latch on first trip
        else:
            self._active.discard(name)

    def trip(self, name: str = "manual") -> None:
        self.set_source(name, True)

    # -- comms-loss watchdog (software fallback) ---------------------------
    def heartbeat(self, now_s: float) -> None:
        """Record a liveness heartbeat and clear the comms-loss source."""
        self._last_heartbeat_s = now_s
        self.set_source(COMMS_SOURCE, False)

    def poll(self, now_s: float) -> None:
        """Trip the chain if the heartbeat is stale (or never arrived)."""
        stale = (self._last_heartbeat_s is None
                 or now_s - self._last_heartbeat_s > self.heartbeat_timeout_s)
        if stale:
            self.set_source(COMMS_SOURCE, True)

    # -- recovery ----------------------------------------------------------
    def reset(self) -> bool:
        """Re-arm only if every source is clear. Returns True on success."""
        if self._active:
            return False
        self.state = EStopState.ARMED
        return True

    # -- status ------------------------------------------------------------
    @property
    def power_enabled(self) -> bool:
        return self.state == EStopState.ARMED

    @property
    def tripped(self) -> bool:
        return self.state == EStopState.ESTOPPED

    @property
    def active_sources(self) -> frozenset:
        return frozenset(self._active)
