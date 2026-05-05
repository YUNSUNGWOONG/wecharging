"""Track segment reservation table.

Each robot's planned route is encoded as a list of (segment_id, t_enter, t_exit)
intervals. The table refuses to admit a reservation that overlaps an existing
one for the same segment.

This is the standard approach for warehouse AGV fleets and airport tarmac
scheduling. It works in 1D arclength because the track topology is linear
within each branch.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable


@dataclass(frozen=True)
class Reservation:
    robot_id: str
    segment_id: int
    t_enter: float       # seconds, monotonic
    t_exit: float

    def overlaps(self, other: "Reservation") -> bool:
        if self.segment_id != other.segment_id:
            return False
        if self.robot_id == other.robot_id:
            # Same robot's intervals on same segment: not a conflict
            return False
        # Closed-interval overlap with epsilon to avoid pathological exact
        # touches counting as conflicts.
        eps = 1e-3
        return not (self.t_exit + eps <= other.t_enter
                    or other.t_exit + eps <= self.t_enter)


@dataclass
class Conflict:
    incoming: Reservation
    existing: Reservation


@dataclass
class ReservationTable:
    """Append-only with optional release-by-robot.

    Optimized for small fleets (<100 robots, <10k active reservations).
    Replace with an interval-tree if it grows.
    """
    _by_segment: dict[int, list[Reservation]] = field(default_factory=dict)

    def admit(self, r: Reservation) -> Conflict | None:
        """Try to insert; return None on success, Conflict otherwise."""
        for existing in self._by_segment.get(r.segment_id, []):
            if r.overlaps(existing):
                return Conflict(incoming=r, existing=existing)
        self._by_segment.setdefault(r.segment_id, []).append(r)
        return None

    def admit_route(self, route: Iterable[Reservation]) -> list[Conflict]:
        """All-or-nothing admit; rolls back on first conflict."""
        accepted: list[Reservation] = []
        for r in route:
            c = self.admit(r)
            if c is not None:
                # Roll back
                for a in accepted:
                    self._by_segment[a.segment_id].remove(a)
                return [c]
            accepted.append(r)
        return []

    def release_robot(self, robot_id: str) -> int:
        """Remove all reservations belonging to a robot. Returns count."""
        count = 0
        for seg_id, lst in self._by_segment.items():
            kept = [r for r in lst if r.robot_id != robot_id]
            count += len(lst) - len(kept)
            self._by_segment[seg_id] = kept
        return count

    def active_for(self, robot_id: str) -> list[Reservation]:
        out: list[Reservation] = []
        for lst in self._by_segment.values():
            out.extend(r for r in lst if r.robot_id == robot_id)
        return out

    def num_reservations(self) -> int:
        return sum(len(v) for v in self._by_segment.values())
