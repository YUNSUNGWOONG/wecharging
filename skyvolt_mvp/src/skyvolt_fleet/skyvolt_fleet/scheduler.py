"""Greedy charge-pile delivery scheduler.

Decisions are 1D in arclength because the closed-loop track is parameterized
by `s`. The scheduler:

1. Receives jobs (target arclength + priority + deadline).
2. For each idle robot, picks the cheapest reachable job (by travel-time +
   priority weighting).
3. Builds a reservation route for the assignment and admits it; if it
   conflicts, falls back to the next-best job.

Worst-case complexity O(R*J*S) where S is the number of segments crossed.
For MVP scale (R<=10, J<=200, S<=20) this is well below 1 ms.

Replace with auction or MILP for higher loads — same `assign()` interface.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Optional

from .reservation import ReservationTable, Reservation


@dataclass(frozen=True)
class Segment:
    """Atomic unit of the track for reservations."""
    segment_id: int
    branch_id: int
    s_start: float
    s_end: float

    @property
    def length(self) -> float:
        return self.s_end - self.s_start


@dataclass
class TrackTopology:
    """Branch + ordered segment list. Single branch for the MVP."""
    segments: list[Segment]

    def segments_between(self, s0: float, s1: float) -> list[Segment]:
        lo, hi = (s0, s1) if s0 <= s1 else (s1, s0)
        return [seg for seg in self.segments
                if not (seg.s_end < lo or seg.s_start > hi)]


@dataclass
class Job:
    job_id: str
    target_arclength_m: float
    priority: int = 0           # higher = more urgent
    deadline_s: float = float("inf")
    submitted_at_s: float = 0.0


@dataclass
class RobotState:
    robot_id: str
    arclength_m: float
    nominal_speed_mps: float = 1.5
    busy: bool = False


@dataclass
class Assignment:
    job: Job
    robot: RobotState
    eta_s: float
    route: list[Reservation]


@dataclass
class Scheduler:
    topology: TrackTopology
    table: ReservationTable = field(default_factory=ReservationTable)
    # Tunables
    priority_time_weight_s: float = 5.0   # 1 priority point ~= 5 s saved
    safety_buffer_s: float = 0.5

    def assign(self,
               jobs: Iterable[Job],
               fleet: Iterable[RobotState],
               now_s: float) -> list[Assignment]:
        """Return one assignment per idle robot, modifying reservations."""
        idle = [r for r in fleet if not r.busy]
        pending = list(jobs)
        out: list[Assignment] = []

        # Sort idle robots by least loaded (already idle, just stable order).
        for robot in sorted(idle, key=lambda r: r.robot_id):
            if not pending:
                break
            best: Optional[tuple[float, Job, list[Reservation]]] = None
            for job in pending:
                cost = self._cost(robot, job, now_s)
                route = self._build_route(robot, job, now_s)
                if route is None:
                    continue
                # Check if reservation can be admitted (without committing yet).
                if self._can_admit(route):
                    if best is None or cost < best[0]:
                        best = (cost, job, route)
            if best is None:
                continue
            cost, job, route = best
            # Commit the route.
            self.table.admit_route(route)
            robot.busy = True
            out.append(Assignment(
                job=job, robot=robot,
                eta_s=cost,
                route=route,
            ))
            pending.remove(job)
        return out

    # ------------------------------------------------------------------
    def _cost(self, robot: RobotState, job: Job, now_s: float) -> float:
        dist = abs(job.target_arclength_m - robot.arclength_m)
        travel = dist / max(robot.nominal_speed_mps, 1e-3)
        wait = max(0.0, now_s - job.submitted_at_s)
        priority_bonus = job.priority * self.priority_time_weight_s
        return travel + wait * 0.1 - priority_bonus

    def _build_route(self,
                     robot: RobotState,
                     job: Job,
                     now_s: float) -> Optional[list[Reservation]]:
        segs = self.topology.segments_between(
            robot.arclength_m, job.target_arclength_m,
        )
        if not segs:
            return None
        v = max(robot.nominal_speed_mps, 1e-3)
        # Order segments in travel direction
        forward = job.target_arclength_m >= robot.arclength_m
        ordered = sorted(segs, key=lambda s: s.s_start, reverse=not forward)
        t = now_s
        route: list[Reservation] = []
        s_cursor = robot.arclength_m
        for seg in ordered:
            seg_dist = abs(seg.s_end - seg.s_start)
            t_enter = t
            t_exit = t + seg_dist / v + self.safety_buffer_s
            route.append(Reservation(
                robot_id=robot.robot_id,
                segment_id=seg.segment_id,
                t_enter=t_enter,
                t_exit=t_exit,
            ))
            t = t_exit
            s_cursor = seg.s_end if forward else seg.s_start
        return route

    def _can_admit(self, route: list[Reservation]) -> bool:
        # Try-and-rollback so we don't leak state.
        accepted: list[Reservation] = []
        for r in route:
            c = self.table.admit(r)
            if c is not None:
                for a in accepted:
                    self.table._by_segment[a.segment_id].remove(a)  # noqa: SLF001
                return False
            accepted.append(r)
        # Roll back the test admit.
        for a in accepted:
            self.table._by_segment[a.segment_id].remove(a)  # noqa: SLF001
        return True
