"""Closed-loop fleet scheduler stress test (Phase 2).

`Scheduler.assign()` hands out at most one job per idle robot per call, so
driving 1000 jobs through 10 robots is a time-stepped simulation:

    queue jobs -> assign idle robots -> robots travel (busy) -> on arrival they
    free their reservations and become idle -> repeat until every job is done.

This harness runs that loop event-by-event, watches for deadlock (eligible jobs
+ idle robots but no admissible route, with no in-flight work to unblock them),
and reports throughput / wait / makespan against a deadlock-free gate.

Why pure Python: the same reason as docking_eval — a scheduling regression must
fail a PR without needing Gazebo. Roadmap Phase 2 target: 1000 jobs, 10 robots,
deadlock-free.

Usage:
    python -m skyvolt_fleet.fleet_sim --jobs 1000 --robots 10 --seed 1
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np

from skyvolt_fleet.scheduler import (
    Scheduler, TrackTopology, Segment, Job, RobotState,
)
from skyvolt_fleet.reservation import ReservationTable

EPS = 1e-6


@dataclass
class FleetSimConfig:
    n_jobs: int = 1000
    n_robots: int = 10
    track_length_m: float = 30.0
    n_segments: int = 20
    speed_mps: float = 1.5
    submit_window_s: float = 600.0     # jobs arrive spread over this window
    seg_buffer_s: float = 0.5          # dwell added per segment (matches scheduler)
    max_iterations: int = 200_000      # runaway guard
    seed: int = 0
    # Fault injection + recovery (deadlock timeout/recovery policy).
    stuck_robot_ids: tuple[int, ...] = ()   # robots that jam on their 1st job
    recovery: bool = False                  # enable timeout-based reclaim
    recovery_grace_s: float = 2.0           # overdue tolerance before reclaim


@dataclass
class _JobRec:
    job: Job
    submit_s: float
    assign_s: Optional[float] = None
    done_s: Optional[float] = None
    robot_id: str = ""


def _build_topology(cfg: FleetSimConfig) -> TrackTopology:
    seg_len = cfg.track_length_m / cfg.n_segments
    return TrackTopology(segments=[
        Segment(i, 0, i * seg_len, (i + 1) * seg_len)
        for i in range(cfg.n_segments)
    ])


def run_fleet_sim(cfg: FleetSimConfig) -> dict:
    rng = np.random.default_rng(cfg.seed)
    topo = _build_topology(cfg)
    table = ReservationTable()
    sch = Scheduler(topology=topo, table=table, safety_buffer_s=cfg.seg_buffer_s)

    margin = cfg.track_length_m * 1e-3
    fleet = [RobotState(f"r{i}",
                        arclength_m=(i + 0.5) * cfg.track_length_m / cfg.n_robots,
                        nominal_speed_mps=cfg.speed_mps)
             for i in range(cfg.n_robots)]

    recs: list[_JobRec] = []
    for i in range(cfg.n_jobs):
        submit = float(rng.uniform(0.0, cfg.submit_window_s))
        target = float(rng.uniform(margin, cfg.track_length_m - margin))
        recs.append(_JobRec(
            job=Job(job_id=f"j{i}", target_arclength_m=target,
                    priority=int(rng.integers(0, 3)), submitted_at_s=submit),
            submit_s=submit))
    recs.sort(key=lambda r: r.submit_s)
    by_id = {r.job.job_id: r for r in recs}

    pending: list[_JobRec] = list(recs)         # not yet completed/assigned
    in_flight: dict[str, tuple[Job, float]] = {}  # robot_id -> (job, done_at)
    stuck_ids = {f"r{i}" for i in cfg.stuck_robot_ids}
    faulted: set[str] = set()
    INF = float("inf")
    now = 0.0
    n_done = 0
    n_recovered = 0
    deadlock = False
    iterations = 0
    peak_reservations = 0

    while n_done < cfg.n_jobs:
        iterations += 1
        if iterations > cfg.max_iterations:
            deadlock = True       # treated as a stall/timeout failure
            break

        # 1) Retire any robot that has arrived; free its track reservations.
        for robot in fleet:
            if robot.busy and in_flight.get(robot.robot_id):
                job, done_at = in_flight[robot.robot_id]
                if done_at <= now + EPS:
                    robot.busy = False
                    robot.arclength_m = job.target_arclength_m
                    table.release_robot(robot.robot_id)
                    by_id[job.job_id].done_s = done_at
                    del in_flight[robot.robot_id]
                    n_done += 1
        if n_done >= cfg.n_jobs:
            break

        # 1b) Timeout recovery: reclaim presumed-stuck robots (whole route
        # overdue) and re-queue their jobs so the fleet is never wedged.
        if cfg.recovery:
            for rid in table.reclaim_expired(now, cfg.recovery_grace_s):
                if rid in in_flight:
                    job, _ = in_flight.pop(rid)
                    rec = by_id[job.job_id]
                    rec.assign_s = None
                    rec.robot_id = ""
                    pending.append(rec)        # re-task to a healthy robot
                    faulted.add(rid)
                    n_recovered += 1

        # 2) Assign eligible (already-submitted) jobs to idle robots.
        eligible = [r for r in pending
                    if r.assign_s is None and r.submit_s <= now + EPS]
        out = sch.assign([r.job for r in eligible], fleet, now)
        for a in out:
            rec = by_id[a.job.job_id]
            rec.assign_s = now
            rec.robot_id = a.robot.robot_id
            travel = (abs(a.job.target_arclength_m - a.robot.arclength_m)
                      / max(a.robot.nominal_speed_mps, 1e-3))
            # A "stuck" robot jams: it never reaches its target (done_at = inf).
            done_at = (INF if a.robot.robot_id in stuck_ids
                       else now + travel + cfg.seg_buffer_s)
            in_flight[a.robot.robot_id] = (a.job, done_at)
            pending.remove(rec)
        peak_reservations = max(peak_reservations, table.num_reservations())

        # 3) Advance time to the next event (arrival, recovery, or submission).
        future = []
        for rid, (_, done) in in_flight.items():
            if done != INF:
                future.append(done)
            elif cfg.recovery:
                # Stuck robot: the next event is when its route times out.
                t_exit = max((r.t_exit for r in table.active_for(rid)),
                             default=now)
                future.append(t_exit + cfg.recovery_grace_s + EPS)
        future += [r.submit_s for r in pending
                   if r.assign_s is None and r.submit_s > now + EPS]
        if future:
            now = max(min(future), now + EPS)
        elif out:
            now += EPS            # progress was made this tick; keep going
        else:
            # No in-flight work, no future arrivals, yet jobs remain unassigned:
            # nothing will ever unblock them -> genuine deadlock/stall.
            deadlock = True
            break

    # ---- metrics ----
    done = [r for r in recs if r.done_s is not None]
    waits = [r.assign_s - r.submit_s for r in done if r.assign_s is not None]
    makespan = max((r.done_s for r in done), default=0.0)
    return {
        "config": {"n_jobs": cfg.n_jobs, "n_robots": cfg.n_robots,
                   "track_length_m": cfg.track_length_m,
                   "n_segments": cfg.n_segments, "seed": cfg.seed},
        "n_completed": len(done),
        "completion_rate": len(done) / max(cfg.n_jobs, 1),
        "deadlocked": deadlock,
        "n_stuck_injected": len(stuck_ids),
        "n_recovered": n_recovered,
        "recovery_enabled": cfg.recovery,
        "iterations": iterations,
        "makespan_s": float(makespan),
        "throughput_jobs_per_s": (len(done) / makespan) if makespan > 0 else 0.0,
        "wait_s": {
            "mean": float(np.mean(waits)) if waits else 0.0,
            "p99": float(np.percentile(waits, 99)) if waits else 0.0,
            "max": float(max(waits)) if waits else 0.0,
        },
        "peak_reservations": peak_reservations,
        "residual_reservations": table.num_reservations(),
    }


def gate(m: dict) -> tuple[bool, list[str]]:
    fails: list[str] = []
    if m["deadlocked"]:
        fails.append("deadlock/stall detected")
    if m["completion_rate"] < 1.0:
        fails.append(f"completion_rate {m['completion_rate']:.3f} < 1.0")
    # Every reservation must be freed once the fleet drains.
    if m["residual_reservations"] != 0:
        fails.append(f"residual_reservations {m['residual_reservations']} != 0")
    return (len(fails) == 0, fails)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="SkyvoltRobot fleet scheduler stress test")
    p.add_argument("--jobs", type=int, default=1000)
    p.add_argument("--robots", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--stuck", type=str, default="",
                   help="comma-separated robot indices to jam (e.g. 0,3)")
    p.add_argument("--recovery", action="store_true",
                   help="enable timeout-based reservation recovery")
    p.add_argument("--out", type=str, default="")
    args = p.parse_args(argv)

    stuck = tuple(int(x) for x in args.stuck.split(",") if x.strip() != "")
    cfg = FleetSimConfig(n_jobs=args.jobs, n_robots=args.robots, seed=args.seed,
                         stuck_robot_ids=stuck, recovery=args.recovery)
    m = run_fleet_sim(cfg)
    text = json.dumps(m, indent=2)
    if args.out:
        Path(args.out).write_text(text)
    print(text)
    ok, fails = gate(m)
    if not ok:
        print("FAIL:", "; ".join(fails), file=sys.stderr)
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
