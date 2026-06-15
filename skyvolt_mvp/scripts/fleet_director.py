#!/usr/bin/env python3
"""Closed-loop fleet director for the Gazebo sim (Phase 2).

Drives the 3+ Gazebo robots with the *same* scheduler / reservation logic that
the pure-Python harness tests, closing the loop:

    generate jobs -> Scheduler.assign -> command cmd_vel toward target ->
    robot moves on the rail (VelocityControl) -> on arrival release the segment
    reservation and re-assign.

Robot motion is kinematic: arclength s maps linearly to world-x on the north
rail, and a model is moved by publishing ignition.msgs.Twist to
/model/<id>/cmd_vel (one publish to start, one to stop — Gazebo integrates the
constant velocity in between, so the director's internal arclength stays in
sync with the sim).

Run it after the sim is up:
    ./scripts/run_sim.sh fleet num_robots:=3      # terminal 1
    python3 scripts/fleet_director.py --robots 3  # terminal 2
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "skyvolt_fleet"))

from skyvolt_fleet.scheduler import (   # noqa: E402
    Scheduler, TrackTopology, Segment, Job, RobotState,
)
from skyvolt_fleet.reservation import ReservationTable  # noqa: E402

# North-rail straight: arclength s in [0, RAIL_LEN] -> world x in [X_MIN, X_MAX].
X_MIN, X_MAX = -1.75, 1.75
RAIL_LEN = X_MAX - X_MIN
RAIL_Y, RAIL_Z = 0.8, 2.0


def s_to_x(s: float) -> float:
    return X_MIN + s


def send_cmd_vel(robot_id: str, vx: float) -> None:
    """Publish a one-shot velocity command to a Gazebo model (kinematic)."""
    subprocess.run(
        ["ign", "topic", "-t", f"/model/{robot_id}/cmd_vel",
         "-m", "ignition.msgs.Twist", "-p", f"linear: {{x: {vx}}}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)


def set_pose(robot_id: str, x: float) -> None:
    """Snap a model onto the rail at world-x (corrects slow physics z-drift)."""
    subprocess.run(
        ["ign", "service", "-s", "/world/parking_lot/set_pose",
         "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
         "--timeout", "400",
         "--req", f'name: "{robot_id}" position: {{x: {x}, y: {RAIL_Y}, z: {RAIL_Z}}}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="Gazebo closed-loop fleet director")
    p.add_argument("--robots", type=int, default=3)
    p.add_argument("--speed", type=float, default=0.5, help="rail speed m/s")
    p.add_argument("--job-interval", type=float, default=2.5,
                   help="seconds between new jobs")
    p.add_argument("--duration", type=float, default=0.0,
                   help="stop after N seconds (0 = run forever)")
    p.add_argument("--seed", type=int, default=1)
    args = p.parse_args(argv)

    import random
    rng = random.Random(args.seed)

    n_seg = 8
    topo = TrackTopology(segments=[
        Segment(i, 0, i * RAIL_LEN / n_seg, (i + 1) * RAIL_LEN / n_seg)
        for i in range(n_seg)])
    table = ReservationTable()
    sch = Scheduler(topology=topo, table=table)

    # Robots start spread along the rail (matching the launch spawn spacing).
    fleet = [RobotState(f"r{i}", arclength_m=0.25 + i * 0.8,
                        nominal_speed_mps=args.speed)
             for i in range(args.robots)]
    arrive_at: dict[str, float] = {}
    target_of: dict[str, float] = {}
    # Per-robot motion so we can track the believed arclength mid-travel.
    motion: dict[str, tuple] = {}   # id -> (start_s, start_t, vx)

    def believed_s(r: RobotState) -> float:
        m = motion.get(r.robot_id)
        if not m:
            return r.arclength_m
        s0, st, vx = m
        s = s0 + vx * (now - st)
        return max(s, target_of[r.robot_id]) if vx < 0 else min(s, target_of[r.robot_id])

    jobs: list[Job] = []
    job_n = 0
    t0 = time.time()
    now = 0.0
    next_job = 0.0
    next_zfix = 0.0
    n_done = 0
    print(f"[director] driving {args.robots} robots; Ctrl+C to stop")

    try:
        while True:
            now = time.time() - t0
            if args.duration and now >= args.duration:
                break

            # 1) Retire arrived robots: stop, snap arclength, free the route.
            for r in fleet:
                if r.busy and now >= arrive_at.get(r.robot_id, 1e9):
                    r.arclength_m = target_of[r.robot_id]
                    send_cmd_vel(r.robot_id, 0.0)
                    motion.pop(r.robot_id, None)
                    table.release_robot(r.robot_id)
                    r.busy = False
                    n_done += 1
                    print(f"[{now:6.1f}s] {r.robot_id} arrived @ s={r.arclength_m:.2f} "
                          f"(done {n_done})")

            # 2) Spawn a new charging job periodically.
            if now >= next_job:
                target = rng.uniform(0.2, RAIL_LEN - 0.2)
                jobs.append(Job(job_id=f"j{job_n}", target_arclength_m=target,
                                priority=rng.randint(0, 2), submitted_at_s=now))
                job_n += 1
                next_job = now + args.job_interval

            # 3) Assign idle robots and start them moving.
            out = sch.assign(jobs, fleet, now)
            for a in out:
                jobs.remove(a.job)
                s0, s1 = a.robot.arclength_m, a.job.target_arclength_m
                vx = args.speed if s1 >= s0 else -args.speed
                dur = abs(s1 - s0) / args.speed
                target_of[a.robot.robot_id] = s1
                arrive_at[a.robot.robot_id] = now + dur
                motion[a.robot.robot_id] = (s0, now, vx)
                send_cmd_vel(a.robot.robot_id, vx)
                print(f"[{now:6.1f}s] {a.robot.robot_id} -> job {a.job.job_id} "
                      f"s={s1:.2f} (eta {dur:.1f}s)")

            # 4) Periodically snap every robot back onto the rail (counters the
            # slow physics z-drift of the suspended kinematic model).
            if now >= next_zfix:
                for r in fleet:
                    set_pose(r.robot_id, s_to_x(believed_s(r)))
                next_zfix = now + 1.5

            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for r in fleet:                      # stop everyone on exit
            send_cmd_vel(r.robot_id, 0.0)
        print(f"[director] stopped. jobs completed: {n_done}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
