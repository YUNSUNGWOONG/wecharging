import pytest

from skyvolt_fleet.scheduler import (
    Scheduler, TrackTopology, Segment, Job, RobotState,
)
from skyvolt_fleet.reservation import ReservationTable


def _topo(n=8, seg_len=1.5):
    return TrackTopology(segments=[
        Segment(i, 0, i * seg_len, (i + 1) * seg_len) for i in range(n)
    ])


def test_single_robot_takes_single_job():
    sch = Scheduler(topology=_topo())
    fleet = [RobotState("r0", arclength_m=0.0)]
    jobs = [Job(job_id="j1", target_arclength_m=4.0, submitted_at_s=0.0)]
    out = sch.assign(jobs, fleet, now_s=0.0)
    assert len(out) == 1
    assert out[0].robot.robot_id == "r0"
    assert out[0].job.job_id == "j1"
    assert fleet[0].busy is True


def test_two_robots_two_jobs_no_self_conflict():
    sch = Scheduler(topology=_topo())
    fleet = [RobotState("r0", arclength_m=0.0),
             RobotState("r1", arclength_m=8.0)]
    jobs = [Job("j1", target_arclength_m=2.0),
            Job("j2", target_arclength_m=10.0)]
    out = sch.assign(jobs, fleet, now_s=0.0)
    assert len(out) == 2
    rids = {a.robot.robot_id for a in out}
    assert rids == {"r0", "r1"}


def test_priority_pulls_more_urgent_job_first():
    sch = Scheduler(topology=_topo(), priority_time_weight_s=10.0)
    fleet = [RobotState("r0", arclength_m=0.0)]
    # Closer job has lower priority; far job is urgent.
    jobs = [Job("near", target_arclength_m=1.0, priority=0),
            Job("far", target_arclength_m=10.0, priority=10)]
    out = sch.assign(jobs, fleet, now_s=0.0)
    assert out[0].job.job_id == "far"


def test_busy_robot_is_skipped():
    sch = Scheduler(topology=_topo())
    r = RobotState("r0", arclength_m=0.0, busy=True)
    out = sch.assign([Job("j1", target_arclength_m=4.0)], [r], now_s=0.0)
    assert out == []


def test_scale_10_robots_200_jobs_no_deadlock():
    sch = Scheduler(topology=_topo(n=20, seg_len=1.0))
    fleet = [RobotState(f"r{i}", arclength_m=i * 1.5) for i in range(10)]
    jobs = [Job(job_id=f"j{i}",
                target_arclength_m=(i * 0.4) % 19.0,
                priority=(i % 3))
            for i in range(200)]
    out = sch.assign(jobs, fleet, now_s=0.0)
    # All idle robots get exactly one job
    assert len(out) == 10
    # No two assignments give the same robot
    assert len({a.robot.robot_id for a in out}) == 10
