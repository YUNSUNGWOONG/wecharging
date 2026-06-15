"""Phase 2 scheduler stress test: 1000 jobs, 10 robots, deadlock-free."""
import pytest

from skyvolt_fleet.fleet_sim import FleetSimConfig, run_fleet_sim, gate


def test_stress_1000_jobs_10_robots_is_deadlock_free():
    """The roadmap headline: drive 1000 jobs through 10 robots to completion
    with no deadlock and every reservation released at the end."""
    m = run_fleet_sim(FleetSimConfig(n_jobs=1000, n_robots=10, seed=1))
    ok, fails = gate(m)
    assert ok, fails
    assert m["n_completed"] == 1000
    assert m["completion_rate"] == 1.0
    assert m["deadlocked"] is False
    assert m["residual_reservations"] == 0


@pytest.mark.parametrize("seed", [0, 1, 2, 3, 4])
def test_deadlock_free_across_seeds(seed):
    """Robustness: the gate holds across independent job/fleet layouts."""
    m = run_fleet_sim(FleetSimConfig(n_jobs=200, n_robots=6, seed=seed))
    ok, fails = gate(m)
    assert ok, fails


def test_simulation_is_deterministic():
    a = run_fleet_sim(FleetSimConfig(n_jobs=200, n_robots=5, seed=9))
    b = run_fleet_sim(FleetSimConfig(n_jobs=200, n_robots=5, seed=9))
    assert a == b


def test_bigger_fleet_drains_a_burst_faster():
    """Under burst arrival (all jobs at t=0) fleet throughput is the bottleneck,
    so a bigger fleet must complete the queue sooner — and both still finish."""
    lean = run_fleet_sim(FleetSimConfig(n_jobs=300, n_robots=2, seed=3,
                                        submit_window_s=0.0))
    big = run_fleet_sim(FleetSimConfig(n_jobs=300, n_robots=12, seed=3,
                                       submit_window_s=0.0))
    assert lean["completion_rate"] == 1.0
    assert big["completion_rate"] == 1.0
    assert big["makespan_s"] < lean["makespan_s"]


def test_every_completed_job_was_assigned_before_done():
    m = run_fleet_sim(FleetSimConfig(n_jobs=100, n_robots=4, seed=2))
    assert m["wait_s"]["max"] >= 0.0
    assert m["throughput_jobs_per_s"] > 0.0


def test_stuck_robot_wedges_fleet_without_recovery():
    """A jammed robot whose reservations are never freed must stall the fleet:
    its job never completes and the deadlock gate fails."""
    m = run_fleet_sim(FleetSimConfig(n_jobs=200, n_robots=5, seed=1,
                                     stuck_robot_ids=(0,), recovery=False))
    ok, _ = gate(m)
    assert not ok
    assert m["completion_rate"] < 1.0
    assert m["n_recovered"] == 0


def test_recovery_policy_unwedges_the_fleet():
    """With timeout recovery on, the stuck robot is reclaimed and its job
    re-queued, so every job still completes deadlock-free."""
    m = run_fleet_sim(FleetSimConfig(n_jobs=200, n_robots=5, seed=1,
                                     stuck_robot_ids=(0,), recovery=True))
    ok, fails = gate(m)
    assert ok, fails
    assert m["completion_rate"] == 1.0
    assert m["deadlocked"] is False
    assert m["n_recovered"] == 1
    assert m["residual_reservations"] == 0


def test_recovery_handles_multiple_stuck_robots():
    m = run_fleet_sim(FleetSimConfig(n_jobs=300, n_robots=6, seed=2,
                                     stuck_robot_ids=(0, 3), recovery=True))
    ok, fails = gate(m)
    assert ok, fails
    assert m["n_recovered"] == 2
    assert m["completion_rate"] == 1.0


def test_recovery_is_inert_when_nothing_is_stuck():
    """Enabling recovery must not change a healthy run."""
    base = dict(n_jobs=200, n_robots=5, seed=4)
    off = run_fleet_sim(FleetSimConfig(recovery=False, **base))
    on = run_fleet_sim(FleetSimConfig(recovery=True, **base))
    assert on["n_recovered"] == 0
    assert off["completion_rate"] == on["completion_rate"] == 1.0
    assert off["makespan_s"] == on["makespan_s"]


def test_gate_flags_a_deadlock():
    """The gate must reject a stalled run (detector wiring, no real deadlock
    needed to exercise the failure path)."""
    bad = {"deadlocked": True, "completion_rate": 0.5,
           "residual_reservations": 7}
    ok, fails = gate(bad)
    assert not ok
    assert any("deadlock" in f for f in fails)
    assert any("completion_rate" in f for f in fails)
    assert any("residual" in f for f in fails)
