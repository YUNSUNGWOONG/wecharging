"""Tests for the offline replay tooling.

The headline guarantee — replay reproduces the live estimate *exactly* — is what
makes a recorded bag a faithful regression artifact.
"""
import json

import pytest

from skyvolt_eval.replay import (
    SensorTick, simulate_trace, replay, replay_metrics,
    write_trace, read_trace,
)


def test_replay_reproduces_live_estimate_bit_for_bit():
    """The localizer is deterministic: replaying the recorded inputs must yield
    the exact estimate the live run produced (stored as ref_s_hat)."""
    ticks = simulate_trace(seed=3, with_imu=False)
    est = replay(ticks)
    assert len(est) == len(ticks)
    for s_replay, t in zip(est, ticks):
        assert s_replay == t.ref_s_hat  # exact equality, same float ops


def test_replay_reproduces_live_estimate_with_imu():
    ticks = simulate_trace(seed=5, with_imu=True)
    est = replay(ticks)
    assert any(t.imu_accel is not None for t in ticks)
    for s_replay, t in zip(est, ticks):
        assert s_replay == t.ref_s_hat


def test_trace_file_round_trip(tmp_path):
    """write -> read -> replay is identical to replaying the in-memory trace."""
    ticks = simulate_trace(seed=7, with_imu=True)
    path = tmp_path / "trace.jsonl"
    write_trace(ticks, path)
    loaded = read_trace(path)
    assert len(loaded) == len(ticks)
    assert replay(loaded) == replay(ticks)
    # Metrics survive the round-trip too.
    assert replay_metrics(loaded) == replay_metrics(ticks)


def test_trace_json_is_compact_and_valid(tmp_path):
    """Each line is valid JSON and omits empty fields (no null spam)."""
    ticks = simulate_trace(seed=1)
    path = tmp_path / "t.jsonl"
    write_trace(ticks, path)
    for line in path.read_text().splitlines():
        obj = json.loads(line)
        assert "dt" in obj
        assert None not in obj.values()


def test_final_docking_estimate_is_tight():
    """At dock the photoeye anchors hard, so the final localization error is
    well under the longitudinal gate (12 mm)."""
    m = replay_metrics(simulate_trace(seed=2))
    assert m["final_loc_err_mm"] is not None
    assert m["final_loc_err_mm"] < 12.0


def test_metrics_handle_trace_without_ground_truth():
    """A trace with no gt_s yields a graceful, gt-free summary."""
    ticks = [SensorTick(dt=0.02, odom=1.0) for _ in range(10)]
    m = replay_metrics(ticks)
    assert m["n_ticks"] == 10
    assert m["n_with_gt"] == 0


def test_replay_respects_initial_state():
    """A non-zero initial arclength shifts the whole estimate trajectory."""
    ticks = [SensorTick(dt=0.02) for _ in range(5)]  # no measurements
    est = replay(ticks, initial_s=4.0, initial_ds=0.0)
    assert est[0] == pytest.approx(4.0, abs=1e-9)
