"""Unit tests for the two-stage deceleration FSM."""
import pytest

from skyvolt_localization.speed_policy import (
    SpeedPolicyFSM, DockingState, SpeedProfile,
)


def _run(fsm, sequence, dt=0.02):
    """sequence: iterable of dicts with keys s, ds, on_cue, on_pos, photo, on_curve."""
    log = []
    for ev in sequence:
        v = fsm.step(
            s_hat=ev.get("s", 0.0),
            ds_hat=ev.get("ds", 0.0),
            on_cue=ev.get("on_cue", False),
            on_pos=ev.get("on_pos", False),
            photoeye=ev.get("photo", False),
            dt=dt,
            on_curve=ev.get("on_curve", False),
        )
        log.append((fsm.state, v))
    return log


def test_idle_yields_zero_velocity():
    fsm = SpeedPolicyFSM()
    v = fsm.step(0.0, 0.0, False, False, False, dt=0.02)
    assert v == 0.0
    assert fsm.state == DockingState.IDLE


def test_full_sequence_reaches_docked():
    fsm = SpeedPolicyFSM()
    fsm.set_goal(target_arclength=10.0, cue_arclength=8.0, pos_arclength=9.0)

    # Cruise for 100 ticks
    for _ in range(100):
        fsm.step(s_hat=0.0, ds_hat=1.5,
                 on_cue=False, on_pos=False, photoeye=False, dt=0.02)
    assert fsm.state == DockingState.CRUISE

    # Cue card
    fsm.step(s_hat=8.0, ds_hat=1.5, on_cue=True, on_pos=False,
             photoeye=False, dt=0.02)
    assert fsm.state == DockingState.APPROACH_1

    # Positioning card
    fsm.step(s_hat=9.0, ds_hat=0.8, on_cue=False, on_pos=True,
             photoeye=False, dt=0.02)
    assert fsm.state == DockingState.APPROACH_2

    # Photoeye triggers stop
    fsm.step(s_hat=10.0, ds_hat=0.2, on_cue=False, on_pos=False,
             photoeye=True, dt=0.02)
    assert fsm.state == DockingState.DOCKED


def test_missed_cue_card_triggers_fault():
    fsm = SpeedPolicyFSM()
    fsm.set_goal(target_arclength=10.0, cue_arclength=8.0, pos_arclength=9.0)
    # Far past cue without on_cue=True
    fsm.step(s_hat=8.5, ds_hat=1.5, on_cue=False, on_pos=False,
             photoeye=False, dt=0.02)
    assert fsm.state == DockingState.FAULT
    assert "cue" in fsm.fault_reason


def test_overshoot_target_triggers_fault():
    fsm = SpeedPolicyFSM()
    fsm.set_goal(target_arclength=10.0)
    fsm.step(s_hat=10.4, ds_hat=0.2, on_cue=True, on_pos=True,
             photoeye=False, dt=0.02)
    assert fsm.state == DockingState.FAULT


def test_curve_uses_lower_cruise_speed():
    fsm = SpeedPolicyFSM(profile=SpeedProfile())
    fsm.set_goal(target_arclength=10.0)
    # Ramp long enough to hit target on a curve.
    for _ in range(200):
        fsm.step(s_hat=1.0, ds_hat=1.0,
                 on_cue=False, on_pos=False, photoeye=False,
                 dt=0.02, on_curve=True)
    # Should converge to 1.0 m/s, not 1.5
    assert fsm.last_cmd_v == pytest.approx(1.0, abs=0.05)


def test_photoeye_emergency_stop_from_cruise():
    """Photoeye triggers DOCKED even if we skipped cards (defensive)."""
    fsm = SpeedPolicyFSM()
    fsm.set_goal(target_arclength=10.0)
    fsm.step(s_hat=5.0, ds_hat=1.5, on_cue=False, on_pos=False,
             photoeye=True, dt=0.02)
    assert fsm.state == DockingState.DOCKED


def test_decel_ramp_respects_max_decel():
    fsm = SpeedPolicyFSM(profile=SpeedProfile(max_decel_mps2=1.0))
    fsm.set_goal(target_arclength=10.0)
    fsm.last_cmd_v = 1.5
    # One tick of dt=0.1 with the photoeye -> jump-to-zero target
    fsm.step(s_hat=9.99, ds_hat=0.2, on_cue=True, on_pos=True,
             photoeye=True, dt=0.1)
    assert fsm.last_cmd_v == pytest.approx(1.4, abs=1e-6)
