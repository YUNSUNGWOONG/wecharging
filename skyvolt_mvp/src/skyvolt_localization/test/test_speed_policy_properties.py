"""Property-based fault-injection tests for the docking speed-policy FSM.

Hypothesis drives the FSM with random tick sequences (estimates, sensor flags,
dt) and asserts the safety invariants that must hold on every tick: bounded
command velocity, jerk-bounded ramps in normal operation, forward-only state
progression, and terminal-state stickiness.
"""
from __future__ import annotations

from hypothesis import given, settings, strategies as st

from skyvolt_localization.speed_policy import (
    SpeedPolicyFSM, SpeedProfile, DockingState,
)

finite = lambda lo, hi: st.floats(  # noqa: E731
    min_value=lo, max_value=hi, allow_nan=False, allow_infinity=False)

# One tick of sensor/estimate inputs.
tick = st.tuples(
    finite(-5.0, 30.0),   # s_hat
    finite(-2.0, 3.0),    # ds_hat
    st.booleans(),        # on_cue
    st.booleans(),        # on_pos
    st.booleans(),        # photoeye
    finite(1e-3, 0.2),    # dt
    st.booleans(),        # on_curve
)
ticks = st.lists(tick, min_size=0, max_size=200)

# A plausible docking goal with mapped cue/pos cards.
goal = st.tuples(finite(5.0, 20.0), finite(1.0, 6.0), finite(2.0, 10.0))


def _fsm_with_goal(goal_tuple) -> SpeedPolicyFSM:
    target, cue, pos = goal_tuple
    fsm = SpeedPolicyFSM()
    # Keep cards ordered (cue before pos before target) like the real map.
    cue = min(cue, target)
    pos = min(max(pos, cue), target)
    fsm.set_goal(target_arclength=target, cue_arclength=cue, pos_arclength=pos)
    return fsm


@given(goal=goal, seq=ticks)
@settings(max_examples=400)
def test_command_velocity_within_bounds(goal, seq):
    """Commanded velocity is never negative and never exceeds top cruise speed."""
    fsm = _fsm_with_goal(goal)
    v_max = fsm.profile.cruise_straight_mps
    for t in seq:
        v = fsm.step(*t)
        assert -1e-9 <= v <= v_max + 1e-9


@given(goal=goal, seq=ticks)
@settings(max_examples=400)
def test_deceleration_is_jerk_bounded_except_estop(goal, seq):
    """In normal operation |Δv| <= max_decel * dt. The only allowed exception
    is the tick on which the FSM trips into FAULT (an intentional e-stop)."""
    fsm = _fsm_with_goal(goal)
    limit = fsm.profile.max_decel_mps2
    for t in seq:
        prev_v = fsm.last_cmd_v
        prev_state = fsm.state
        dt = t[5]
        v = fsm.step(*t)
        entered_fault = (fsm.state == DockingState.FAULT
                         and prev_state != DockingState.FAULT)
        if not entered_fault:
            assert abs(v - prev_v) <= limit * dt + 1e-9


@given(goal=goal, seq=ticks)
@settings(max_examples=400)
def test_state_progression_is_forward_only(goal, seq):
    """Excluding FAULT, the docking stage may only advance, never regress:
    CRUISE -> APPROACH_1 -> APPROACH_2 -> DOCKED."""
    order = {
        DockingState.CRUISE: 0,
        DockingState.APPROACH_1: 1,
        DockingState.APPROACH_2: 2,
        DockingState.DOCKED: 3,
    }
    fsm = _fsm_with_goal(goal)
    rank = order[fsm.state]
    for t in seq:
        fsm.step(*t)
        if fsm.state == DockingState.FAULT:
            break
        new_rank = order[fsm.state]
        assert new_rank >= rank, "docking stage regressed"
        rank = new_rank


@given(goal=goal, seq=ticks, tail=ticks)
@settings(max_examples=300)
def test_terminal_states_are_sticky(goal, seq, tail):
    """Once DOCKED or FAULT, the FSM stays there (no set_goal/reset) and keeps
    commanding a ramp toward zero — it never spontaneously drives again.

    Note: the command may still be high on entry (an e-stop from cruise decels
    smoothly), so the invariant is *monotonic non-increase toward 0*, not an
    absolute speed cap."""
    fsm = _fsm_with_goal(goal)
    for t in seq:
        fsm.step(*t)
    if fsm.state not in (DockingState.DOCKED, DockingState.FAULT):
        return  # precondition not reached for this example
    terminal = fsm.state
    for t in tail:
        prev_v = fsm.last_cmd_v
        v = fsm.step(*t)
        assert fsm.state == terminal, "left a terminal state without reset"
        assert v <= prev_v + 1e-9, "velocity rose again in a terminal state"
        assert v >= -1e-9


@given(goal=goal, seq=ticks)
@settings(max_examples=400)
def test_photoeye_forces_immediate_dock(goal, seq):
    """Whenever the photoeye fires while still moving, the FSM must latch DOCKED
    on that very tick (emergency-stop semantics)."""
    fsm = _fsm_with_goal(goal)
    moving = (DockingState.CRUISE, DockingState.APPROACH_1, DockingState.APPROACH_2)
    for t in seq:
        s_hat, ds_hat, on_cue, on_pos, photoeye, dt, on_curve = t
        was_moving = fsm.state in moving
        fsm.step(*t)
        if photoeye and was_moving:
            assert fsm.state == DockingState.DOCKED
