"""E-stop chain tests (Phase 4)."""
import pytest

from skyvolt_safety.estop import EStopChain, EStopState, COMMS_SOURCE
from skyvolt_safety.load_monitor import LoadMonitor, SafetyAction


def _armed():
    """A chain that's been started with a fresh heartbeat."""
    c = EStopChain(heartbeat_timeout_s=1.0)
    c.heartbeat(0.0)
    return c


def test_starts_armed_with_power():
    c = _armed()
    assert c.state == EStopState.ARMED
    assert c.power_enabled
    assert not c.tripped


def test_any_source_trips_and_cuts_power():
    c = _armed()
    c.trip("hw_button")
    assert c.tripped
    assert not c.power_enabled
    assert "hw_button" in c.active_sources


def test_trip_latches_even_after_source_clears():
    c = _armed()
    c.set_source("fault", True)
    c.set_source("fault", False)        # source gone...
    assert c.tripped                    # ...but the trip is latched
    assert c.active_sources == frozenset()


def test_reset_refused_while_a_source_is_active():
    c = _armed()
    c.trip("hw_button")
    assert c.reset() is False           # still asserting -> refuse
    assert c.tripped


def test_reset_succeeds_once_all_sources_clear():
    c = _armed()
    c.trip("hw_button")
    c.set_source("hw_button", False)
    assert c.reset() is True
    assert c.power_enabled


def test_all_of_multiple_sources_must_clear_to_reset():
    c = _armed()
    c.trip("hw_button")
    c.trip("load_estop")
    c.set_source("hw_button", False)
    assert c.reset() is False           # load_estop still active
    c.set_source("load_estop", False)
    assert c.reset() is True


def test_heartbeat_watchdog_trips_on_staleness():
    c = EStopChain(heartbeat_timeout_s=1.0)
    c.heartbeat(0.0)
    c.poll(0.5)
    assert c.power_enabled               # still fresh
    c.poll(2.0)                          # > 1.0 s since last heartbeat
    assert c.tripped
    assert COMMS_SOURCE in c.active_sources


def test_fresh_heartbeat_clears_comms_source_for_reset():
    c = EStopChain(heartbeat_timeout_s=1.0)
    c.heartbeat(0.0)
    c.poll(2.0)                          # trip on staleness
    assert c.tripped
    c.heartbeat(2.1)                     # comms restored
    assert c.reset() is True             # now re-armable


def test_load_monitor_estop_feeds_the_chain():
    """The load/tilt monitor's ESTOP action drives the e-stop chain."""
    chain = _armed()
    mon = LoadMonitor(rated_load_n=1000.0)
    reading = mon.assess([900, 900])     # 1800 > 1000 -> ESTOP
    chain.set_source("load_estop", reading.action == SafetyAction.ESTOP)
    assert chain.tripped
