"""Fleet-health Prometheus metrics tests (Phase 4)."""
from skyvolt_safety.metrics import (
    MetricsRegistry, FleetSnapshot, fleet_health_text,
)


def _samples(text: str) -> dict:
    """Parse 'name value' sample lines (ignoring # HELP/# TYPE) -> dict."""
    out = {}
    for line in text.splitlines():
        if not line or line.startswith("#"):
            continue
        name, value = line.rsplit(" ", 1)
        out[name] = value
    return out


def test_registry_emits_help_and_type():
    reg = MetricsRegistry()
    reg.gauge("x_metric", 5, "a description")
    text = reg.render()
    assert "# HELP x_metric a description" in text
    assert "# TYPE x_metric gauge" in text
    assert "x_metric 5" in text


def test_counter_type_is_emitted():
    reg = MetricsRegistry()
    reg.counter("jobs_total", 42, "done")
    assert "# TYPE jobs_total counter" in reg.render()


def test_bool_renders_as_one_or_zero():
    reg = MetricsRegistry()
    reg.gauge("flag_true", True)
    reg.gauge("flag_false", False)
    s = _samples(reg.render())
    assert s["flag_true"] == "1"
    assert s["flag_false"] == "0"


def test_labels_are_rendered():
    reg = MetricsRegistry()
    reg.gauge("robot_busy", 1, labels={"robot": "r0"})
    assert 'robot_busy{robot="r0"} 1' in reg.render()


def test_fleet_health_text_carries_the_snapshot():
    snap = FleetSnapshot(robots_total=3, robots_busy=2, jobs_queued=5,
                         jobs_completed=17, reservations_active=9,
                         estop_tripped=True, rail_load_n=1600.0,
                         track_tilt_deg=0.4)
    s = _samples(fleet_health_text(snap))
    assert s["skyvolt_robots_total"] == "3"
    assert s["skyvolt_robots_busy"] == "2"
    assert s["skyvolt_jobs_queued"] == "5"
    assert s["skyvolt_jobs_completed_total"] == "17"
    assert s["skyvolt_reservations_active"] == "9"
    assert s["skyvolt_estop_tripped"] == "1"
    assert s["skyvolt_rail_load_newtons"] == "1600.0"
    assert s["skyvolt_track_tilt_degrees"] == "0.4"


def test_every_metric_has_help_and_type():
    text = fleet_health_text(FleetSnapshot())
    names = {ln.split()[0] for ln in text.splitlines()
             if ln and not ln.startswith("#")}
    for name in names:
        assert f"# HELP {name} " in text
        assert f"# TYPE {name} " in text


def test_completed_is_a_counter_others_gauges():
    text = fleet_health_text(FleetSnapshot())
    assert "# TYPE skyvolt_jobs_completed_total counter" in text
    assert "# TYPE skyvolt_robots_total gauge" in text
