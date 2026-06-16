"""HTTP /metrics endpoint + Grafana dashboard tests (Phase 4)."""
import json
import os
import urllib.request

import pytest

import skyvolt_safety
from skyvolt_safety.metrics import FleetSnapshot
from skyvolt_safety.metrics_server import MetricsServer

GRAFANA = os.path.join(os.path.dirname(skyvolt_safety.__file__), "..",
                       "grafana", "fleet_health.json")


def _get(port, path):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=3) as r:
        return r.status, r.read().decode("utf-8")


def test_metrics_endpoint_serves_live_snapshot():
    snap = FleetSnapshot(robots_total=3, robots_busy=2, jobs_completed=17,
                         estop_tripped=True)
    server = MetricsServer(lambda: snap, port=0).start()
    try:
        status, body = _get(server.port, "/metrics")
        assert status == 200
        assert "skyvolt_robots_busy 2" in body
        assert "skyvolt_jobs_completed_total 17" in body
        assert "skyvolt_estop_tripped 1" in body
    finally:
        server.stop()


def test_root_serves_html_dashboard():
    server = MetricsServer(lambda: FleetSnapshot(), port=0).start()
    try:
        status, body = _get(server.port, "/")
        assert status == 200
        assert "<!DOCTYPE html>" in body
        assert "Fleet Health" in body
        assert "/metrics" in body          # the page polls the endpoint
    finally:
        server.stop()


def test_unknown_path_is_404():
    server = MetricsServer(lambda: FleetSnapshot(), port=0).start()
    try:
        with pytest.raises(urllib.error.HTTPError) as exc:
            _get(server.port, "/nope")
        assert exc.value.code == 404
    finally:
        server.stop()


def test_provider_is_called_each_scrape():
    """Live data: each scrape reflects the latest snapshot."""
    state = {"n": 0}

    def provider():
        state["n"] += 1
        return FleetSnapshot(jobs_completed=state["n"])

    server = MetricsServer(provider, port=0).start()
    try:
        _, b1 = _get(server.port, "/metrics")
        _, b2 = _get(server.port, "/metrics")
        assert "skyvolt_jobs_completed_total 1" in b1
        assert "skyvolt_jobs_completed_total 2" in b2
    finally:
        server.stop()


def test_grafana_dashboard_is_valid_json():
    with open(GRAFANA) as f:
        dash = json.load(f)
    assert dash["title"] == "SkyvoltRobot Fleet Health"
    assert dash["panels"] and len(dash["panels"]) >= 5


def test_grafana_panels_reference_real_metrics():
    with open(GRAFANA) as f:
        dash = json.load(f)
    exprs = " ".join(t["expr"] for p in dash["panels"] for t in p["targets"])
    for metric in ("skyvolt_robots_busy", "skyvolt_jobs_completed_total",
                   "skyvolt_estop_tripped", "skyvolt_rail_load_newtons"):
        assert metric in exprs
