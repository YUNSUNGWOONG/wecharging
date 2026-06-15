"""Smoke tests for the read-only operator dashboard + /fleet payload.

Calls the route handlers directly (no HTTP client needed). Skipped where
FastAPI isn't installed — the algorithm tests don't depend on it."""
import pytest

pytest.importorskip("fastapi")
from api.server import dashboard, get_fleet, _DASHBOARD_HTML  # noqa: E402


def test_dashboard_page_is_html():
    html = dashboard()
    assert html is _DASHBOARD_HTML
    assert html.lstrip().startswith("<!DOCTYPE html>")
    assert "SkyvoltRobot Fleet" in html
    assert "READ-ONLY" in html


def test_dashboard_is_read_only():
    """Read-only first: the page exposes no job-submission controls."""
    html = dashboard().lower()
    assert "<form" not in html
    assert "<button" not in html


def test_dashboard_polls_the_fleet_endpoint():
    assert 'fetch("/fleet")' in dashboard()


def test_fleet_payload_drives_the_dashboard():
    d = get_fleet()
    assert d["track_length_m"] > 0
    assert isinstance(d["robots"], list) and d["robots"]
    for r in d["robots"]:
        assert {"robot_id", "arclength_m", "busy"} <= r.keys()
    assert "reservations" in d and "queued_jobs" in d
