"""Fleet-health metrics in Prometheus exposition format (Phase 4).

Turns a snapshot of fleet + safety state into Prometheus text so a Grafana
board can chart fleet health (utilization, queue depth, reservations, e-stop,
rail load, track tilt). Dependency-free: emits the simple exposition format
directly, so no prometheus_client is required to test or render it. A ROS node
/ HTTP `/metrics` endpoint that calls render() is a thin wrapper on top.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class _Metric:
    name: str
    mtype: str          # "gauge" | "counter"
    help: str
    samples: list = field(default_factory=list)   # (labels dict, value)


class MetricsRegistry:
    """Minimal Prometheus exposition-format registry."""

    def __init__(self) -> None:
        self._metrics: dict[str, _Metric] = {}

    def gauge(self, name: str, value: float, help: str = "",
              labels: Optional[dict] = None) -> None:
        self._add(name, "gauge", help, value, labels)

    def counter(self, name: str, value: float, help: str = "",
                labels: Optional[dict] = None) -> None:
        self._add(name, "counter", help, value, labels)

    def _add(self, name, mtype, help, value, labels) -> None:
        m = self._metrics.get(name)
        if m is None:
            m = _Metric(name=name, mtype=mtype, help=help)
            self._metrics[name] = m
        m.samples.append((labels or {}, value))

    def render(self) -> str:
        lines: list[str] = []
        for m in self._metrics.values():
            if m.help:
                lines.append(f"# HELP {m.name} {m.help}")
            lines.append(f"# TYPE {m.name} {m.mtype}")
            for labels, value in m.samples:
                lbl = ""
                if labels:
                    lbl = "{" + ",".join(f'{k}="{v}"'
                                         for k, v in labels.items()) + "}"
                v = int(value) if isinstance(value, bool) else value
                lines.append(f"{m.name}{lbl} {v}")
        return "\n".join(lines) + "\n"


@dataclass
class FleetSnapshot:
    """A point-in-time view of fleet + safety state to publish as metrics."""
    robots_total: int = 0
    robots_busy: int = 0
    jobs_queued: int = 0
    jobs_completed: int = 0
    reservations_active: int = 0
    estop_tripped: bool = False
    rail_load_n: float = 0.0
    track_tilt_deg: float = 0.0


def fleet_health_text(snap: FleetSnapshot) -> str:
    """Render a fleet snapshot as Prometheus exposition text."""
    reg = MetricsRegistry()
    reg.gauge("skyvolt_robots_total", snap.robots_total, "Robots in the fleet")
    reg.gauge("skyvolt_robots_busy", snap.robots_busy, "Robots currently on a job")
    reg.gauge("skyvolt_jobs_queued", snap.jobs_queued, "Pending charging jobs")
    reg.counter("skyvolt_jobs_completed_total", snap.jobs_completed,
                "Charging jobs completed")
    reg.gauge("skyvolt_reservations_active", snap.reservations_active,
              "Active track-segment reservations")
    reg.gauge("skyvolt_estop_tripped", snap.estop_tripped,
              "E-stop chain tripped (1 = tripped)")
    reg.gauge("skyvolt_rail_load_newtons", snap.rail_load_n,
              "Total rail-support load (N)")
    reg.gauge("skyvolt_track_tilt_degrees", snap.track_tilt_deg,
              "Track tilt (deg)")
    return reg.render()
