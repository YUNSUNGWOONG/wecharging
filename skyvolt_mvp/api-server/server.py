"""FastAPI front for the FleetManager.

Run standalone (sim/dev) or behind the ROS service inside the cluster.
This server speaks plain HTTP/JSON so non-ROS clients (mobile app, Grafana,
billing service) can submit jobs.

POST /jobs                 -> queue a charging job
GET  /jobs/{job_id}        -> status
GET  /fleet                -> current fleet state
"""
from __future__ import annotations

import time
import uuid
from typing import Optional

from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

from skyvolt_fleet.scheduler import Scheduler, Job, RobotState, TrackTopology, Segment
from skyvolt_fleet.reservation import ReservationTable


# --------------------------------------------------------------------------
# In-process state. In production this lives in the ROS node and persists.
_topo = TrackTopology(segments=[
    Segment(i, 0, i * 1.5, (i + 1) * 1.5) for i in range(8)
])
_scheduler = Scheduler(topology=_topo, table=ReservationTable())
_fleet: dict[str, RobotState] = {
    "r0": RobotState("r0", arclength_m=0.0),
    "r1": RobotState("r1", arclength_m=4.0),
    "r2": RobotState("r2", arclength_m=8.0),
}
_jobs: dict[str, dict] = {}


# --------------------------------------------------------------------------
class JobIn(BaseModel):
    target_arclength_m: float
    priority: int = 0
    deadline_s: float = Field(default=0.0, description="0 = no deadline")
    requester_id: str = ""


class JobOut(BaseModel):
    job_id: str
    state: str
    assigned_robot_id: Optional[str] = None
    eta_s: Optional[float] = None


app = FastAPI(title="SkyvoltRobot Fleet API", version="0.1.0")


@app.post("/jobs", response_model=JobOut)
def submit_job(body: JobIn) -> JobOut:
    job_id = str(uuid.uuid4())
    j = Job(
        job_id=job_id,
        target_arclength_m=body.target_arclength_m,
        priority=body.priority,
        submitted_at_s=time.time(),
    )
    _jobs[job_id] = {"job": j, "state": "queued",
                     "assigned_robot_id": None, "eta_s": None}
    # Drive one assignment pass.
    pending = [v["job"] for v in _jobs.values() if v["state"] == "queued"]
    assignments = _scheduler.assign(pending, _fleet.values(), time.time())
    for a in assignments:
        _jobs[a.job.job_id]["state"] = "assigned"
        _jobs[a.job.job_id]["assigned_robot_id"] = a.robot.robot_id
        _jobs[a.job.job_id]["eta_s"] = a.eta_s
    rec = _jobs[job_id]
    return JobOut(job_id=job_id, state=rec["state"],
                  assigned_robot_id=rec["assigned_robot_id"],
                  eta_s=rec["eta_s"])


@app.get("/jobs/{job_id}", response_model=JobOut)
def get_job(job_id: str) -> JobOut:
    rec = _jobs.get(job_id)
    if rec is None:
        raise HTTPException(404, "job not found")
    return JobOut(job_id=job_id, state=rec["state"],
                  assigned_robot_id=rec["assigned_robot_id"],
                  eta_s=rec["eta_s"])


@app.get("/fleet")
def get_fleet() -> dict:
    return {
        "track_length_m": max((s.s_end for s in _topo.segments), default=0.0),
        "robots": [
            {"robot_id": r.robot_id,
             "arclength_m": r.arclength_m,
             "busy": r.busy}
            for r in _fleet.values()
        ],
        "reservations": _scheduler.table.num_reservations(),
        "queued_jobs": sum(1 for v in _jobs.values() if v["state"] == "queued"),
    }


@app.get("/", response_class=HTMLResponse)
def dashboard() -> str:
    """Read-only operator dashboard: live fleet view, no controls."""
    return _DASHBOARD_HTML


_DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>SkyvoltRobot Fleet — Operator View</title>
<style>
  :root { --bg:#0f1722; --panel:#172230; --line:#26354a; --txt:#e6edf5;
          --muted:#8aa0b8; --blue:#2974c7; --busy:#e08539; --idle:#3fb27f; }
  * { box-sizing:border-box; }
  body { margin:0; font-family:system-ui,Segoe UI,Roboto,sans-serif;
         background:var(--bg); color:var(--txt); }
  header { display:flex; align-items:center; gap:12px; padding:16px 24px;
           border-bottom:1px solid var(--line); }
  header h1 { font-size:18px; margin:0; font-weight:600; }
  .badge { font-size:11px; padding:3px 8px; border-radius:10px;
           background:#33405333; color:var(--muted); border:1px solid var(--line); }
  .spacer { flex:1; }
  #updated { font-size:12px; color:var(--muted); }
  main { padding:24px; max-width:1000px; margin:0 auto; }
  .cards { display:grid; grid-template-columns:repeat(4,1fr); gap:12px; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:10px;
          padding:14px 16px; }
  .card .k { font-size:12px; color:var(--muted); }
  .card .v { font-size:26px; font-weight:700; margin-top:4px; }
  .panel { background:var(--panel); border:1px solid var(--line); border-radius:10px;
           padding:18px; margin-top:20px; }
  .panel h2 { font-size:13px; color:var(--muted); margin:0 0 14px;
              text-transform:uppercase; letter-spacing:.04em; }
  svg { width:100%; height:120px; display:block; }
  table { width:100%; border-collapse:collapse; font-size:14px; }
  th,td { text-align:left; padding:8px 10px; border-bottom:1px solid var(--line); }
  th { color:var(--muted); font-weight:500; font-size:12px; }
  .dot { display:inline-block; width:9px; height:9px; border-radius:50%; margin-right:7px; }
  .err { color:var(--busy); }
</style>
</head>
<body>
<header>
  <h1>SkyvoltRobot Fleet</h1>
  <span class="badge">OPERATOR VIEW · READ-ONLY</span>
  <span class="spacer"></span>
  <span id="updated">connecting…</span>
</header>
<main>
  <div class="cards">
    <div class="card"><div class="k">Robots</div><div class="v" id="c-robots">–</div></div>
    <div class="card"><div class="k">Busy</div><div class="v" id="c-busy">–</div></div>
    <div class="card"><div class="k">Reservations</div><div class="v" id="c-res">–</div></div>
    <div class="card"><div class="k">Queued jobs</div><div class="v" id="c-queue">–</div></div>
  </div>
  <div class="panel">
    <h2>Track</h2>
    <svg id="track" viewBox="0 0 1000 120" preserveAspectRatio="none"></svg>
  </div>
  <div class="panel">
    <h2>Robots</h2>
    <table><thead><tr><th>Robot</th><th>Arclength (m)</th><th>State</th></tr></thead>
      <tbody id="rows"></tbody></table>
  </div>
</main>
<script>
const IDLE = "#3fb27f", BUSY = "#e08539";
function draw(d) {
  const L = d.track_length_m || 1, W = 1000, pad = 30, y = 60;
  const x = s => pad + (s / L) * (W - 2 * pad);
  let svg = `<line x1="${pad}" y1="${y}" x2="${W-pad}" y2="${y}" stroke="#26354a" stroke-width="4"/>`;
  for (let m = 0; m <= L; m += Math.max(1, Math.round(L/12))) {
    svg += `<line x1="${x(m)}" y1="${y-6}" x2="${x(m)}" y2="${y+6}" stroke="#26354a"/>`;
    svg += `<text x="${x(m)}" y="${y+24}" fill="#8aa0b8" font-size="11" text-anchor="middle">${m}</text>`;
  }
  for (const r of d.robots) {
    const c = r.busy ? BUSY : IDLE;
    svg += `<circle cx="${x(r.arclength_m)}" cy="${y}" r="9" fill="${c}" stroke="#0f1722" stroke-width="2"/>`;
    svg += `<text x="${x(r.arclength_m)}" y="${y-16}" fill="#e6edf5" font-size="11" text-anchor="middle">${r.robot_id}</text>`;
  }
  document.getElementById("track").innerHTML = svg;
  document.getElementById("c-robots").textContent = d.robots.length;
  document.getElementById("c-busy").textContent = d.robots.filter(r => r.busy).length;
  document.getElementById("c-res").textContent = d.reservations;
  document.getElementById("c-queue").textContent = d.queued_jobs;
  document.getElementById("rows").innerHTML = d.robots.map(r =>
    `<tr><td>${r.robot_id}</td><td>${r.arclength_m.toFixed(2)}</td>` +
    `<td><span class="dot" style="background:${r.busy?BUSY:IDLE}"></span>${r.busy?"busy":"idle"}</td></tr>`
  ).join("");
}
async function tick() {
  try {
    const d = await (await fetch("/fleet")).json();
    draw(d);
    document.getElementById("updated").textContent =
      "updated " + new Date().toLocaleTimeString();
    document.getElementById("updated").classList.remove("err");
  } catch (e) {
    const u = document.getElementById("updated");
    u.textContent = "disconnected"; u.classList.add("err");
  }
}
tick(); setInterval(tick, 1000);
</script>
</body>
</html>"""


def main() -> None:  # pragma: no cover
    import os
    import uvicorn
    port = int(os.environ.get("PORT", "8080"))
    uvicorn.run(app, host="0.0.0.0", port=port)


if __name__ == "__main__":  # pragma: no cover
    main()
