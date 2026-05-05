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
        "robots": [
            {"robot_id": r.robot_id,
             "arclength_m": r.arclength_m,
             "busy": r.busy}
            for r in _fleet.values()
        ],
        "reservations": _scheduler.table.num_reservations(),
        "queued_jobs": sum(1 for v in _jobs.values() if v["state"] == "queued"),
    }


def main() -> None:  # pragma: no cover
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)


if __name__ == "__main__":  # pragma: no cover
    main()
