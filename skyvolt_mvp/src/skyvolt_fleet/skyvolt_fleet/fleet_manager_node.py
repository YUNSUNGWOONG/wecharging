"""ROS 2 wrapper for the Scheduler + ReservationTable.

This node:
- Maintains fleet state from /<robot_id>/track_state
- Receives jobs via the AssignJob service
- Publishes /<robot_id>/goal_arclength to drive each speed-policy node
- Publishes /fleet_status for dashboards
"""
from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64
    from skyvolt_msgs.msg import TrackState, FleetStatus
    from skyvolt_msgs.srv import AssignJob
    HAVE_ROS = True
except Exception:
    HAVE_ROS = False

from .reservation import ReservationTable
from .scheduler import (Scheduler, TrackTopology, Segment, Job, RobotState)


def _default_topology() -> TrackTopology:
    """One closed loop split into 8 segments of ~equal length.

    Total perimeter: 2 * 3.5 + 2 * pi * 0.8 ~= 12.03 m
    """
    perim = 2 * 3.5 + 2 * 3.14159 * 0.8
    n = 8
    seg_len = perim / n
    return TrackTopology(segments=[
        Segment(segment_id=i, branch_id=0,
                s_start=i * seg_len,
                s_end=(i + 1) * seg_len)
        for i in range(n)
    ])


if HAVE_ROS:
    class FleetManagerNode(Node):
        def __init__(self) -> None:
            super().__init__("fleet_manager")
            topo = _default_topology()
            self._scheduler = Scheduler(topology=topo, table=ReservationTable())
            self._fleet: dict[str, RobotState] = {}
            self._jobs: list[Job] = []

            self._goal_pubs: dict[str, rclpy.publisher.Publisher] = {}
            self._status_pub = self.create_publisher(FleetStatus, "fleet_status", 10)

            self.create_service(AssignJob, "assign_job", self._on_assign_job)
            self.create_timer(0.1, self._tick)
            # Wildcard subscription handled via robot list (configured by launch).
            self.declare_parameter("robot_ids", ["r0"])
            for rid in self.get_parameter("robot_ids").value:
                self._wire_robot(str(rid))

        def _wire_robot(self, rid: str) -> None:
            self._fleet[rid] = RobotState(robot_id=rid, arclength_m=0.0)
            self._goal_pubs[rid] = self.create_publisher(
                Float64, f"/{rid}/goal_arclength", 10)
            self.create_subscription(
                TrackState, f"/{rid}/track_state",
                lambda m, _rid=rid: self._on_track(m, _rid), 10)

        def _on_track(self, msg: "TrackState", rid: str) -> None:
            if rid in self._fleet:
                self._fleet[rid].arclength_m = msg.arclength_m

        def _on_assign_job(self, req: "AssignJob.Request",
                           res: "AssignJob.Response") -> "AssignJob.Response":
            j = Job(
                job_id=req.job.job_id,
                target_arclength_m=req.job.target_arclength_m,
                priority=int(req.job.priority),
                submitted_at_s=self.get_clock().now().nanoseconds * 1e-9,
            )
            self._jobs.append(j)
            res.accepted = True
            res.assigned_robot_id = ""
            res.reason = "queued"
            return res

        def _tick(self) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            assignments = self._scheduler.assign(
                jobs=self._jobs, fleet=self._fleet.values(), now_s=now)
            for a in assignments:
                self._jobs = [j for j in self._jobs if j.job_id != a.job.job_id]
                m = Float64()
                m.data = float(a.job.target_arclength_m)
                self._goal_pubs[a.robot.robot_id].publish(m)

            status = FleetStatus()
            status.stamp = self.get_clock().now().to_msg()
            status.num_robots = len(self._fleet)
            status.num_pending_jobs = len(self._jobs)
            status.num_active_jobs = sum(1 for r in self._fleet.values() if r.busy)
            status.num_reservations = self._scheduler.table.num_reservations()
            status.avg_job_age_s = 0.0
            self._status_pub.publish(status)


def main(argv=None):
    if not HAVE_ROS:
        raise RuntimeError("rclpy not available; this node requires ROS 2.")
    rclpy.init(args=argv)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
