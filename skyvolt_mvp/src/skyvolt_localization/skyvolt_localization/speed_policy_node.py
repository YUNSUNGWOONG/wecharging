"""Thin ROS 2 wrapper around SpeedPolicyFSM.

Subscribes:
    /<robot_id>/track_state    skyvolt_msgs/TrackState
    /<robot_id>/rfid           skyvolt_msgs/RfidDetection
    /<robot_id>/photoeye       skyvolt_msgs/PhotoeyeDetection
    /<robot_id>/goal_arclength std_msgs/Float64
Publishes:
    /<robot_id>/cmd_arclength_vel  geometry_msgs/Twist
    /<robot_id>/docking_state      skyvolt_msgs/DockingState
"""
from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64
    from geometry_msgs.msg import Twist
    from skyvolt_msgs.msg import (
        TrackState, RfidDetection, PhotoeyeDetection, DockingState as DockingStateMsg,
    )
    HAVE_ROS = True
except Exception:
    HAVE_ROS = False

from .speed_policy import SpeedPolicyFSM, DockingState


if HAVE_ROS:
    class SpeedPolicyNode(Node):
        def __init__(self) -> None:
            super().__init__("speed_policy")
            self.declare_parameter("robot_id", "r0")
            self.robot_id = self.get_parameter("robot_id").value
            self.fsm = SpeedPolicyFSM()
            self._latest_track: TrackState | None = None
            self._on_cue = False
            self._on_pos = False
            self._photo = False
            self._last_t = self.get_clock().now()

            self.create_subscription(TrackState, "track_state",
                                     self._track_cb, 50)
            self.create_subscription(RfidDetection, "rfid", self._rfid_cb, 50)
            self.create_subscription(PhotoeyeDetection, "photoeye",
                                     self._photo_cb, 50)
            self.create_subscription(Float64, "goal_arclength",
                                     self._goal_cb, 10)
            self._cmd_pub = self.create_publisher(Twist, "cmd_arclength_vel", 10)
            self._state_pub = self.create_publisher(DockingStateMsg, "docking_state", 10)
            self.create_timer(0.02, self._tick)

        def _track_cb(self, m): self._latest_track = m
        def _rfid_cb(self, m):
            if int(m.kind) == 1:
                self._on_cue = True
            elif int(m.kind) == 2:
                self._on_pos = True
        def _photo_cb(self, m):
            self._photo = bool(m.triggered)
        def _goal_cb(self, m: "Float64"):
            self.fsm.set_goal(target_arclength=float(m.data))

        def _tick(self) -> None:
            if self._latest_track is None:
                return
            now = self.get_clock().now()
            dt = (now - self._last_t).nanoseconds * 1e-9
            self._last_t = now

            v = self.fsm.step(
                s_hat=self._latest_track.arclength_m,
                ds_hat=self._latest_track.speed_mps,
                on_cue=self._on_cue,
                on_pos=self._on_pos,
                photoeye=self._photo,
                dt=dt,
                on_curve=False,  # sim plugin will set this in Phase 2
            )
            # Latch each card to a single edge.
            self._on_cue = False
            self._on_pos = False
            self._photo = False

            tw = Twist()
            tw.linear.x = float(v)
            self._cmd_pub.publish(tw)

            ds = DockingStateMsg()
            ds.stamp = now.to_msg()
            ds.robot_id = str(self.robot_id)
            ds.state = int(self.fsm.state)
            ds.lateral_error_mm = -1.0
            ds.longitudinal_error_mm = -1.0
            self._state_pub.publish(ds)


def main(argv=None):
    if not HAVE_ROS:
        raise RuntimeError("rclpy not available; this node requires ROS 2.")
    rclpy.init(args=argv)
    node = SpeedPolicyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
