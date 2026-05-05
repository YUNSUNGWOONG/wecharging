"""Thin ROS 2 wrapper around TrackLocalizer.

Subscribes:
    /<robot_id>/odom            nav_msgs/Odometry      (provides linear vel)
    /<robot_id>/rfid            skyvolt_msgs/RfidDetection
    /<robot_id>/photoeye        skyvolt_msgs/PhotoeyeDetection
Publishes:
    /<robot_id>/track_state     skyvolt_msgs/TrackState   @ 50 Hz
"""
from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from skyvolt_msgs.msg import (RfidDetection, PhotoeyeDetection, TrackState)
    HAVE_ROS = True
except Exception:
    HAVE_ROS = False

from .track_localizer import (TrackLocalizer, RfidObservation,
                              PhotoeyeObservation)


if HAVE_ROS:
    class LocalizerNode(Node):
        def __init__(self) -> None:
            super().__init__("localizer")
            self.declare_parameter("robot_id", "r0")
            self.robot_id = self.get_parameter("robot_id").value
            self.loc = TrackLocalizer()
            self._last_t = self.get_clock().now()

            self.create_subscription(Odometry, "odom", self._odom_cb, 50)
            self.create_subscription(RfidDetection, "rfid", self._rfid_cb, 50)
            self.create_subscription(PhotoeyeDetection, "photoeye",
                                     self._photo_cb, 50)
            self._pub = self.create_publisher(TrackState, "track_state", 10)
            self.create_timer(0.02, self._tick)

        def _tick(self) -> None:
            now = self.get_clock().now()
            dt = (now - self._last_t).nanoseconds * 1e-9
            self._last_t = now
            self.loc.predict(dt)
            s, ds = self.loc.state
            msg = TrackState()
            msg.stamp = now.to_msg()
            msg.robot_id = str(self.robot_id)
            msg.branch_id = 0
            msg.arclength_m = s
            msg.speed_mps = ds
            msg.arclength_var = float(self.loc.covariance[0, 0])
            self._pub.publish(msg)

        def _odom_cb(self, m: "Odometry") -> None:
            self.loc.update_odom(m.twist.twist.linear.x)

        def _rfid_cb(self, m: "RfidDetection") -> None:
            kind = {1: "cue", 2: "positioning", 3: "docking"}.get(int(m.kind), "cue")
            self.loc.update_rfid(RfidObservation(
                tag_arclength_m=m.expected_arclength_m,
                kind=kind, tag_id=m.tag_id,
            ))

        def _photo_cb(self, m: "PhotoeyeDetection") -> None:
            if m.triggered:
                # Photoeye position is implicit from the docking-card map; in
                # this MVP the sim publishes the arclength via expected. In a
                # real system this comes from a config-time tag map.
                self.loc.update_photoeye(PhotoeyeObservation(
                    docking_arclength_m=self.loc.state[0],  # snap to current
                    triggered=True,
                    docking_card_id=m.docking_card_id,
                ))


def main(argv=None):
    if not HAVE_ROS:
        raise RuntimeError("rclpy not available; this node requires ROS 2.")
    rclpy.init(args=argv)
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
