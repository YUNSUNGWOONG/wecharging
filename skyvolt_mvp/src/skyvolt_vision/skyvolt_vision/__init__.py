"""SkyvoltRobot charging-port vision (Phase 3).

Pure-Python + numpy 6-DoF pose estimation, the tested IP for plug insertion.
Perception front-ends and the Gazebo RGB-D sensor are separate.
"""
from .port_pose import (
    Pose6DoF, PoseFit, ChargingPort, estimate_pose,
    approach_distance, lateral_offset,
)
from .servo import (
    VisualServo, ServoPhase, ServoCommand, ServoResult, simulate_insertion,
)
from .detector import (
    CameraIntrinsics, deproject, detect_socket_corners,
    detect_port_points, detect_port_pose,
)

__all__ = [
    "Pose6DoF",
    "PoseFit",
    "ChargingPort",
    "estimate_pose",
    "approach_distance",
    "lateral_offset",
    "VisualServo",
    "ServoPhase",
    "ServoCommand",
    "ServoResult",
    "simulate_insertion",
    "CameraIntrinsics",
    "deproject",
    "detect_socket_corners",
    "detect_port_points",
    "detect_port_pose",
]
