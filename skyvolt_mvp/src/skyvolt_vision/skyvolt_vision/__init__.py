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
]
