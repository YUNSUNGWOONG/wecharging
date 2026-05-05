"""SkyvoltRobot localization & docking speed policy.

The two pure-Python building blocks (`TrackLocalizer`, `SpeedPolicyFSM`) are
the actual IP and are the tested classes. The `*_node.py` files are thin ROS
wrappers.
"""
from .track_localizer import TrackLocalizer, RfidObservation, PhotoeyeObservation
from .speed_policy import SpeedPolicyFSM, DockingState, SpeedProfile

__all__ = [
    "TrackLocalizer",
    "RfidObservation",
    "PhotoeyeObservation",
    "SpeedPolicyFSM",
    "DockingState",
    "SpeedProfile",
]
