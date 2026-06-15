#!/usr/bin/env bash
# Launch the Gazebo (Fortress) simulation with one SkyvoltRobot.
#
# Bundles the environment setup and the workarounds needed on this machine:
#   * sources ROS 2 Humble + the colcon workspace
#   * forces the NVIDIA EGL/GLX vendor so Gazebo's renderer does not fall back
#     to the Mesa dri2 path (which fails on this NVIDIA box and blanks the view)
#
# Prereq: build the workspace once with  ./scripts/build_ros.sh  (or `colcon build`).
#
# Usage:
#   ./scripts/run_sim.sh            # single robot
#   ./scripts/run_sim.sh fleet      # multi-robot fleet (fleet.launch.py)
# Note: no `-u`; ROS's setup.bash references unset vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail

cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash
source install/setup.bash

# Render on the NVIDIA GPU; avoids "libEGL: failed to create dri2 screen".
export DISPLAY="${DISPLAY:-:0}"
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __GLX_VENDOR_LIBRARY_NAME=nvidia

if [ "${1:-single}" = "fleet" ]; then
    ros2 launch skyvolt_sim fleet.launch.py "${@:2}"
else
    ros2 launch skyvolt_sim single_robot.launch.py "${@:2}"
fi
