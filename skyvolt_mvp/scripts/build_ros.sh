#!/usr/bin/env bash
# Build the ROS 2 colcon workspace (all skyvolt_* packages).
#
# Note: plain `colcon build` is used (NOT --symlink-install): the newer
# setuptools in this environment rejects the `--editable` flag that
# --symlink-install passes to ament_python packages.
# Note: no `-u`; ROS's setup.bash references unset vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail

cd "$(dirname "$0")/.."

source /opt/ros/humble/setup.bash
colcon build "$@"

echo
echo "Build done. Source it with:  source install/setup.bash"
echo "Then run the sim with:       ./scripts/run_sim.sh"
