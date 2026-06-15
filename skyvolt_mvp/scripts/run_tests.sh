#!/usr/bin/env bash
# Run the pure-Python unit tests without a colcon build.
# PYTEST_DISABLE_PLUGIN_AUTOLOAD avoids ROS's launch_testing pytest plugins,
# which are incompatible with newer pytest and break test collection when the
# ROS environment is sourced.
set -euo pipefail

cd "$(dirname "$0")/.."

# -p hypothesis: autoload is off, so opt the Hypothesis plugin back in for the
# property-based tests in skyvolt_localization/test.
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  -p hypothesis \
  src/skyvolt_localization/test \
  src/skyvolt_fleet/test \
  src/skyvolt_eval/test \
  src/skyvolt_manipulator/test \
  src/skyvolt_vision/test \
  src/skyvolt_safety/test \
  "$@"
