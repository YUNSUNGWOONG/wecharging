#!/usr/bin/env bash
# Start the SkyvoltRobot Fleet API server.
#
# This directory is self-contained: the `skyvolt_fleet` package sits right next
# to server.py, so no PYTHONPATH juggling or colcon build is needed.
#
# Usage:
#   ./run.sh                 # start on the default port (8080)
#   PORT=9000 ./run.sh       # start on a custom port
set -euo pipefail

cd "$(dirname "$0")"

PORT="${PORT:-8080}" python3 server.py
