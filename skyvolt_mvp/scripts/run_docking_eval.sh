#!/usr/bin/env bash
# One-shot regression run. Use in CI.
set -euo pipefail

cd "$(dirname "$0")/.."

# Make algorithm packages importable without colcon build.
export PYTHONPATH="$(pwd)/src/skyvolt_localization:$(pwd)/src/skyvolt_fleet:$(pwd)/src/skyvolt_eval:${PYTHONPATH:-}"

python3 -m skyvolt_eval.docking_eval --trials "${1:-100}" --seed "${2:-1}"
