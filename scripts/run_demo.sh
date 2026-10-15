#!/usr/bin/env bash
set -e
source "$(dirname "$0")/environment.sh"
IRA1_RUN_DIR="$IRA1_ROOT/data/run_$(date +%Y%m%d_%H%M%S)"
ros2 run ira1_sim record --ros-args -p "output_dir:=$IRA1_RUN_DIR" &
IRA1_RECORDER_PID=$!
trap 'kill -INT "$IRA1_RECORDER_PID" 2>/dev/null || true' EXIT
ros2 run ira1_sim drive
