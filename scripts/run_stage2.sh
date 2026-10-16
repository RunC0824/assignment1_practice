#!/usr/bin/env bash
set -e
source "$(dirname "$0")/environment.sh"
# Run only against a newly started simulation with the robot still at its origin.
mkdir -p "$IRA1_ROOT/data"
IRA1_RUN_DIR="$(mktemp -d "$IRA1_ROOT/data/stage2_$(date +%Y%m%d_%H%M%S)_XXXX")"
printf '%s\n' "$IRA1_RUN_DIR" > "$IRA1_ROOT/data/latest_stage2.txt"
printf 'Stage 2 output: %s\n' "$IRA1_RUN_DIR"
python3 "$IRA1_ROOT/scripts/live_stage2.py" "$IRA1_RUN_DIR" &
IRA1_VIEW_PID=$!
finalize_stage2() {
    kill "$IRA1_VIEW_PID" 2>/dev/null || true
    if [ -s "$IRA1_RUN_DIR/errors.csv" ]; then
        python3 "$IRA1_ROOT/scripts/summarize_stage2.py" "$IRA1_RUN_DIR"
    fi
}
trap finalize_stage2 EXIT
ros2 launch ira1_sim stage2.launch.py "output_dir:=$IRA1_RUN_DIR"
