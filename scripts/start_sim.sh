#!/usr/bin/env bash
set -e
source "$(dirname "$0")/environment.sh"
exec ros2 launch ira1_sim sim.launch.py
