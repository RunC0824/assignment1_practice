#!/bin/bash
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
exec gz sim -r world.sdf
