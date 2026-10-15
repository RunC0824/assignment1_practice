# Source this file before running any IRA1 ROS command.
IRA1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$IRA1_ROOT/install/setup.bash"
export ROS_LOG_DIR="$IRA1_ROOT/logs/ros"
export ROS_DOMAIN_ID=41
export GZ_PARTITION=ira1
export FASTRTPS_DEFAULT_PROFILES_FILE="$IRA1_ROOT/src/ira1_sim/config/dds_udp.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GSETTINGS_BACKEND=memory
