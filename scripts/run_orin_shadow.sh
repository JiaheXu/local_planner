#!/usr/bin/env bash
# Run on Orin. Only planner proposals are published; no automatic base enable.
set -euo pipefail
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd "${PACKAGE_ROOT}/../.." && pwd)"
BRINGUP="${WORKSPACE}/src/astribot_planner_bringup"
source "${BRINGUP}/environment.sh"
set +u
export AMENT_CURRENT_PREFIX="${WORKSPACE}/install/local_planner"
source "${AMENT_CURRENT_PREFIX}/share/local_planner/local_setup.bash"
unset AMENT_CURRENT_PREFIX
set -u
export ROS_DOMAIN_ID=25 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/astribot_ros/robot_system_ctrl/fastdds_udp.xml
export ROS_LOG_DIR="${WORKSPACE}/log/orin-shadow-ros"
mkdir -p "${ROS_LOG_DIR}"
exec 9>"${WORKSPACE}/planner.lock"
flock -n 9 || { echo 'An Orin planner already holds planner.lock' >&2; exit 1; }
exec ros2 launch local_planner astribot_orin_shadow.launch.py "bringup_dir:=${BRINGUP}"
