#!/usr/bin/env bash
# Manual Orin launch. Match basic/leased mode with the PC relay.
set -euo pipefail
BASIC_CONTROL=false
IGNORE_LIDAR=false
for option in "$@"; do
  case "${option}" in
    --basic-control) BASIC_CONTROL=true ;;
    --ignore-lidar) IGNORE_LIDAR=true ;;
    *) echo "Unknown option: ${option}" >&2; exit 2 ;;
  esac
done
if [[ ${IGNORE_LIDAR} != true ]]; then
  echo "Usage: $0 --ignore-lidar [--basic-control]" >&2
  exit 2
fi
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
export PLANNER_TRANSPORT_DIR="${BRINGUP}"
export ROS_LOG_DIR="${WORKSPACE}/log/orin-dispatch-ros"
mkdir -p "${ROS_LOG_DIR}"
exec 9>"${WORKSPACE}/planner.lock"
flock -n 9 || { echo 'An Orin planner is running; press Ctrl+C in its terminal first.' >&2; exit 1; }
LOG_FILE="${WORKSPACE}/log/orin-dispatch-planner.log"
exec > >(tee -a "${LOG_FILE}") 2>&1
echo 'MPC dispatch via the PC base owner; collision checking OFF. Send a new RViz goal after owner is armed.'
echo "Basic Twist control: ${BASIC_CONTROL}"
exec ros2 launch local_planner astribot_orin_dispatch.launch.py \
  "bringup_dir:=${BRINGUP}" "planner_source:=${PACKAGE_ROOT}" "basic_control:=${BASIC_CONTROL}"
