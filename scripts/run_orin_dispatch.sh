#!/usr/bin/env bash
# Manual Orin launch. Match basic/leased mode with the PC relay.
set -euo pipefail
BASIC_CONTROL=false
IGNORE_LIDAR=false
OBSTACLE_MODE=''
DRY_RUN=false
for option in "$@"; do
  case "${option}" in
    --basic-control) BASIC_CONTROL=true ;;
    --ignore-lidar|--avoid-obstacles)
      if [[ -n ${OBSTACLE_MODE} ]]; then echo 'Choose exactly one obstacle mode.' >&2; exit 2; fi
      OBSTACLE_MODE="${option}"
      if [[ ${option} == --ignore-lidar ]]; then IGNORE_LIDAR=true; fi ;;
    --dry-run) DRY_RUN=true ;;
    *) echo "Unknown option: ${option}" >&2; exit 2 ;;
  esac
done
if [[ -z ${OBSTACLE_MODE} ]]; then
  echo "Usage: $0 (--avoid-obstacles|--ignore-lidar) [--basic-control] [--dry-run]" >&2
  exit 2
fi
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd "${PACKAGE_ROOT}/../.." && pwd)"
BRINGUP="${WORKSPACE}/src/astribot_planner_bringup"
if [[ ${DRY_RUN} == true ]]; then
  printf 'ROS_DOMAIN_ID=25 ros2 launch local_planner astribot_orin_dispatch.launch.py basic_control:=%s ignore_lidar:=%s\n' "${BASIC_CONTROL}" "${IGNORE_LIDAR}"
  exit 0
fi
source "${BRINGUP}/environment.sh"
set +u
export AMENT_CURRENT_PREFIX="${WORKSPACE}/install/local_planner"
source "${AMENT_CURRENT_PREFIX}/share/local_planner/local_setup.bash"
unset AMENT_CURRENT_PREFIX
if [[ ${BASIC_CONTROL} == true ]]; then
  export AMENT_CURRENT_PREFIX="${WORKSPACE}/install/local_planner_interfaces"
  if [[ ! -f "${AMENT_CURRENT_PREFIX}/share/local_planner_interfaces/local_setup.bash" ]]; then
    echo "Build navigation actions: bash ${PACKAGE_ROOT}/scripts/build_navigation_interfaces.sh" >&2
    exit 1
  fi
  source "${AMENT_CURRENT_PREFIX}/share/local_planner_interfaces/local_setup.bash"
  unset AMENT_CURRENT_PREFIX
  /usr/bin/python3 -c 'from local_planner_interfaces.action import NavigateLocalGoal'
fi
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
echo "Direct-goal XY+yaw MPC via the PC base owner; no path selector. Cloud-check mode: ${OBSTACLE_MODE}. Waiting for a new goal."
echo "Basic Twist control: ${BASIC_CONTROL}"
exec ros2 launch local_planner astribot_orin_dispatch.launch.py \
  "bringup_dir:=${BRINGUP}" "planner_source:=${PACKAGE_ROOT}" "basic_control:=${BASIC_CONTROL}" \
  "ignore_lidar:=${IGNORE_LIDAR}"
