#!/usr/bin/env bash
# Run on the PC alongside RViz; bridges goals and visualization, never /cmd_vel.
set -euo pipefail
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd "${PACKAGE_ROOT}/../.." && pwd)"
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
set +u
source /opt/ros/humble/setup.bash
export AMENT_CURRENT_PREFIX="${WORKSPACE}/install/domain_bridge"
source "${AMENT_CURRENT_PREFIX}/share/domain_bridge/local_setup.bash"
unset AMENT_CURRENT_PREFIX
set -u
export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
if [[ -n "${PLANNER_DDS_PROFILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${PLANNER_DDS_PROFILE}"
fi
exec 9>/tmp/astribot-orin-goal-bridge.lock
flock -n 9 || { echo 'Orin goal bridge already running' >&2; exit 1; }
pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 0' INT TERM
for route in goals feedback; do
  "${WORKSPACE}/install/domain_bridge/lib/domain_bridge/domain_bridge" \
    "${PACKAGE_ROOT}/config/orin_shadow_${route}.yaml" &
  pids+=("$!")
done
wait -n
