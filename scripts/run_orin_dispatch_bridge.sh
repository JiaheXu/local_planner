#!/usr/bin/env bash
# PC relay: leased protocol by default, plain Twist with --basic-control.
set -euo pipefail
BRIDGE=orin_dispatch_bridge.py
if [[ $# == 1 && "$1" == --basic-control ]]; then
  BRIDGE=orin_basic_bridge.py
elif [[ $# != 0 ]]; then
  echo "Usage: $0 [--basic-control]" >&2
  exit 2
fi
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec 9>/tmp/astribot-orin-goal-bridge.lock
flock -n 9 || { echo 'Stop the previous Orin goal bridge before starting dispatch relay.' >&2; exit 1; }
echo "Starting ${BRIDGE}; waiting for Orin commands and fresh base feedback."
mkdir -p "${PACKAGE_ROOT}/../../log"
exec > >(tee -a "${PACKAGE_ROOT}/../../log/orin-dispatch-bridge.log") 2>&1
exec /usr/bin/python3 "${PACKAGE_ROOT}/python/${BRIDGE}"
