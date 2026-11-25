#!/usr/bin/env bash
# Build only message/action types. Starts no ROS nodes or hardware processes.
set -eo pipefail
export PYTHONNOUSERSITE=1
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="$(cd "${PACKAGE_ROOT}/../.." && pwd)"
source /opt/ros/humble/setup.bash
cd "${WORKSPACE}"
exec /usr/bin/colcon --log-base "${WORKSPACE}/log/navigation_interfaces" build \
  --base-paths "${PACKAGE_ROOT}/interfaces/local_planner_interfaces" \
  --packages-select local_planner_interfaces \
  --build-base "${WORKSPACE}/build_navigation_interfaces" \
  --install-base "${WORKSPACE}/install" \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
