#!/usr/bin/env bash
# Build and verify this package only. No ROS nodes or hardware startup.
set -euo pipefail
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${LOCAL_PLANNER_WORKSPACE:-$(cd "${PACKAGE_ROOT}/../.." && pwd)}"
PREFIX="${LOCAL_PLANNER_PREFIX:-${WORKSPACE}/install/local_planner}"
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
set +u
source /opt/ros/humble/setup.bash
set -u
cmake -S "${PACKAGE_ROOT}" -B "${WORKSPACE}/build/local_planner" \
  "-DCMAKE_INSTALL_PREFIX=${PREFIX}" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "${WORKSPACE}/build/local_planner" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "${WORKSPACE}/build/local_planner"
set +u
export AMENT_CURRENT_PREFIX="${PREFIX}"
source "${PREFIX}/share/local_planner/local_setup.bash"
unset AMENT_CURRENT_PREFIX
set -u
ctest --test-dir "${WORKSPACE}/build/local_planner" --output-on-failure
