#!/usr/bin/env bash
# Build/verify only. Native coordinator tests use isolated domain228 shadow nodes;
# no robot owner, hardware startup, or live-domain publishing.
set -euo pipefail
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${LOCAL_PLANNER_WORKSPACE:-$(cd "${PACKAGE_ROOT}/../.." && pwd)}"
PREFIX="${LOCAL_PLANNER_PREFIX:-${WORKSPACE}/install/local_planner}"
BUILD_DIR="${LOCAL_PLANNER_BUILD_DIR:-${WORKSPACE}/build/local_planner}"
ROS_SETUP="${LOCAL_PLANNER_ROS_SETUP:-/opt/ros/humble/setup.bash}"
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1
set +u
source "${ROS_SETUP}"
set -u
cmake -S "${PACKAGE_ROOT}" -B "${BUILD_DIR}" \
  "-DCMAKE_INSTALL_PREFIX=${PREFIX}" -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "${BUILD_DIR}" -j "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "${BUILD_DIR}"
set +u
export AMENT_CURRENT_PREFIX="${PREFIX}"
source "${PREFIX}/share/local_planner/local_setup.bash"
unset AMENT_CURRENT_PREFIX
set -u
ctest --test-dir "${BUILD_DIR}" --output-on-failure
