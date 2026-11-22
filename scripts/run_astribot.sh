#!/usr/bin/env bash
# Planner package entry point. Never starts or enables robot hardware.
set -euo pipefail
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DISPATCH=false
MODE=run
BACKEND=
PYTHON_DEBUG=true
for arg in "$@"; do
  case "$arg" in
    --dispatch) DISPATCH=true ;;
    --check) MODE=check ;;
    --dry-run) MODE=preview ;;
    --planner=mpc) BACKEND=mpc ;;
    --planner=library) BACKEND=library ;;
    --python-debug) PYTHON_DEBUG=true ;;
    --cpp) PYTHON_DEBUG=false ;;
    *) echo "Usage: $0 [--dispatch] [--planner=mpc|library] [--python-debug|--cpp] [--check|--dry-run]" >&2; exit 2 ;;
  esac
done
COMMAND=(/opt/ros/humble/bin/ros2 launch local_planner astribot.launch.xml "dispatch:=${DISPATCH}")
COMMAND+=("python_debug:=${PYTHON_DEBUG}")
[[ -z "${BACKEND}" ]] || COMMAND+=("planner_backend:=${BACKEND}")
if [[ "${MODE}" == preview ]]; then
  printf 'ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 '
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi
# Source checkout or the installed lib/local_planner entry point.
if [[ -f "${PACKAGE_ROOT}/package.xml" ]]; then
  WORKSPACE="${LOCAL_PLANNER_WORKSPACE:-$(cd "${PACKAGE_ROOT}/../.." && pwd)}"
  PREFIX="${LOCAL_PLANNER_PREFIX:-${WORKSPACE}/install/local_planner}"
else
  PREFIX="${LOCAL_PLANNER_PREFIX:-$(cd "${PACKAGE_ROOT}/.." && pwd)}"
fi
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
[[ -f "${PREFIX}/share/local_planner/local_setup.bash" ]] || {
  echo "Missing installed local_planner at ${PREFIX}; run the package's scripts/build.sh." >&2; exit 1;
}
set +u
source /opt/ros/humble/setup.bash
export AMENT_CURRENT_PREFIX="${PREFIX}"
source "${PREFIX}/share/local_planner/local_setup.bash"
unset AMENT_CURRENT_PREFIX
set -u
if [[ "${MODE}" == check ]]; then
  "${COMMAND[@]}" --show-args
  "${PREFIX}/lib/local_planner/astribotCoordinator" --check
  if [[ "$PYTHON_DEBUG" == true ]]; then
    /usr/bin/python3 - "${PREFIX}/lib/local_planner" <<'PY'
import sys, runpy
from pathlib import Path
sys.path.insert(0, sys.argv[1])
runpy.run_path(str(Path(sys.argv[1]) / 'astribotCoordinatorPython'), run_name='dependency_check')
PY
  fi
  "${PREFIX}/lib/local_planner/planar_mpc_test"
  "${PREFIX}/lib/local_planner/astribot_core_test"
  echo 'Package dependencies and installed launch checked; no ROS nodes started.'
  exit 0
fi
echo "Astribot planner dispatch=${DISPATCH}. Select Auto Base Control before a new RViz goal."
exec "${COMMAND[@]}"
