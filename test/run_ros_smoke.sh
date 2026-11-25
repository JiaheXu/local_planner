#!/usr/bin/env bash
# Only synthetic inputs, localhost DDS domain 227, private shadow output.
set -euo pipefail
PREFIX="${1:?installed package prefix required}"
BACKEND="${2:-mpc}"
IMPLEMENTATION="${3:-cpp}"
SCENARIO="${4:-full}"
[[ "$BACKEND" == mpc || "$BACKEND" == library ]] || exit 2
[[ "$IMPLEMENTATION" == cpp || "$IMPLEMENTATION" == python ]] || exit 2
[[ "$SCENARIO" == full || "$SCENARIO" == shutdown ]] || exit 2
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
export ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/astribot-cpp-smoke}"
mkdir -p "$ROS_LOG_DIR"
BIN="$PREFIX/lib/local_planner"
SHARE="$PREFIX/share/local_planner"
backend_pid= coordinator_pid=
cleanup() {
  [[ -z "$coordinator_pid" ]] || kill -TERM "$coordinator_pid" 2>/dev/null || true
  [[ -z "$backend_pid" ]] || kill -TERM "$backend_pid" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT
"$BIN/localPlanner" --ros-args --params-file "$SHARE/config/astribot.yaml" \
  -p "pathFolder:=$SHARE/paths" -p "planner_backend:=$BACKEND" \
  -r /scan:=/astribot/local_planner/scan -r /way_point:=/astribot/local_planner/way_point \
  -r /planner_cancel:=/astribot/local_planner/cancel -r /path:=/astribot/local_planner/path \
  -r /free_paths:=/astribot/local_planner/free_paths -r /mpc_request:=/astribot/local_planner/mpc_request \
  -r /mpc_result:=/astribot/local_planner/mpc_result > "$ROS_LOG_DIR/backend-$BACKEND.log" 2>&1 &
backend_pid=$!
COORDINATOR="$BIN/astribotCoordinator"
[[ "$IMPLEMENTATION" != python ]] || COORDINATOR="$BIN/astribotCoordinatorPython"
"$COORDINATOR" --ros-args --params-file "$SHARE/config/astribot.yaml" \
  -p dispatch:=false -p "planner_backend:=$BACKEND" \
  > "$ROS_LOG_DIR/coordinator-$BACKEND.log" 2>&1 &
coordinator_pid=$!
export LOCAL_PLANNER_TEST_BACKEND_PID="$backend_pid"
export LOCAL_PLANNER_TEST_COORDINATOR_PID="$coordinator_pid"
"$BIN/astribot_ros_smoke" "$BACKEND" "$SCENARIO"
