# C++ Astribot planner migration

**Goal:** Add C++ runtime/tests and XML launch while preserving the public ROS
interface. Latest user steering keeps Python as the default for debugging now;
`--cpp` explicitly selects C++ for later use.
The user's request authorizes this migration. No live launch or robot commands.

**Architecture:** Retain localPlanner C++ backend and its MpcRequest/MpcResult
interface; replace astribotCoordinator with an rclcpp executable of the same name.
Move geometry, timing, goal sequence and path session into C++ headers. XML launch
starts both nodes and terminates the session if either exits. XPolicy's script
entry point remains scripts/run_astribot.sh. The latest user steering retains all
Python source for debugging (`--python-debug`) and removes LiDAR IMU subscriptions
from both coordinators. FAST-LIO's IMU input remains unchanged.

## Constraints
- Use front-only default, optional rear/auto selection, source pinning per goal,
  SDK-free coordinator, one output publisher, MODEL/armed/fresh owner gating.
- Preserve source-time, monotonic receipt, schema, frame, epoch,
  publisher identity, command bound and collision gate. Scan age limits 500 ms;
  odometry receipt/progression reference250ms/stable200ms. Odometry itself drives
  the source-clock reference; no planner IMU subscriptions or IMU readiness gates.
- Preserve turn-drive-turn, 0.03m/0.05rad tolerance, MPC0.1m/s, library0.05m/s.
- Source files stay under ros2_ws/src/local_planner. Work in /tmp staging until
  build/review succeeds. Back up original dirty package before removals.
- The compiled coordinator is named astribotCoordinator. Retained Python debug
  entry is astribotCoordinatorPython. Launch chooses exactly one coordinator.

## Tasks
1. Port python/navigation_feedback.py, collision_feedback.py and
   local_planner_core.py into include/local_planner/astribot_core.hpp and native
   tests; port python/local_planner_node.py to src/astribotCoordinator.cpp.
   Add executable/library declarations to CMakeLists as needed. Use existing
   yaml-cpp and ament_index_cpp for geometry; original source is parity oracle.
2. Root migrates launch to XML, packaging/dependencies and launcher check, and
   ports synthetic integration harness to C++ (domain227, localhost, shadow only).
3. Build stage into /tmp/astribot-cpp-ws; run native unit and synthetic ROS tests
   for MPC and library, loss/invalid/stale inputs, cancel, rotation and arrival.
4. Independently review migration against backed-up Python source; fix findings.
5. Install reviewed files while retaining Python debug source. Rebuild active
   package, verify native executable and both launcher choices
   check. Update docs and instruct user to restart manually.

## Review focus
- No goal resumes after timing recovers or an owner/source generation changes.
- Fresh arrival cannot legitimize stale source stamps or nonfinite JSON feedback.
- Organized/big-endian clouds and rear coordinate transform retain exact contract.
- Delayed MPC results cannot refresh expiry; cancellation invalidates epochs.
- XML shutdown and installed launcher resolve the compiled coordinator, never a
  stale Python script. New tests run release builds without assert being disabled.
