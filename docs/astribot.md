# Astribot planner operation

All local-planner implementation, configuration, launch files and tests live in
`ros2_ws/src/local_planner`. XPolicy is the robot interface: it provides sensor
topics and the sole SDK owner that consumes selected velocity commands.
The planner imports no XPolicy code, reads no XPolicy files at runtime and never
enables hardware. The desktop supervisor only invokes this package launcher.

## Build and launch

```bash
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/build.sh
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh --check
```

Build/install directories are `ros2_ws/build/local_planner` and
`ros2_ws/install/local_planner`. Both MPC and library modes use this install.
`LOCAL_PLANNER_WORKSPACE` and `LOCAL_PLANNER_PREFIX` support alternative build
and install locations. Rebuild after editing source configuration or code.

`scripts/run_astribot.sh` defaults to private shadow commands. `--dispatch`
publishes `/cmd_vel`; `--planner=library` selects the original library backend.
`config/astribot.yaml` defaults to `planner_backend: mpc`. The installed launch
entry is `ros2 launch local_planner astribot.launch.py` with `dispatch` and
`planner_backend` arguments. An exited component shuts down its peer.

**Start Astribot** calls the package launcher with `--dispatch`, alongside the
formal XPolicy owner, sensors, FAST-LIO and RViz. Startup preflights run before
the owner starts. The base starts disabled/HOLD. **Auto Base Control** selects
and enables the owner; then a **new** RViz **2D Goal Pose** can be accepted.
**Cancel Goal** clears the target, and **Stop Astribot** stops the supervised
process groups. Do not run Move Forward or another `/cmd_vel` publisher alongside
this planner. XPolicy retains command-source arbitration, enable/stop services,
the 200 ms command watchdog and the command-to-SDK sign mapping.

## ROS contract

| Input | Topic / contract |
|---|---|
| Front point cloud | `/sdk/astribot_lidar/front_scan`, PointCloud2 XYZ in nominal front `livox_frame` |
| Sensor clock references | `/sdk/astribot_lidar/front_imu`, `/sdk/astribot_lidar/back_imu` |
| Chassis odometry | `/fast_lio/chassis/odometry`, `map → astribot_viz_chassis` |
| Goal / cancel | `/astribot_viz/goal_pose`, `/astribot_viz/goal_cancel` |
| Owner state | `/astribot/base/status`, JSON String with source/generation/feedback state |

Output is an unstamped planar Twist on `/cmd_vel` in dispatch mode or
`/astribot/local_planner/cmd_vel_shadow` otherwise. Status and the orange predicted
map path are `/astribot/local_planner/status` and `/astribot/local_planner/path_map`.
The coordinator requires one publisher per required input/output and pins their
identities per goal. Dispatch requires the auto owner armed in MODEL, hardware
feedback at most 50 ms old, no pending stop, and unchanged generation.

The front cloud and odometry retain original sensor stamps. Advancing IMU clocks
validate source time, including the existing +37 s epoch; no receipt restamping.
Reference freshness is 100 ms, cloud/odometry receipt freshness 250 ms, front/rear
reference agreement 100 ms and cloud/pose source separation at most 200 ms.
`python/navigation_feedback.py` implements the planner's observation checks.

## Goal stages and backends

1. `rotate_to_goal`: turn in place toward the selected position, within 0.05 rad.
2. `drive`: use MPC or the library until actual map-position error is at most 0.03 m.
3. `rotate_to_yaw`: turn in place to the RViz arrow yaw, within 0.05 rad. Position
   drift outside tolerance restarts the approach.

Maximum local-goal distance is 3 m. Both turns command zero translation and at
most 0.1 rad/s. Selecting a replacement goal resets predictions and pending
results; cancel stops every phase. Short local paths do not count as arrival.

The C++ `src/localPlanner.cpp` selects the backend. MPC lives in
`include/local_planner/planar_mpc.hpp` and `mpc_ros.hpp`; it does not load lattice
assets. The library backend uses the existing lattice and same-stamped scan/goal
pairs. The legacy `pathFollower` binary is built for compatibility but not launched.
The package's `python/local_planner_node.py` owns goal stages and the command
checks; `local_planner_core.py` owns geometry and library-path following.

MPC uses a 2 s horizon (20 × 0.1 s), planar vx/vy and zero drive-stage yaw. Eigen
ADMM solves a convex quadratic objective with linear constraints; this is an
independent implementation, not the OSQP library. Mathematical reference:
[ADMM QP formulation](https://osqp.org/docs/solver/index.html).
Its speed bound is `|vx| + |vy| <= 0.1 m/s`; diagonal speed is therefore lower.
Component acceleration is at most 0.14 m/s². The library follower remains at
0.05 m/s. The owner independently applies its final velocity/acceleration limits.

`MpcRequest` and `MpcResult` carry source stamps and goal epochs. Requests include
goal, retained obstacles, local-PC send time and last actually published velocity.
The coordinator rejects unmatched, cancelled, out-of-order, late or invalid
replies. Moving results require 21 finite planar samples consistent with the
command and step limits. Blocked results require zero and an origin-only path.
The C++ solve acceptance budget is 40 ms with 4,000 iterations; cached proposals
expire 150 ms after request creation. Failures cancel the target. Commands run at
20 Hz; requests normally follow the front cloud at 10 Hz. Dispatch acceleration
uses elapsed time, so faster replies cannot increase the ramp. Stops are immediate.

## Collision geometry and limits

`config/astribot_geometry.yaml` pins the nominal front-LiDAR mount:
translation `[0.17393, 0.16893, 0.170397]` m, yaw approximately −45°. This composes
model z=0.073397 m with the SDK weld-to-base offset of 0.097 m exactly once.
It matches XPolicy's current observation-frame contract; runtime imports are not
required. Source provenance is Astribot-Dev/astribot_descriptions commit
`32b67e270fecd49bd5444ca49824dcc171dfb3e6`, `lidar_mid360_front.xml` and the SDK
`astribot_torso.yaml` weld definition. The older unverified vendor calibration
candidate is not substituted. Mount changes must update the observation contract
and this package's configuration consistently.

Finite sensor returns with range ≥0.05 m are transformed to chassis coordinates.
Empty/all-invalid clouds are rejected. Retained obstacle heights are 0.03–1.8 m,
radial distances 0.30–4 m; the inner radius removes nominal chassis self-returns.
The library path lattice radius is fixed at 0.55 m. MPC uses a 0.55 m centre
clearance, up to 24 nearby separating half-plane constraints and validation of
predicted samples against every retained return. A new cloud rechecks cached
predictions. The actually dispatched MPC command is checked at 0.55 m through
the owner's 0.2 s command lifetime, including after acceleration limiting.
An additional 0.40 m / 2 s commanded-sweep check covers both backends and turns.

These are nominal chassis-only checks. Front blind spots, dropped returns, low
obstacles, slopes/drop-offs, extended arms, sensor/pose error and braking distance
are not fully covered. There is no global route search or guarantee of finding
a detour. Solver deadlines are acceptance checks, not hard real-time preemption.
Physical navigation and collision response require supervised qualification.

## Verification

`scripts/build.sh` runs CTest: the native optimizer regression executable and all
Python tests in `test/`. These cover geometry, goal stages, timing, cancellation,
request/result validation, rate limiting, owner gating and collision checks.

Run the synthetic ROS harness on isolated domain 227 (no SDK or hardware owner):

```bash
source /opt/ros/humble/setup.bash
export AMENT_CURRENT_PREFIX=/home/ubuntu/ros2_ws/install/local_planner
source "$AMENT_CURRENT_PREFIX/share/local_planner/local_setup.bash"
unset AMENT_CURRENT_PREFIX
env -u FASTRTPS_DEFAULT_PROFILES_FILE -u FASTDDS_DEFAULT_PROFILES_FILE \
  PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=0 \
  ROS_LOG_DIR=/tmp/local-planner-smoke \
  /usr/bin/python3 /home/ubuntu/ros2_ws/src/local_planner/test/astribot_local_planner_ros_smoke.py --planner=mpc
# Repeat with --planner=library.
```

Both backends cover clear travel, obstacles, cancel, stale cloud/odometry,
invalid returns, 0.2 m arrival, turn–drive–turn, final-turn cancel and C++ process
loss. MPC additionally covers lateral drift through arrival. The harness asserts
no `/cmd_vel` publisher exists. UDP may be denied in the sandbox; shared-memory
ROS transport supports these local scenarios, not networked DDS qualification.

Earlier implementation and benchmark records are retained under `docs/history/`.
They describe earlier layouts and must not be used as current launch instructions.
