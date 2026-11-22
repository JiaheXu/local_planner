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
The coordinator currently defaults to Python for debugging. `--cpp` selects the
compiled version, and `--python-debug` explicitly selects Python. They never run
alongside each other through this launch. The legacy
`astribot.launch.py` is retained as a Python-debug launch entry.
`config/astribot.yaml` defaults to `planner_backend: mpc`. The installed launch
entry is `ros2 launch local_planner astribot.launch.xml` with `dispatch` and
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
| Collision point clouds | `/sdk/astribot_lidar/back_scan` by default; front is considered only with `collision_lidar: auto` |
| Chassis odometry | `/fast_lio/chassis/odometry`, `map → astribot_viz_chassis` |
| Goal / cancel | `/astribot_viz/goal_pose`, `/astribot_viz/goal_cancel` |
| Owner state | `/astribot/base/status`, JSON String with source/generation/feedback state |

Output is an unstamped planar Twist on `/cmd_vel` in dispatch mode or
`/astribot/local_planner/cmd_vel_shadow` otherwise. Status and the orange predicted
map path are `/astribot/local_planner/status` and `/astribot/local_planner/path_map`.
The coordinator requires one publisher per required input/output and pins their
identities per goal. Dispatch requires the auto owner armed in MODEL, hardware
feedback at most 50 ms old, no pending stop, and unchanged generation.

The default `collision_lidar: rear` in `config/astribot.yaml` uses rear cloud and
FAST-LIO odometry for every goal. Front cloud errors do not gate motion. Rear failure
stops/cancels; healthy front data does not bypass it. Rebuild/restart after changing
this configuration. The optional `auto` value restores selection of a fresh valid
front cloud, otherwise a fresh valid rear cloud, at each new goal.
Selection stays fixed for that goal: a newly appearing front feed cannot replace
rear obstacles or cached predictions. Loss/invalidity of the selected feed stops
and cancels the goal; submit a new goal to select another healthy feed. There is
no automatic motion resumption. If no permitted feed is usable, output remains zero.
Publisher conflicts and identity changes on required inputs still reject motion.
Status exposes `collision_lidar`, `collision_topic` and `collision_input_errors`.

The selected cloud and odometry retain original sensor stamps. The planner does
not subscribe to front/back IMU; FAST-LIO retains its own IMU input. Advancing
odometry timestamps drive the source-time reference, including the existing +37 s
epoch; no receipt restamping. Reference and odometry receipt freshness are 250 ms.
Cloud receipt age, source age behind odometry, and cloud/pose source separation
each allow up to 2 seconds (inclusive). Cloud future-stamp allowance remains 100 ms.
`include/local_planner/astribot_core.hpp` implements the planner's observation checks.

Odometry source progress is checked against an established monotonic-time baseline,
not the interval between adjacent callbacks. Delivery lag exceeding 250 ms blocks
readiness without permanently poisoning the clock. Once delivery catches up,
200 ms of stable advancing samples is required before accepting a new goal.
A continuously delayed stream cannot become fresh by rebasing. Backward source
time, invalid/regressing monotonic time, or source time jumping more than 250 ms
ahead of the qualified baseline latches a fault; resolve timing and restart the
planner. Initial qualification may discard a buffered startup baseline.
Timing failure during a goal cancels it and publishes zero; recovery never resumes
that goal. Submit a new RViz goal after inputs recover. No sensor restamping or
changes to odometry freshness limits are involved.

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
The compiled `src/astribotCoordinator.cpp` owns goal stages and command checks;
`include/local_planner/astribot_core.hpp` owns geometry, timestamps and library
path following. Python source and tests remain available for debugging; both
implementations use the same contracts and have no planner IMU subscriptions.

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
The C++ solve acceptance budget is 40 ms with 4,000 iterations. New replies must
arrive within 150 ms of request creation; an accepted command expires 250 ms
after request creation (including a final check immediately before dispatch).
The separate command lifetime accommodates 10 Hz cloud arrival jitter, request
encoding/solver delivery and the 50 ms command tick. A measured 148 ms cloud gap
plus 15–26 ms Python request encoding could exceed the old 150 ms command limit
and cancel the goal immediately after alignment. Both Python and C++ use the
same limits; receiving a reply never refreshes the original request time.
Failures still cancel the target; no automatic resume occurs after expiry.
Commands run at 20 Hz; requests normally follow the selected cloud at 10 Hz. Dispatch acceleration
uses elapsed time, so faster replies cannot increase the ramp. Stops are immediate.

## Collision geometry and limits

**Temporary Python debugging default: collision checks are disabled** by
`python_collision_checks_enabled: false` in `config/astribot.yaml`. Python sends
an empty obstacle set to MPC (or an all-infinite scan to the library) and uses
that empty set for prediction and command collision checks. It therefore provides
**no obstacle avoidance**, including during initial/final turns. Status exposes
`collision_checks_enabled: false`, and startup logs a warning. C++ ignores this
Python-only option and retains all collision checks unchanged. Set the option to
`true`, rebuild/install and restart to restore Python collision checking.
Cloud layout/frame/freshness and odometry checks, base-owner validation, command
expiry, speed limits and operator cancellation remain enabled in either mode.

With Python collision checks enabled, discarding an unsafe prediction starts a
separate 0.5 s wait for a replacement, rather than timing out against the original
drive-phase start. Repeated collision checks keep output zero without discarding
in-flight requests or extending that wait. A missing replacement still cancels
the goal; the error records the reason the previous prediction was invalidated.
This replanning timer fix currently applies only to Python.

`config/astribot_geometry.yaml` pins the nominal front-LiDAR mount:
translation `[0.17393, 0.16893, 0.170397]` m, yaw approximately −45°. This composes
model z=0.073397 m with the SDK weld-to-base offset of 0.097 m exactly once.
It matches XPolicy's current observation-frame contract; runtime imports are not
required. Source provenance is Astribot-Dev/astribot_descriptions commit
`32b67e270fecd49bd5444ca49824dcc171dfb3e6`, `lidar_mid360_front.xml` and the SDK
`astribot_torso.yaml` weld definition. The older unverified vendor calibration
candidate is not substituted. Mount changes must update the observation contract
and this package's configuration consistently.

The current vendor **rear cloud is already expressed in front-LiDAR coordinates**,
despite the `/back_scan` topic name. `rear_cloud_coordinates: front_lidar` explicitly
pins the same convention used by XPolicy's FAST-LIO observation contract. The
planner applies the front-frame-to-chassis transform once for either cloud; it
does not apply the physical rear mounting transform to already transformed data.
Unknown rear coordinate conventions fail startup. This nominal convention must
be requalified after robot driver/calibration changes. Rear-only operation has
different blind spots; it does not provide front-sensor visibility guarantees.

Finite sensor returns with range ≥0.05 m are transformed to chassis coordinates.
Empty/all-invalid clouds are rejected. Retained obstacle heights are 0.03–1.8 m,
radial distances 0.30–4 m; the inner radius removes nominal chassis self-returns.
The library path lattice radius is fixed at 0.55 m. MPC uses a 0.55 m centre
clearance, up to 24 nearby separating half-plane constraints and validation of
predicted samples against every retained return. A new cloud rechecks cached
predictions. The actually dispatched MPC command is checked at 0.55 m through
the owner's 0.2 s command lifetime, including after acceleration limiting.
An additional 0.40 m / 2 s commanded-sweep check covers both backends and turns.

These are nominal chassis-only checks. Sensor blind spots, dropped returns, low
obstacles, slopes/drop-offs, extended arms, sensor/pose error and braking distance
are not fully covered. There is no global route search or guarantee of finding
a detour. Solver deadlines are acceptance checks, not hard real-time preemption.
Physical navigation and collision response require supervised qualification.

## Verification

`scripts/build.sh` runs native CTest regressions for the optimizer and coordinator
core, native coordinator boundary tests on isolated localhost domain228, and the
retained Python regression suite. These cover geometry, timing, goal stages,
owner/source gates, reply validation, cancellation and command deadlines. The
native synthetic ROS harness exercises the two compiled nodes together.

Run the synthetic ROS harness on isolated domain 227 (no SDK or hardware owner):

```bash
source /opt/ros/humble/setup.bash
export AMENT_CURRENT_PREFIX=/home/ubuntu/ros2_ws/install/local_planner
source "$AMENT_CURRENT_PREFIX/share/local_planner/local_setup.bash"
unset AMENT_CURRENT_PREFIX
env -u FASTRTPS_DEFAULT_PROFILES_FILE -u FASTDDS_DEFAULT_PROFILES_FILE \
  ROS_LOG_DIR=/tmp/local-planner-smoke \
  bash /home/ubuntu/ros2_ws/src/local_planner/test/run_ros_smoke.sh \
  /home/ubuntu/ros2_ws/install/local_planner mpc
# Repeat with library instead of mpc. Add a third argument python to test the
# debugging coordinator, or cpp shutdown for final-zero shutdown verification.
# The harness enforces domain227/localhost and shadow output.
```

Both backends cover clear travel, obstacles, cancel, stale cloud/odometry,
invalid returns, 0.2 m arrival, turn–drive–turn, final-turn cancel and C++ process
loss. MPC additionally covers lateral drift through arrival. The harness asserts
rear-only operation needs no front publishers. Unit tests cover front
preference, fallback on missing/invalid/stale front or clock disagreement, source
pinning, inactive-source isolation, and selected-source loss with healthy backup.
The harness asserts no `/cmd_vel` publisher exists. UDP may be denied in the sandbox; shared-memory
ROS transport supports these local scenarios, not networked DDS qualification.

Earlier implementation and benchmark records are retained under `docs/history/`.
They describe earlier layouts and must not be used as current launch instructions.

2026-11-21 fallback verification: 79 Python tests and native C++ CTest passed;
isolated domain-227 ROS runs passed rear-only MPC, rear-only library and front
MPC regression scenarios. The installed launcher dependency check passed.
No robot motion, live-session restart or physical collision qualification was
performed by these checks. Restart Start Astribot to load the installed update.
