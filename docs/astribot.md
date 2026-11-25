# Astribot planner operation

## Orin basic mode: acknowledged navigation action

Build `bash scripts/build_navigation_interfaces.sh` on PC and Orin once, then use
the usual desktop Start Astribot and manual Orin command:
`bash ~/ros2_ws/src/local_planner/scripts/run_orin_dispatch.sh --ignore-lidar --basic-control`.
The launcher sources `local_planner_interfaces` automatically. The tiny interface
build starts no nodes and does not rebuild the MPC backend.

RViz still publishes `/astribot_viz/goal_pose` locally. The PC relay sends
`local_planner_interfaces/action/NavigateLocalGoal` to
`/cocomatrix/local_planner/navigate` in Orin domain 25. Action acceptance, feedback,
completion/abort and cancellation are correlated by UUID. The basic coordinator
no longer subscribes to remote goal/cancel topics. Its existing MPC, navigation
checks, tolerances, velocity limits and owner-independent planning are unchanged.
It waits up to 5 s for navigation, then aborts visibly. Completion requires both
position and yaw tolerances; other cleared goals return abort reasons.

The desktop queues the latest click up to 10 s for discovery/cancellation and
waits 5 s for acceptance. Expired clicks are not replayed after reconnection.
Canceled or timed-out requests never open the PC command gate on late acceptance.
Replacement waits for the prior terminal result. An uncertain send is retained
until its outcome is known, rather than risking duplicate execution. The desktop
log and `/astribot/local_planner/goal_delivery` JSON topic show delivery state and
goal UUID. Network loss cannot guarantee delivery. Twist remains the command
transport; no additional SDK writer, ownership gate or velocity watchdog is added.
The obsolete standalone `run_orin_dispatch_bridge.sh --basic-control` refuses
startup; use the desktop relay instead.

## Manual terminal debugging on the PC

Start Astribot starts the owner/sensors/FAST-LIO bridge/RViz and requests automatic
`/cmd_vel` enable once. It waits up to 60 seconds for one command publisher and
the owner's startup checks. Within that window, launch the planner manually:
```bash
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh --dispatch --python-debug --planner=mpc
```
Wait for `enabled: true`, then send a new RViz goal. No Auto Base Control click is
needed on initial startup. The Python planner with `--dispatch` continuously
maintains automatic enable for the existing `/cmd_vel` owner. It recovers from
`publisher_conflict_or_missing` and `command_timeout` HOLD after the goal clears,
using fresh owner status, confirmed stop, fresh hardware feedback and fresh zero
commands from one publisher. Transient service refusals retry at most once per
second, with no overall wait deadline. It never races the desktop's initial
enable. Manual disable, faults, other source selection and uncertain service
timeouts latch automatic recovery off for that planner process. Goals cancelled
by an interruption are not restored. To explicitly select/enable Auto Base Control:
```bash
bash /home/ubuntu/xpolicylabdagger/scripts/astribot_base_auto.sh
```
Then send a new RViz goal. Stop the manual planner with Ctrl+C separately from
the desktop session. Never run two `/cmd_vel` publishers.

The Python coordinator emits one compact diagnostic line per second, and immediately
on phase/owner/error transitions:
`[local+planner]: {cmd_vel} [0.100,0.000,0.000], status{drive; C++ MPC tracking}`.
Values are commanded `[vx, vy, wz]` in m/s, m/s and rad/s, not measured motion.
ROS launch may prepend its process label. Startup and event logs remain available.
The launcher appends stdout and stderr to `/tmp/astribot-local-planner.log` on
every run, including a timestamped session marker. `LOCAL_PLANNER_LOG` overrides
the path; `--dry-run` and `--check` do not create this log.
Completion and cancellation clear the goal, predictions and per-goal publisher
pins, then keep publishing zero commands while listening for the next goal.
The terminal shows `idle; waiting for next goal`; stale odometry or a disabled
owner still appears as a current blocker. A new goal requires valid inputs and
an armed owner, but no planner restart. Stop reasons remain in the STOP event
log and detailed status on
`/astribot/local_planner/status`. These diagnostics add no command publisher or
enable request. They are for the Python coordinator; `--cpp` has its own logs.

## Explicit supervised reverse test

`scripts/run_astribot.sh --supervised-reverse-test` selects Python/MPC testing;
add `--dispatch` only inside the authorized formal robot launch. Normal startup
keeps obstacle checking enabled. The test flag ignores obstacles in MPC and
Python dispatch checks, but still requires valid/fresh sensor inputs, a ready
owner, unique publishers and timely MPC replies. Status exposes both the test
flag and `collision_checks_enabled: false`.

Goals must be behind the chassis, <=10 cm (5 mm allowance), with <=5 mm lateral
offset and <=0.01 rad heading change. The coordinator preserves heading, caps
translation at 0.03 m/s and uses 1 cm arrival tolerance. It cancels at 15 seconds,
12 cm reverse travel, 1 cm forward travel, 2 cm lateral drift or 0.1 rad yaw drift.
Cancel and final hardware-owner protections stay active. Use one fixed target
across diagnostic retries; never accumulate repeated reverse targets.
The flag is incompatible with `--cpp` and `--planner=library` and changes no YAML
defaults. Restart without it to restore obstacle checking.

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

**Start Astribot** starts the formal XPolicy owner, sensors, FAST-LIO and RViz.
Startup preflights run before the owner starts. Launch the planner manually with
`--dispatch` within 60 seconds; the startup helper automatically selects and
enables `/cmd_vel` after its checks pass. Python dispatch also maintains enable
after publisher-loss and command-timeout stops. Other stops/faults never
auto-rearm; **Auto Base Control** explicitly re-enables the owner. Then a **new** RViz
**2D Goal Pose** can be accepted.
**Cancel Goal** clears the target, and **Stop Astribot** stops the supervised
process groups. Do not run Move Forward or another `/cmd_vel` publisher alongside
this planner. XPolicy retains command-source arbitration, enable/stop services,
the 200 ms command watchdog and the command-to-SDK sign mapping.

## ROS contract

| Input | Topic / contract |
|---|---|
| Collision point clouds | `/sdk/astribot_lidar/front_scan` by default; explicit `rear` or `auto` modes can use `/sdk/astribot_lidar/back_scan` |
| Chassis odometry | `/fast_lio/chassis/odometry`, `map → astribot_viz_chassis` |
| Goal / cancel | `/astribot_viz/goal_pose`, `/astribot_viz/goal_cancel` |
| Owner state | `/astribot/base/status`, JSON String with source/generation/feedback state |

Output is an unstamped planar Twist on `/cmd_vel` in dispatch mode or
`/astribot/local_planner/cmd_vel_shadow` otherwise. Status and the orange predicted
map path are `/astribot/local_planner/status` and `/astribot/local_planner/path_map`.
The coordinator requires one publisher per required input/output and pins their
identities per goal. Dispatch requires the auto owner armed in MODEL, hardware
feedback at most 50 ms old, no pending stop, and unchanged generation.

The default `collision_lidar: front` in `config/astribot.yaml` subscribes only to
front cloud and uses it with FAST-LIO odometry for every goal. Rear cloud errors
do not gate motion. Front failure stops motion; healthy rear data does not bypass
it. Rebuild/restart after changing this configuration. Explicit `rear` subscribes
only to rear cloud. Optional `auto` selects a fresh valid front cloud, otherwise a
fresh valid rear cloud, at each new goal.
Selection stays fixed for that goal: a newly appearing front feed cannot replace
rear obstacles or cached predictions. Python MPC retains the goal during receipt-age
gaps and resumes after valid data returns from the same feed. Invalid data still
cancels the goal; submit a new goal to select another healthy feed. The library
backend retains cancellation on feed loss. If no permitted feed is usable, output remains zero.
Publisher conflicts and identity changes on required inputs still reject motion.
Status exposes `collision_lidar`, `collision_topic` and `collision_input_errors`.

For PC testing, the default Python coordinator uses `python/receipt_feedback.py`:
odometry and the selected cloud must each have been received locally within
**2 seconds inclusive**, measured with the PC monotonic clock. Their header
timestamps are not compared with each other or with the PC wall clock. Each
stream has its own receipt age; odometry cannot refresh a missing LiDAR stream.
Original stamps remain unchanged and must advance within each stream for replay
checks. MPC request identities are independent of sensor stamps. Invalid frames/geometry and regressing
local monotonic time remain rejected. This receipt policy cannot detect sensor
or network backlog that arrives as new callbacks; it is the requested temporary
PC-testing policy, not a claim of capture-time alignment.

The independent C++ coordinator selected by `--cpp` retains its existing stricter
reference-clock policy in `include/local_planner/astribot_core.hpp`; it is not
selected by Start Astribot. The C++ MPC solver remains in use and its reply/command
deadlines are unchanged. The previous Python reference validators remain for
legacy tests. Neither coordinator subscribes to LiDAR IMU; FAST-LIO retains IMU.
Python MPC publishes zero and retains the active goal during temporary sensor
receipt gaps or missing solver commands. Fresh valid data and a fresh solution
allow it to resume. Explicit cancellation, invalid data, owner faults and publisher
identity changes still clear the goal; those goals never resume automatically. Status
reports `sensor_freshness_policy: independent_receipt_age_2s` after restart.

## Goal stages and backends

Normal Python MPC stays in `drive` while translating forward, backward or sideways
and aligning the RViz goal yaw simultaneously. Both map-position error <=0.03 m
and wrapped yaw error <=0.05 rad are required for completion. Translation speed
is limited to min(0.1, 0.6 * distance_m) m/s, with zero inside 3 cm; yaw correction
continues at up to 0.1 rad/s. Cached MPC velocities are rotated from their request
chassis frame into the current chassis frame. Crossing the position tolerance
does not switch phases or discard the optimizer result. The bounded reverse-test
mode retains its original heading and distance behavior.

The Python library backend and native C++ coordinator retain `rotate_to_goal`
before driving: turn in place toward the selected position, within 0.05 rad.

Maximum local-goal distance is 3 m. The staged backend turns command zero translation and at
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

Both MPC variants use a 2 s horizon (20 × 0.1 s). In Python `--ignore-lidar`
mode, `pose_mpc.hpp` jointly optimizes body vx/vy/yaw-rate in a 60-variable QP.
Each solve linearizes holonomic SE(2) dynamics about the previous published body
velocity rollout. The returned path is the nonlinear rollout of optimized
controls, with an orientation at every pose. This is one sequential-QP step per
request, not an exact nonlinear optimizer. Other Python modes retain the XY
solver plus proportional yaw; native C++ retains staged yaw. Eigen
ADMM solves a convex quadratic objective with linear constraints; this is an
independent implementation, not the OSQP library. Mathematical reference:
[ADMM QP formulation](https://osqp.org/docs/solver/index.html).
Its speed bound is `|vx| + |vy| <= 0.1 m/s`; diagonal speed is therefore lower.
Component acceleration is at most 0.14 m/s². The library follower remains at
0.05 m/s. The owner independently applies its final velocity/acceleration limits.

No-obstacle pose mode retains the 0.1 rad/s angular ceiling and adds a 0.2 rad/s²
optimizer angular acceleration bound. Both optimizer and dispatch reduce XY speed
to `min(0.1, 0.6*distance_m)`. Angular speed is capped at
`min(0.1, 0.03+0.07*min(distance_m/0.3,1), 0.8*abs(wrapped_yaw_error))` rad/s.
Inside 3 cm, translation stops but orientation correction remains possible;
inside 0.05 rad, rotation stops. Optimizer caps allow bounded braking from the
previous command. Dispatch enforces terminal caps immediately, and cancellation
also bypasses the ramps. These final stops are intentionally not acceleration
limited. The initial tuning is software-tested, not hardware-qualified.

`MpcRequest.plan_yaw`, `goal_yaw` (radians relative to the request chassis), and
`previous_yaw_rate` (rad/s) explicitly select pose planning. Pose mode rejects
obstacle-bearing requests. Both hosts need a full `scripts/build.sh` build for
the new request layout; the action interface is unchanged. The coordinator
validates unit planar quaternions and reconstructs body-frame velocities from
the path to check angular/translation speed and acceleration and the first
command. RViz path orientations are transformed back into map coordinates.

On Orin, build with the same vendor ROS environment as the manual runtime and a
separate CMake directory; the system ROS installation lacks the required PCL
packages. Do not combine vendor CMake generators with system ROS Python modules:
```bash
LOCAL_PLANNER_ROS_SETUP="$HOME/ros2_ws/src/astribot_planner_bringup/environment.sh" \
LOCAL_PLANNER_BUILD_DIR="$HOME/ros2_ws/build/local_planner_vendor_yaw" \
bash "$HOME/ros2_ws/src/local_planner/scripts/build.sh"
```
The PC uses the normal `bash scripts/build.sh` command. These commands build and
run offline/isolated-domain tests; neither starts the real planner or SDK writer.

`MpcRequest` and `MpcResult` carry request identities and goal epochs. Python uses
strictly increasing local ROS timestamps so repeated solves of the latest valid
observation are distinct; the native coordinator retains source stamps. Requests include
goal, retained obstacles, local-PC send time and last actually published velocity.
The coordinator rejects unmatched, cancelled, out-of-order, late or invalid
replies. Moving results require 21 finite planar samples consistent with the
command and step limits. Blocked results require zero and an origin-only path.
The C++ solve acceptance budget is 40 ms with 4,000 iterations. New replies must
arrive within 150 ms of request creation. In the Python coordinator, an accepted
command expires 1 second after request creation (including a final check
immediately before dispatch). The separate native
C++ coordinator retains its 250 ms command lifetime. Receiving a reply never
refreshes the original request time.
Python MPC replans every 100 ms while driving, using the latest validated pose and
selected cloud (or no obstacles with `--ignore-lidar`). Each solve predicts 2 s,
but only its first control is used; the rest is replaced by subsequent solves.
Commands publish at 20 Hz. Sensor receipt ages are never refreshed by solving.
Missing results, command expiry and solver failures stop motion and retain the
goal for retry. Late replies are discarded; they cannot extend command lifetime.
After a stop, old in-flight predictions are invalidated and motion requires a new
solution. Invalid results, owner faults, publisher changes and explicit cancel
still clear the goal (basic mode does not gate planning on owner status).
In no-obstacle pose mode, yaw comes from the same accepted MPC result as XY;
there is no proportional-yaw fallback while waiting for a result.
Dispatch acceleration
uses elapsed time, so faster replies cannot increase the ramp. Stops are immediate.

## Collision geometry and limits

For a Python MPC test without cloud messages, add `--ignore-lidar` to
`scripts/run_astribot.sh`. This per-run option disables collision avoidance and
cloud subscriptions, source selection, publisher checks and freshness gates.
The 10 Hz planning timer uses the latest validated FAST-LIO chassis odometry for
empty-obstacle MPC requests. FAST-LIO still requires its own upstream LiDAR/IMU inputs; this option
only removes the planner's cloud dependency. Odometry validation, owner gating,
goal stages, speed/acceleration limits, command expiry and cancellation remain
active. It requires Python and MPC. Normal launches remain collision enabled.
Status reports `ignore_lidar: true`, `collision_checks_enabled: false`, null
collision source/topic and no collision-input errors. Default output remains
shadow; `--dispatch` still explicitly selects `/cmd_vel`.

**Python collision avoidance is enabled by default** with
`python_collision_checks_enabled: true` and `collision_lidar: front` in
`config/astribot.yaml`. Front-cloud obstacles feed MPC (or the library scan),
invalidate unsafe predictions, and stop unsafe translation or in-place turns.
Status reports `collision_checks_enabled: true` and, during a goal,
`collision_topic: /sdk/astribot_lidar/front_scan`.

The topic matches XPolicy's `real_world/robots/astribot_s1/rviz_sensors.py` and
`rviz_sensors.yml`: `/livox/lidar_front` is relayed as `PointCloud2` to
`/sdk/astribot_lidar/front_scan`. `NativeRelay` preserves the payload and header;
the planner applies the front-LiDAR-to-chassis transform below exactly once.

The explicit Python debugging override `python_collision_checks_enabled: false`
still bypasses obstacle avoidance, logs a warning, and reports the disabled state.
C++ ignores that option and always checks collisions. Cloud layout/frame/freshness,
odometry, base-owner validation, command expiry, speed limits and operator
cancellation remain checked in either mode. Rebuild/install and restart after
changing configuration.

With Python collision checks enabled, discarding an unsafe prediction keeps output
zero while the 10 Hz planning timer seeks a replacement. Repeated collision checks
preserve in-flight requests. Missing replacements retain the goal in a waiting
state; explicit cancel still clears it immediately. This applies only to Python MPC.

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
front-only operation needs no rear publishers or subscriptions. Unit tests cover
front-only failure without rear fallback, explicit rear/auto modes, auto front
preference and fallback on missing/invalid/stale front or clock disagreement, source
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
