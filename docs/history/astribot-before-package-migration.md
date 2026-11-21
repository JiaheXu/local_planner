# Astribot front-LiDAR local planner

The existing **C++ `localPlanner` executable** in
`/home/ubuntu/ros2_ws/src/local_planner` uses **MPC by default** for the drive
stage, with its original path lattice as an explicit alternative.
Both use XPolicy observations and the existing base owner. **No Nav2 is used.**
The legacy `pathFollower` executable is not launched: it mixes local paths with
world-position calculations, has odometry disconnected, publishes TwistStamped,
and only increases yaw gain near obstacles.

## Data and control path

```text
XPolicy front_scan PointCloud2 → nominal front mount → chassis obstacles
FAST-LIO chassis odometry + RViz goal → chassis-frame destination
                    ↓ source-time checks
turn toward destination → C++ MPC (default) or library / follower → align goal yaw
                    ↓ collision/freshness/source/owner gates
shadow Twist, or explicit /cmd_vel → existing XPolicy owner → SDK
```

Inputs on domain 0:

| Input | Topic |
|---|---|
| Front XYZ cloud | `/sdk/astribot_lidar/front_scan` |
| Front and rear sensor clock references | `/sdk/astribot_lidar/front_imu`, `/sdk/astribot_lidar/back_imu` |
| Chassis pose | `/fast_lio/chassis/odometry` (`map → astribot_viz_chassis`) |
| Goal / cancel | `/astribot_viz/goal_pose`, `/astribot_viz/goal_cancel` |
| Existing owner state | `/astribot/base/status` |

The front cloud's `livox_frame` is interpreted explicitly as the nominal front
LiDAR coordinate system from `description/observation_frames.yml`, not as the
rear IMU. Front mount in chassis: `[0.17393, 0.16893, 0.170397]` m, yaw −45°.
Finite XYZ returns with sensor range at least 0.05 m are transformed into chassis coordinates;
clouds with no usable returns are rejected. Heights 0.03–1.8 m
and radii 0.30–4 m are retained. The library backend additionally projects these
points to a nearest-return angular scan, paired with a same-stamp destination.
The inner radius removes nominal chassis self-returns. These filters and mount
are nominal, not per-robot collision calibration.

The front collision input is already enabled when the desktop planner starts;
no second LiDAR consumer or SDK connection is required. Source verification on
2026-11-21 matched the configured transform against these local SDK resources:

- `/home/ubuntu/astribot_sdk_ros2/astribot_scripts/integrations/fast_lio/rear_nominal_extrinsics.json`,
  field `front_in_chassis_base`: model-relative translation
  `[0.17393, 0.16893, 0.073397]` m and yaw −45°.
- `/home/ubuntu/astribot_sdk_ros2/astribot_config/robot_config/astribot_s1/astribot_torso.yaml`,
  `transform.weld_to_base_pose`: adds `[0, 0, 0.097]` m once to express the model
  mount in the SDK chassis frame used by FAST-LIO chassis odometry.
- `/home/ubuntu/astribot_sdk_ros2/astribot_scripts/README.md`, nominal S1 mount
  section: identifies the pinned MJCF `lidar_mid360_front.xml`; the local
  whole-body URDF omits the LiDAR frame.

Thus `p_base = Rz(-45°) p_front + [0.17393, 0.16893, 0.170397]` m.
The older SDK README also records a different **unverified** vendor calibration
candidate (z=0.054 m and a slightly different yaw). It is not the active model
transform and has not been substituted silently. Confirming driver-applied
transforms and per-robot calibration remains a live qualification task.

Original sensor stamps are preserved. Advancing front/rear IMUs validate their
respective source times, with 100 ms reference freshness and 250 ms local
cloud/odometry freshness. Front/rear clock disagreement over 100 ms or cloud/pose
separation over 200 ms blocks a goal. The existing +37 s sensor epoch is not
mistaken for PC-clock staleness; no streams are restamped to receipt time.

## Build and check

```bash
cd /home/ubuntu/xpolicylabdagger
bash scripts/setup_astribot_local_planner.sh
bash scripts/astribot_local_planner.sh --check
```

Both backends require the built C++ package. MPC uses Eigen and does not load
the lattice assets. The Python/SciPy MPC implementation has been removed.
The build runs CTest, and `--check` runs the C++ regression executable without
starting ROS nodes or hardware. The Python XPolicy adapter retains sensor
conversion, goal-stage sequencing and dispatch safety checks.

To check the alternative library backend:

```bash
bash scripts/astribot_local_planner.sh --planner=library --check
```

Build output is under `.xpolicy/ros2/astribot/install/local_planner`. The supplied
external C++ source was updated in place. Its patch is versioned at
`real_world/robots/astribot_s1/tools/local_planner_astribot.patch`; setup refuses
an incompatible/unpatched source instead of overwriting it. Set
`ASTRIBOT_LOCAL_PLANNER_SOURCE` when that checkout is elsewhere. Original source
commit was `b0dad73`, original `src/localPlanner.cpp` SHA256
`e479632d6f31b85a3793f7883c693498fb6fa36fbc4b32e40f25ba497525d5b5`.

## Desktop Start Astribot

**Start Astribot** now launches the local planner automatically alongside the
formal base owner, sensors, FAST-LIO and RViz. The planner publishes `/cmd_vel`
with zero output until a goal is accepted. The base starts in HOLD/disabled;
click **auto base control**, require successful enable, then select a new
**2D Goal Pose**. **Cancel Goal** clears that target. **Stop Astribot** stops
the planner with the rest of the session. A component exiting shuts down its
siblings, and all dependency checks finish before starting the owner.

Do not launch a second planner or Move Forward in this desktop session: two
automatic command publishers trigger the existing conflict guard. An existing
session must be stopped and started to load this launcher change.

The desktop reads `planner_backend: mpc` in
`real_world/robots/astribot_s1/local_planner.yml`. Change that value to `library`
and restart the session to persistently select the previous backend. For a
standalone launch, `--planner=mpc` or `--planner=library` overrides the file.
There is no automatic fallback between backends if MPC fails.

## Standalone shadow test

1. Use sensor-only observation mode (without the desktop base planner), e.g.
   `bash /home/ubuntu/xpolicylabdagger/scripts/astribot_desktop.sh sensors`.
2. Run `bash /home/ubuntu/xpolicylabdagger/scripts/astribot_local_planner.sh`.
3. In RViz, keep fixed frame `map` and choose **2D Goal Pose** ahead of the robot,
   within 3 m. The orange path shows the selected route. If RViz was already
   open before the config update, add a Path display on
   `/astribot/local_planner/path_map` or load the updated RViz config.
4. **Cancel Goal** discards the target and path. Ctrl+C exits the planner.

The standalone script defaults to `/astribot/local_planner/cmd_vel_shadow`, which cannot command the owner.
Without simulated robot motion a shadow goal does not become reached merely
because a path was calculated. Status: `/astribot/local_planner/status`.

## Standalone supervised dispatch

The desktop already performs this launch; the steps below apply only to a
separately managed formal owner session with no planner running.

After reviewing the shadow result and preparing a clear supervised test area:

1. Close Move Forward and any other automatic `/cmd_vel` publisher; joystick
   may remain on `/joy_cmd_vel`.
2. Launch `bash /home/ubuntu/xpolicylabdagger/scripts/astribot_local_planner.sh --dispatch`.
3. Click **auto base control** and require a successful enable result, then select
   a **new** RViz goal. A goal selected while disabled is rejected, not queued.
4. Use **Cancel Goal** to stop following; use the existing disable/stop controls
   for owner-level stopping. Fault/unknown stopping must be resolved through the
   existing owner workflow, never by bypassing it.

The planner sends no enable or hardware requests. It emits zero while idle and
on cancel. Source selection/generation changes, missing or duplicate publishers,
invalid observations, and stale paths clear the goal. Clear but blocked paths
emit zero while continuing to replan. Selecting another RViz goal replaces the
old one and restarts the turn–drive–turn sequence. In a separately managed session, loss of sensor or owner feedback
also clears the target. The desktop supervisor additionally terminates its
planner process when stopping the session.

## Limits and verification

The goal is executed in three stages:

1. **`rotate_to_goal`**: turn in place to face the destination, within **0.05 rad**
   (about 2.9°). Goals behind the starting heading are allowed; the robot turns
   toward them rather than driving backward.
2. **`drive`**: use the selected MPC/library controller until the actual selected
   map position is within **0.03 m**. A short predicted path endpoint is not
   treated as the final destination.
3. **`rotate_to_yaw`**: turn in place until the RViz arrow's yaw is within
   **0.05 rad**. Completion requires both position and yaw tolerances. If position
   drifts outside 0.03 m during final alignment, restart the approach.

Translation is zero in both rotation stages. Maximum translation speed is
**0.1 m/s for MPC**, **0.05 m/s for the library**, with yaw limited to **0.1 rad/s**.
Goals beyond 3 m remain rejected. At the current position, a goal
with a different arrow direction performs only final-yaw alignment. Cancel Goal,
collision checking and stale-data/owner/source gates apply to every stage.
Predictions, warm starts and in-flight snapshots are discarded on stage transitions;
a new solution/path must arrive after initial alignment. Status includes `phase`,
`planner_backend`, `mpc_solve_ms` and `[vx, vy, wz]` on
`/astribot/local_planner/status`.

### MPC drive stage

The canonical implementation is in the C++ package:

- `src/localPlanner.cpp`: selects `mpc` (default) or `library` at startup.
- `include/local_planner/planar_mpc.hpp`: planar velocity integrator with a
  **2 s horizon** (20 steps of 0.1 s), convex quadratic position-tracking objective
  and linear speed, acceleration and obstacle constraints.
- `include/local_planner/mpc_ros.hpp` and `msg/MpcRequest.msg`, `msg/MpcResult.msg`:
  typed request/result ROS interface, returning command proposals and predictions.
- `test/planar_mpc_test.cpp`: actual C++ optimizer regression executable.

The small Eigen implementation uses bounded ADMM iterations and checks primal
and dual residuals before accepting a result. The mathematical reference is the
[ADMM QP formulation](https://osqp.org/docs/solver/index.html); this is an
independent implementation, not the OSQP library. It outputs chassis
`linear.x` and `linear.y` for the omni-wheel base, with zero yaw command during
driving; the surrounding stages handle yaw. This is a kinematic controller, not
a wheel-force or detailed acceleration-dynamics model.

The conservative speed constraint is `|vx| + |vy| <= 0.1 m/s`, so diagonal motion
is slower than axial motion. Each component changes by at most 0.014 m/s per
0.1 s control step (vector acceleration at most approximately 0.198 m/s²).
The existing owner independently limits speed and acceleration.
The adapter additionally bounds accepted MPC commands by elapsed time since the
last dispatch; faster clouds or multiple replies cannot increase the ramp rate.
Immediate stop commands bypass the ramp.
Because limiting can alter direction during a reversal, the actual dispatched
command is also checked at 0.55 m clearance through the owner's 0.2 s command
lifetime. A failed check produces zero immediately.

MPC plans outside a **0.55 m** chassis-centre clearance. Up to 24 nearest returns
within the reachable horizon contribute separating half-plane constraints;
the predicted samples are then checked against **all** retained returns. A
new cloud also revalidates cached predictions between solves. Obstacles inside
the clearance or invalidated predictions produce zero. Conservative half-planes
can stop the robot instead of finding a route around an obstacle; there is no
global map, global search or guarantee of escaping a local dead end.

Commands/readiness checks run at 20 Hz, with C++ solving on front-cloud requests
(normally 10 Hz). Each request carries the source stamp, goal epoch, local PC
send time, local goal, actual last-dispatched velocity and retained obstacles.
Results must match a pending request and the current goal epoch. Unblocked
results require 21 finite planar prediction samples, first displacement matching
`0.1 * command`, and bounded per-step speeds and accelerations. Blocked results
require zero velocity and an origin-only prediction. Old, cancelled, malformed
or unmatched results cannot restore a target.

The solver has a **40 ms acceptance budget**, 4,000-iteration cap, and 150 ms cached
command expiry measured from request creation. Failure or a late/invalid result
clears the goal and publishes zero; freshness and owner readiness are checked
again on result receipt and before dispatch. Both backend processes are supervised.
The time
budget is checked between iterations and after solving, not a hard real-time
preemption guarantee. The owner's independent 200 ms command watchdog remains
the final stale-command guard.

The fixed path-lattice collision radius is 0.55 m; scale shrinking is disabled.
The additional command check evaluates a two-second arc using the nominal URDF
0.30 m chassis radius plus 0.10 m margin. Sensor/pose uncertainty, front blind
spots, dropped returns, low obstacles, slopes, drop-offs and extended arms are
not fully covered by this chassis-only local model. It has no global map or
route planner and can stop at a local dead end. Hardware collision avoidance,
braking distance and travel accuracy require supervised qualification.

`tests/test_astribot_local_planner.py` covers geometry, cloud schemas, map/local
frames, actual-goal arrival, collision envelopes, path ordering, owner gates,
rotation stages, final yaw, angle wraparound, positional drift and cancellation.
`tests/test_astribot_local_mpc.py` invokes the C++ regression binary and covers
request/result epochs, expiry, cancellation races, malformed predictions,
dispatch-time acceleration, cache invalidation and backend launch selection.
`tests/astribot_local_planner_ros_smoke.py --planner=mpc|library` runs the **real selected backend and adapter**
with synthetic observations on isolated domain 227: clear path, front obstacle,
cancel, cloud loss, odometry loss, simulated 0.2 m arrival and zero-range invalid
clouds, plus the complete turn–drive–turn sequence and cancelling final rotation.
It asserts no `/cmd_vel` publisher exists.
MPC additionally tests lateral drift correction and arrival without lattice output.
Both modes test loss of the C++ process producing zero and clearing the target.
No robot motion or SDK startup was used for these checks.

2026-11-21 software verification: 226 focused pytest cases passed, C++ planner
and RViz tool builds passed, RViz ctest 1/1 passed, launch `--check` and syntax
checks passed. All six isolated ROS smoke scenarios passed; the simulated
0.2 m target stopped at FAST-LIO x=0.170 m, within the requested 0.03 m tolerance.

The subsequent turn–drive–turn update passed 236 focused tests and all eight
isolated ROS scenarios, including complete initial rotation / library following /
final-yaw alignment, and cancellation during final rotation. The common readiness
gate checks cloud-to-pose timing in every stage. This is software/synthetic
verification; physical rotation accuracy and collision coverage remain unverified.

### Historical Python MPC verification (superseded by C++ implementation)

253 focused pytest cases passed; both backend launch preflights, Python compile
and shell syntax checks passed. The isolated ROS harness passed nine MPC and
eight library scenarios. The synthetic 0.2 m goal finished at x=0.173 m with MPC
and x=0.170 m with the library. MPC lateral-drift correction also completed the
goal. Source review found and fixed cached predictions overlooking a new cloud;
regressions cover new returns at 0.50 m and a future-path conflict at 0.60 m.

Before the subsequent MPC speed increase from 0.05 to 0.1 m/s, on this PC,
system `/usr/bin/python3`, `PYTHONNOUSERSITE=1` and
`OPENBLAS_NUM_THREADS=1`, 100 sequential solves per case produced:

| Input | Median solve ms | p95 ms | Maximum ms |
|---|---:|---:|---:|
| No retained obstacles | 0.270 | 0.350 | 0.742 |
| 100-point wall at x=0.60 m | 12.865 | 14.811 | 18.645 |
| 20,000 returns, radii 0.56–4 m | 10.054 | 11.210 | 11.950 |

All 300 solves passed the 40 ms acceptance budget. Goal was `[0.2, 0.02]` m;
each first command became the next previous-velocity input with warm starts.
The wall spans y=−1..1 m. Random-cloud generation used NumPy seed 12, uniform
angles over ±π and uniform radii. These measure the optimizer, not total ROS
latency or guaranteed timing under concurrent FAST-LIO/camera load.

Re-run current synthetic checks with each backend (built C++ messages required):

```bash
source /opt/ros/humble/setup.bash
export AMENT_CURRENT_PREFIX="$PWD/.xpolicy/ros2/astribot/install/local_planner"
source "$AMENT_CURRENT_PREFIX/share/local_planner/local_setup.bash"
unset AMENT_CURRENT_PREFIX
env -u FASTRTPS_DEFAULT_PROFILES_FILE -u FASTDDS_DEFAULT_PROFILES_FILE \
  PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  ROS_DOMAIN_ID=227 ROS_LOCALHOST_ONLY=0 ROS_LOG_DIR=/tmp/astribot-planner-test \
  .xpolicy/venvs/astribot-s1-py310/bin/python \
  tests/astribot_local_planner_ros_smoke.py --planner=mpc
# Repeat with --planner=library to exercise the alternative C++ backend.
```

The sandbox denied UDP socket creation; local shared-memory ROS transport still
completed the scenarios. Networked DDS and physical MPC motion/collision response
were not qualified. No hardware owner was started, enabled or commanded.

The subsequent MPC speed-limit update to **0.1 m/s** passed 72 focused planner
and base-command tests, all nine MPC synthetic ROS scenarios, the no-send launch
preflight and Python compilation. The synthetic 0.2 m goal stopped at x=0.173 m.
Acceleration, position/yaw tolerances and collision clearances remain unchanged;
the obstacle constraint search range expands automatically with the speed limit.
The existing owner already permits 0.1 m/s. Physical motion at this updated
planner speed has not been tested.

### Current C++ implementation verification (2026-11-21)

The package was rebuilt in Release mode, and its CTest optimizer test passed.
The Python optimizer module was removed. The source patch stored in XPolicy
matches the modified external package, including the C++ headers, typed messages
and optimizer test. Startup preflights now require this package for both backends.

Native C++ benchmark on this PC (`g++ -O2`, Eigen, 100 solves per case,
goal `[0.2,0.02]`, warm starts and previous returned commands):

| Retained obstacles | Median ms | p95 ms | Maximum ms |
|---|---:|---:|---:|
| None | 0.315 | 0.519 | 1.631 |
| 100-point wall at x=0.60 m | 10.677 | 11.524 | 13.250 |
| 20,000 radial returns from 0.56–4 m | 4.213 | 4.246 | 4.731 |

All 300 solves succeeded within the 40 ms budget. The random case used C++
`std::mt19937(12)` and uniform angles/radii. These numbers exclude ROS transport,
Python sensor conversion and concurrent FAST-LIO/camera load.

The owner yaw mapping was also changed from `[1,1,-1]` to `[1,1,1]` after the
operator reported positive ROS angular.z turning right. The sign is applied
once at the existing command-to-SDK boundary for both auto and joystick.
The intended convention remains positive-left/CCW; no odometry sign was changed.
This correction is unit-tested and still needs physical direction verification.

Final verification: **259 focused pytest cases**, the C++ CTest, both launch
preflights, Python compilation and shell syntax checks passed. Ten C++ MPC and
nine C++ library isolated ROS scenarios passed, including planner-process loss.
The synthetic 0.2 m goal stopped at x=0.178 m (MPC) and x=0.170 m (library).
Review findings on reply-rate acceleration, inconsistent predicted paths and
clipped reversal clearance were reproduced and covered by passing regressions.
No real robot owner or hardware motion was started for this change.
