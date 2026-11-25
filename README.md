# Local planner

This ROS 2 package owns the complete Astribot local-planning pipeline: the C++
MPC/library backends, goal stages, path following, collision checks, sensor/pose
validation, launch/configuration, build scripts and tests. It communicates with
XPolicy through ROS topics only; it imports no XPolicy modules or robot SDK.

The Python MPC default is **simultaneous translation and target-yaw tracking**,
with a 0.1 m/s MPC translation limit and 0.03 m / 0.05 rad goal tolerances.
Translation slows to at most `0.6 * distance_m` m/s near the goal, then zero
inside 3 cm; yaw correction continues until the heading tolerance is met.
Cached XY commands are re-expressed in the current chassis frame while turning.
The library backend and bounded reverse-test mode retain their staged behavior.
Forward, backward and sideways goals start without turning toward the position.
The library backend and native C++ coordinator retain their turn-first sequence.
The original library backend remains selectable. MPC is C++. For current
debugging, the default ROS coordinator is Python; `--cpp` selects the updated
native coordinator explicitly.

## Build and check

```bash
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/build.sh
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh --check
```

The build installs into `ros2_ws/install/local_planner`, then runs native CTest and
Python regression tests. Native coordinator tests use isolated localhost DDS
domain 228 and shadow commands; no robot owner or hardware is started. Source config
changes require rebuilding to update the installed copy.

## Run

```bash
# Private shadow output; cannot command XPolicy's owner.
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh
# Override the installed MPC default for this invocation.
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh --planner=library
# Python MPC test: no cloud messages or obstacle avoidance; odometry still required.
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh --python-debug --ignore-lidar
```

`--dispatch` selects `/cmd_vel`. XPolicy's **Start Astribot** starts the owner and
waits up to 60 seconds for a single automatic command publisher. Launch this
planner manually with `--dispatch`; startup automatically enables Auto Base
Control after the owner's checks pass. Wait for `enabled: true`, then send a new
RViz goal. The Python planner with `--dispatch` maintains automatic base enable:
after publisher loss or command timeout, it requests enable again once the goal
is cleared, one publisher is present, zero commands/hardware feedback are fresh
and the stop is confirmed. Transient service refusals retry at most once per
second. Faults, manual disables and joystick selection still require explicit
handling. **Cancel Goal** clears the target; **Stop Astribot** stops the desktop
session. Stop the manually launched planner separately with Ctrl+C. Do not start
a second planner or Move Forward publisher in the same dispatch session.

After sourcing this package's install, the standard ROS launch is:

```bash
ros2 launch local_planner astribot.launch.xml dispatch:=false planner_backend:=mpc
```

## Ownership and ROS interface

The C++ coordinator in `src/astribotCoordinator.cpp` handles
ROS inputs, timestamp validation, goal stages, collision gates and command output;
`src/localPlanner.cpp` provides the MPC/library backend. XML launch starts both.
The Python implementation in `python/` is the current default for debugging.
`scripts/run_astribot.sh --cpp` selects C++, and `--python-debug` selects Python;
the XML launch starts
exactly one coordinator. Both versions use odometry and cloud timestamps and do
not subscribe to LiDAR IMU. FAST-LIO still uses IMU independently.

| This package | XPolicy robot interface |
|---|---|
| Goal sequencing, MPC/library choice, path following | Sensor relay and robot observation topics |
| Front/rear cloud selection, coordinate conversion, collision checks | Sole SDK owner and hardware feedback |
| FAST-LIO pose/reference freshness checks | Command-source selection, enable/disable, watchdog |
| `/cmd_vel` or private shadow Twist publication | Final command validation and SDK yaw mapping |

Collision input defaults to **front only** (`collision_lidar: front` in
`config/astribot.yaml`): `/sdk/astribot_lidar/front_scan`. Python collision
avoidance is enabled (`python_collision_checks_enabled: true`): front obstacles
feed the planner and gate both translation and in-place turns. Rear data
cannot select a different source or block a goal. Explicit `collision_lidar: rear`
selects rear only; `auto` enables front-preferred selection with rear fallback
at a new goal. Rebuild/restart after changing this configuration.
Front-only mode stops if front data is unusable, even when rear is healthy. The selected feed is
pinned per goal; loss stops/cancels instead of automatically resuming on another
feed. Status reports `collision_lidar` and `collision_topic`.
Other inputs are
`/fast_lio/chassis/odometry`, `/astribot_viz/goal_pose`,
`/astribot_viz/goal_cancel` and `/astribot/base/status`. Planner output/status and
predicted paths remain under `/astribot/local_planner/`.

Configuration lives in `config/astribot.yaml` and `config/astribot_geometry.yaml`.
The geometry file pins the existing nominal front mount and the current vendor
rear-cloud-in-front-coordinates convention with source provenance;
it must agree with the robot observation-frame contract when mounts change.
There is no runtime read of XPolicy's robot profile.

See [Astribot operation and verification](docs/astribot.md) for limits and tests.
