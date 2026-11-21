# Local planner

This ROS 2 package owns the complete Astribot local-planning pipeline: the C++
MPC/library backends, goal stages, path following, collision checks, sensor/pose
validation, launch/configuration, build scripts and tests. It communicates with
XPolicy through ROS topics only; it imports no XPolicy modules or robot SDK.

The default is **rotate toward goal → C++ MPC drive → rotate to target yaw**,
with a 0.1 m/s MPC translation limit and 0.03 m / 0.05 rad goal tolerances.
The original library backend remains selectable. MPC is implemented in C++;
the package's ROS coordinator and geometry helpers are Python.

## Build and check

```bash
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/build.sh
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh --check
```

The build installs into `ros2_ws/install/local_planner`, then runs CTest and
the package's Python tests. It never starts a robot or ROS nodes. Source config
changes require rebuilding to update the installed copy.

## Run

```bash
# Private shadow output; cannot command XPolicy's owner.
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh
# Override the installed MPC default for this invocation.
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_astribot.sh --planner=library
```

`--dispatch` selects `/cmd_vel`. XPolicy's **Start Astribot** desktop supervisor
calls this package entry point with `--dispatch`; it starts the robot owner in
HOLD. Select **Auto Base Control** before a new RViz goal. **Cancel Goal** clears
the target; **Stop Astribot** terminates the supervised processes. Do not start
a second planner or Move Forward publisher in the same dispatch session.

After sourcing this package's install, the standard ROS launch is:

```bash
ros2 launch local_planner astribot.launch.py dispatch:=false planner_backend:=mpc
```

## Ownership and ROS interface

| This package | XPolicy robot interface |
|---|---|
| Goal sequencing, MPC/library choice, path following | Sensor relay and robot observation topics |
| Nominal front-cloud conversion, collision checks | Sole SDK owner and hardware feedback |
| FAST-LIO pose/reference freshness checks | Command-source selection, enable/disable, watchdog |
| `/cmd_vel` or private shadow Twist publication | Final command validation and SDK yaw mapping |

Inputs remain `/sdk/astribot_lidar/front_scan`, front/back IMUs,
`/fast_lio/chassis/odometry`, `/astribot_viz/goal_pose`,
`/astribot_viz/goal_cancel` and `/astribot/base/status`. Planner output/status and
predicted paths remain under `/astribot/local_planner/`.

Configuration lives in `config/astribot.yaml` and `config/astribot_geometry.yaml`.
The geometry file pins the existing nominal front mount with source provenance;
it must agree with the robot observation-frame contract when mounts change.
There is no runtime read of XPolicy's robot profile.

See [Astribot operation and verification](docs/astribot.md) for limits and tests.
