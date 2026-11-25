# Manual Orin MPC dispatch, collision checking disabled

## Basic Twist control (manual test mode)

Use `--basic-control` on **both** launch commands to remove transport leases,
session IDs and goal acknowledgments:

```text
PC RViz -> PoseStamped -> Orin Python coordinator + C++ MPC
                       <- onboard FAST-LIO odometry
Orin Twist -> PC /cmd_vel (domain 0) -> existing SDK owner -> mobile base
```

Keep Start Astribot running on the PC. Stop any previous planner/relay before
switching modes. Launch the PC relay first:

```bash
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_orin_dispatch_bridge.sh --basic-control
```

Then launch the planner on Orin:

```bash
bash /home/astribot/ros2_ws/src/local_planner/scripts/run_orin_dispatch.sh --ignore-lidar --basic-control
```

The relay automatically enables the existing `/cmd_vel` owner once it receives
fresh zero commands and fresh hardware feedback. No Enable click is needed for
that initial startup. Goals use `/cocomatrix/local_planner/goal_pose`; velocities
use `/cocomatrix/local_planner/cmd_vel`, both on domain 25. Owner status returns
to the coordinator as ordinary JSON, without leases. Send a new RViz goal after
the relay reports `armed /cmd_vel`.

The owner remains armed after completion/cancel while fresh zeros continue.
After a planner restart, fresh zeros allow recovery from a command timeout or
missing publisher, without restarting the PC relay. An explicit disable,
joystick selection or fault still takes precedence. A deliberate new basic-relay
launch can enable an initially disabled automatic owner; it does not switch away
from joystick control or clear a fault.

Collision checking and all cloud subscriptions are disabled. Speed limits,
fresh odometry, fresh hardware feedback, the existing 200 ms command watchdog,
and a single SDK writer remain. The relay never repeats a cached velocity.
Command writers use volatile depth-1 QoS with a 200 ms DDS lifespan. Plain Twist
has no source timestamp and this mode has no application-level replay/goal lease
validation. PC validates RViz goal age on its own clock; the forwarded goal has
a 2 s DDS lifespan and is restamped on Orin to avoid comparing host clocks.

Logs append to `ros2_ws/log/orin-dispatch-bridge.log` on PC and
`ros2_ws/log/orin-dispatch-planner.log` on Orin. No live robot process is started
by installing these files. Use the commands above manually.

Verified on 2026-11-25: 195 local unit tests; synthetic PC relay tests for initial
enable, plain Twist, goal/cancel forwarding, loss of stream, automatic recovery
after planner restart, duplicate publishers and manual disable; and the Orin
ARM64 MPC test for basic-mode commands, cancel and the next goal. Existing leased
mode also passes its Orin test. These checks used isolated localhost domains,
not the live robot. Physical movement with basic mode has not been tested.

## Existing leased mode

The hardware owner still consumes `geometry_msgs/msg/Twist` on `/cmd_vel` in
**domain 0**, as configured by XPolicy `base_commands.yml`. Publishing a Twist
on domain 25 alone does not reach that subscriber. No SDK message change is needed.

The supported route is:

```text
PC RViz PoseStamped (domain 0)
  -> existing XPolicy PlannerBridge goal lease
  -> Orin /cocomatrix/local_planner/goal (domain 25)
  -> Python coordinator + C++ MPC, using onboard chassis odometry
  -> /cocomatrix/local_planner/command (leased String envelope)
  -> PC PlannerBridge validates lease/session/goal/owner generation
  -> /cmd_vel (Twist, domain 0)
  -> existing XPolicy base owner -> existing robot SDK
```

The Orin `cmd_vel_unleased` topic is a diagnostic proposal only. It is never
forwarded to hardware. There is exactly one domain-0 command publisher: the
leased relay. The relay reuses XPolicy's existing `PlannerBridge`/`CommandLease`;
it does not create a hardware writer or call the vendor SDK.

## Manual launch

Keep the existing Start Astribot base-owner session running. Stop the previous
shadow planner with **Ctrl+C** in its Orin terminal before launching dispatch.

On the PC, replace the shadow goal bridge with:

```bash
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_orin_dispatch_bridge.sh
```

On Orin:

```bash
bash /home/astribot/ros2_ws/src/local_planner/scripts/run_orin_dispatch.sh --ignore-lidar
```

The PC relay uses the existing checked auto-enable policy: the owner must be in
automatic mode, stopped, have fresh hardware feedback, and receive fresh zero
commands from the sole publisher. It will not override manual disable, joystick
selection, faults or a conflicting publisher. Startup must begin with no goal;
send a **new RViz goal** after enable. The previous shadow goal is not resumed.

This launch explicitly disables planner collision checking and cloud subscriptions.
Velocity/acceleration limits, odometry freshness, owner validation and command
leases remain active. MPC replans every 100 ms. No boot service is installed or
enabled; the planner remains manually launched.

The existing transport pins publisher identities. Restart the PC dispatch relay
when restarting/replacing an Orin dispatch coordinator; an identity change is
latched as a transport conflict until the relay restarts. Stop the old relay
before starting another; the PC launchers share a process lock.

## Logs and checks

Orin runtime log: `/home/astribot/ros2_ws/log/orin-dispatch-planner.log`.
PC relay log for the prepared background run: `/tmp/astribot-orin-dispatch-bridge.log`.

Validated on 2026-11-25 with synthetic inputs only:

- PC localhost domains 211/212: shadow status cannot enable, checked enable after
  zero commands, leased goal and nonzero Twist forwarding, cancellation rejecting
  the old goal.
- Orin localhost domain 213: ARM64 C++ MPC + leased Python adapter without any
  LiDAR input, goal acknowledgment, nonzero proposals, cancel and delayed-goal
  rejection. No `/cmd_vel` publisher or hardware writer in that test.
- Local planner and existing transport unit suites: 196 tests passed.

Physical movement under this new dispatch launch has not been tested by the agent.
