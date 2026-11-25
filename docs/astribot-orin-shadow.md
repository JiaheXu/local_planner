# Orin MPC and PC RViz goal connectivity

The planner is deployed at `astribot@192.168.0.11:/home/astribot/ros2_ws/src/local_planner`.
It runs the Python coordinator and ARM64 C++ MPC on Orin, with onboard FAST-LIO
chassis odometry. The launch below is **shadow-only**, with collision checks
disabled and no `/cmd_vel` publication or owner-enable request.

## Start

On the PC, alongside the existing RViz:

```bash
bash /home/ubuntu/ros2_ws/src/local_planner/scripts/run_orin_goal_bridge.sh
```

On Orin:

```bash
bash /home/astribot/ros2_ws/src/local_planner/scripts/run_orin_shadow.sh
```

Each launcher has a process lock to prevent duplicate instances. Do not also
start the older `astribot_planner_bringup/run.sh`: that launch uses the separate
leased dispatch transport. No boot service was enabled by this deployment.

## Route

```text
PC RViz, domain 0
  /astribot_viz/goal_pose (PoseStamped)
    -> PC domain_bridge, config/orin_shadow_goals.yaml
    -> domain 25 /cocomatrix/local_planner/goal_pose
    -> Orin astribot_orin_coordinator
    -> Orin astribot_orin_mpc (10 Hz requests)
    -> /cocomatrix/local_planner/cmd_vel_shadow

Orin /cocomatrix/fast_lio/front/odometry
    -> existing astribot_planner_bringup/odometry.py chassis transform
    -> /cocomatrix/local_planner/chassis_odometry
    -> coordinator

Orin status, path_map, cmd_vel_shadow
    -> PC domain_bridge, config/orin_shadow_feedback.yaml
    -> domain 0 /astribot/local_planner/{status,path_map,cmd_vel_shadow}
```

Goal cancellation travels from `/astribot_viz/goal_cancel` to
`/cocomatrix/local_planner/goal_cancel`. Bridge routes do not include `/cmd_vel`
or base-owner services. The Orin source and PC source geometry files were verified
identical before launch. The onboard chassis converter does not depend on a
round trip through the PC odometry bridge.

The PC launcher uses the same default DDS transport as the existing RViz.
An optional `PLANNER_DDS_PROFILE` can select another profile. In this deployment,
the camera UDP-only profile did not discover local RViz; the default transport
did. Orin uses its existing robot-system DDS profile.

## Validation on 2026-11-25

- ARM64 build and all four CTest suites passed, including the Python suite.
- A 12-second onboard odometry sample measured 10.0 Hz, maximum interval 0.1203 s,
  and no intervals over 1 s. This is a short measurement, not a long-term guarantee.
- A guarded PC probe published a 10 cm goal on RViz's actual topic after matching
  both RViz and the bridge. Orin accepted it and returned active status in 0.138 s.
  A nonzero shadow command followed; the reported C++ solve took 2.156 ms.
- The cancellation probe returned the coordinator to inactive. Neither domain
  had a `/cmd_vel` publisher during the test. No robot motion was commanded.
- The user's actual RViz click was also accepted on Orin at map position
  `(-0.599, -0.674)` m, yaw `0.527` rad. The coordinator continued producing
  backward shadow commands `[-0.100, 0.000, 0.000]` while tracking that goal.

The initial one-shot probe preceded DDS matching and was not delivered. Wait for
the goal publisher to match the bridge before sending a one-shot test message.

## Logs and backup

Orin build: `/home/astribot/ros2_ws/log/local-planner-build.log`.
Orin runtime: `/home/astribot/ros2_ws/log/orin-shadow-planner.log`.
PC bridge: `/tmp/astribot-orin-goal-bridge.log`.
PC connectivity probe: `/tmp/orin-link-probe.log`.

Pre-sync Orin source backup:
`/home/astribot/ros2_ws/backups/local_planner-20261125-113411.tar.gz`.
The sync preserved Orin-only files and did not copy PC build products.

The running background launch PIDs are recorded in
`/home/astribot/ros2_ws/log/orin-shadow-planner.pid` on Orin and
`/tmp/astribot-orin-goal-bridge.pid` on the PC. Send `SIGINT` to the recorded
Orin launch PID or `SIGTERM` to the PC wrapper PID to stop the respective run.
