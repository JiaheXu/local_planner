# Astribot local planner integration

User request: adapt `/home/ubuntu/ros2_ws/src/local_planner` to front LiDAR collision avoidance and FAST-LIO odometry, retaining XPolicy as the robot interface. Nav2 stays removed.

## Approach

Reuse the supplied C++ path lattice and its existing assets. Add an Astribot-owned ROS adapter/follower instead of using the legacy follower, whose odometry subscription is disabled and whose obstacle response only increases yaw gain. The adapter consumes XPolicy's front PointCloud2 and rear IMU/chassis odometry, transforms the cloud using the nominal model mount, and supplies paired chassis-frame scan/goal inputs to the lattice. Paths are matched to their input timestamps and transformed to map coordinates before following them.

The user confirmed RViz goal/cancel as the destination interface, then requested
turn–drive–turn execution while retaining the library and deferring MPC. First
turn toward the destination (0.05 rad tolerance), follow library paths to within
0.03 m, then turn to the arrow yaw (0.05 rad). Both rotations command zero
translation. Goals behind the starting heading are handled by turning first;
reverse driving is not commanded. Position drift during final alignment restarts
the approach. No automatic movement without a newly accepted goal.

## Control and failure behavior

The adapter is the only planner publisher of Twist `/cmd_vel` in explicit dispatch mode; default standalone mode uses a private shadow topic. It never enables the base or uses the SDK. The formal owner remains responsible for source selection, hardware writes and stop confirmation. A selected joystick source, disabled/faulted owner, owner-generation change, publisher conflict, expired/invalid cloud/odometry/path, or cancel clears the target and emits zero. Hardware runs are not part of implementation.

Keep 0.05 m/s translation and 0.1 rad/s yaw caps. Use the nominal 0.30 m chassis radius plus 0.10 m clearance, fixed lattice scale 1.0, no reverse, bounded lookahead and an independent swept-command collision check. This is a local planner, not global route planning; front occlusion, low obstacles, extended arms and nominal extrinsic calibration remain hardware qualification limits.

## Integration and verification

Add an explicit local-planner launcher alongside the existing observation system; do not silently start an additional `/cmd_vel` publisher with the existing Move Forward tool. RViz displays the selected local path. Source updates in the external local_planner checkout are preserved as a reproducible patch in XPolicy. Build only the updated package; verify geometry, frame conversion, cancel, stale inputs, publisher/owner gates, goal distance and obstacles with offline and isolated ROS tests.
