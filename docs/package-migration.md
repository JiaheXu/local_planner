# Make local_planner the sole planner source tree

1. Move the tested ROS coordinator, geometry/following/stage helpers, launch,
   configuration, build/run scripts, planner tests and operator documentation to
   `/home/ubuntu/ros2_ws/src/local_planner`. MPC remains the existing C++ backend.
2. Remove runtime imports of XPolicy. The package owns its source-clock/odometry
   validation and a documented nominal front-LiDAR transform in its config.
   Preserve current topic, goal/cancel, speed, tolerance and safety semantics.
3. Install the package into the ROS workspace. Update XPolicy desktop supervision
   to call the external package launcher. Keep only robot-interface wiring and
   a short documentation link in XPolicy; remove duplicate planner sources,
   launch/config/tests, wrappers and the mirrored implementation patch.
4. Build and run package CTest/pytest and isolated ROS smoke scenarios for both
   backends; test XPolicy desktop invocation and owner contracts. Check installed
   launch paths and absence of runtime XPolicy dependencies. No hardware run.

The requested architecture is already explicit: no new planner behavior or
solver rewrite. Planner collision/goal checks move with the planner; XPolicy's
single hardware owner, command arbitration, watchdog and SDK sign mapping stay.

Completed on 2026-11-21. The package now owns its C++ backends, Python ROS
coordinator/helpers, configuration, launch, scripts, tests and documentation.
The old XPolicy planner copies, implementation patch and planner test files
were removed. XPolicy desktop supervision invokes this package's launcher only.
The active install is `/home/ubuntu/ros2_ws/install/local_planner`.

Verification: CTest 2/2 (native MPC test and 61 Python tests), both source and
installed launcher preflights, 212 XPolicy robot-interface tests, ten MPC and
nine library synthetic ROS scenarios passed. MPC stopped a synthetic 0.2 m goal
at 0.173 m; library at 0.170 m. The installed coordinator imports without any
XPolicy directory on its Python path. Nominal package geometry matches the
robot observation contract. Read-only review found no migration blocker.

No SDK owner, hardware enable or physical robot motion was started. Local
shared-memory ROS testing does not qualify networked DDS or physical navigation.
