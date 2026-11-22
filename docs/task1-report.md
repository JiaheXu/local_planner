# Astribot planner core port

The C++14 header `include/local_planner/astribot_core.hpp` implements fixed-baseline
IMU qualification, stamped odometry and collision observations, cloud field decoding,
geometry loading, sweep validation, goal phases, path sessions, and bounded following.
The parent task owns the coordinator executable, publisher and owner checks, backend
response validation, CMake wiring, and integration evidence.

`test/astribot_core_test.cpp` was compiled with GCC, C++14, optimization and
`-DNDEBUG`, using ROS 2 Humble headers, Eigen, yaml-cpp, and ament_index_cpp.
The resulting executable passed against the staged geometry configuration exposed
through an isolated ament package index. It checks source and receipt timing,
duplicate/reversed stamps, catch-up, 500 ms collision freshness, organized
little-/big-endian data with row padding and offset fields, invalid returns,
rear geometry, motion envelopes, path epochs and turn-drive-turn phases.

This standalone run used no ROS node or hardware. Package compilation and ROS
integration results are tracked by the parent task.
