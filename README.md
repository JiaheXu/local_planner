# local_planner

C++ local planning package.

Builds the `localPlanner` and `pathFollower` executables used for local obstacle-aware path planning and path following from ROS sensor, odometry, and map data.

## Build targets

```bash
ros2 run local_planner localPlanner
ros2 run local_planner pathFollower
```

## Key files

- `src/localPlanner.cpp`: local planner node.
- `src/pathFollower.cpp`: path following node.
- `launch/local_planner.launch`: launch entrypoint.
- `paths/`: path and correspondence data.
