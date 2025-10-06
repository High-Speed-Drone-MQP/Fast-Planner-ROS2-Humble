# Fast-Planner ROS 2 (Humble) — Build and Package Overview

This repo contains a ROS 2 (Humble) port of Fast-Planner. Below are prerequisites, build commands, and a short overview of each package.

## Prerequisites
- Ubuntu 22.04, ROS 2 Humble, colcon
- Build tools: cmake, g++, make
- Eigen3 (usually preinstalled with ROS 2)
- PCL (for visualization utilities): `sudo apt-get install libpcl-dev`
- Optional: quadrotor_msgs (only for the example `traj_server` publisher; not required for the planner itself)
- NLopt (for bspline optimization)
  - Option A (use the bundled source):
    - Build once:
      ```bash
      cd Fast-Planner-ROS2-Humble/nlopt
      cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
      cmake --build build -j
      ```
    - The build will output `libnlopt.so` and headers under `nlopt/build` which are auto-discovered by `bspline_opt`.
  - Option B (system package): `sudo apt-get install libnlopt-dev`

## Build
Run from the workspace root (the directory that has this `Fast-Planner-ROS2-Humble` folder):

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the core Fast-Planner packages explicitly (uav simulator packages not yet migrated from ROS1 Noetic)
colcon build --symlink-install \
  --packages-select \
  bspline bspline_opt path_searching plan_env poly_traj traj_utils plan_manage \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Notes
- `quadrotor_msgs` is optional and only used by `traj_server`. The planner builds without it.
- If you updated the repo or CMake, clean builds help:
  ```bash
  rm -rf build install log
  colcon build --symlink-install --packages-up-to plan_manage
  ```

## Run
After building, source the workspace and run the planner node:
```bash
source install/setup.bash
ros2 run plan_manage fast_planner_node
```
The planner expects:
- Odometry on `/odom_world` (nav_msgs/Odometry)
- Waypoints on `/waypoint_generator/waypoints` (nav_msgs/Path) or parameterized waypoints
- An environment map from `plan_env` (e.g., PointCloud2 processed into an SDF/EDT)

`traj_server` is an example publisher from planned Bspline trajectories to a position command; for PX4, you likely want your own bridge node that subscribes to `plan_manage/msg/Bspline` and publishes PX4 setpoints.

## Package Overview
- `bspline`: Core B-spline math utilities (NonUniformBspline) used across the planner.
- `bspline_opt`: Trajectory optimization (smoothing, feasibility, constraints) on B-splines, backed by NLopt.
- `path_searching`: Front-end search modules (e.g., kinodynamic A* and topological planning helpers).
- `plan_env`: Environment representation, signed distance field (SDF) and Euclidean distance transform (EDT) queries.
- `poly_traj`: Minimum-snap polynomial trajectory utilities (global or coarse trajectories).
- `traj_utils`: Visualization helpers for paths, splines, and trajectories.
- `plan_manage`: High-level orchestration and nodes:
  - Custom message `plan_manage/msg/Bspline` (builtin_interfaces/Time, geometry_msgs types)
  - Nodes:
    - `fast_planner_node`: main planner executable (kino/topo FSMs, manager)
    - `traj_server` (optional): example publisher that converts B-spline to a position command message

## Common Issues
- NLopt not found: ensure you built `nlopt/build` or installed `libnlopt-dev`.
- Missing quadrotor_msgs: not required; `traj_server` compiles without it. If you want to skip building `traj_server` entirely, we can add a build option.
- Header not found between local packages: this port exports include dirs and libraries properly; prefer building all listed packages together with colcon.
