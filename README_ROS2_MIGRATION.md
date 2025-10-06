ROS1 -> ROS2 Migration Checklist for Fast-Planner-ROS2-Humble

This document lists the actionable checklist to migrate the repository from ROS1 to ROS2 (Foxy/Galactic/Humble style). It focuses on the kinds of code, build, and packaging changes required and calls out files found that use ROS1 APIs. Use it as a step-by-step plan and mark items as you complete them.

Quick notes
- This repo currently uses ROS1 APIs extensively (ros::init, ros::NodeHandle, ros::Publisher/Subscriber, ros::Timer, ros::Time, ROS_INFO/ROS_WARN/ROS_ERROR macros, etc.).
- Migration requires both source code edits and build/package updates (CMakeLists.txt and package.xml -> ament_cmake and package.xml format for ROS2).
- Choose a ROS2 distribution target (e.g., Humble). This checklist assumes ROS2 rclcpp-style APIs and ament_cmake build system.

Top-level migration steps (high level)
- Update package build system to ament_cmake: update CMakeLists.txt and package.xml for each package.
- Replace ROS1 runtime APIs with ROS2 equivalents (rclcpp, rclcpp::Logger, rclcpp::Clock/Time/Duration, timers, publishers/subscribers, parameters, Node lifecycle where needed).
- Replace ROS logging macros with RCLCPP_* macros or node-based logging.
- Replace ros::spin / spinOnce with rclcpp::spin / executor usage.
- Adapt message types and include paths if messages are custom or use different namespaces/types in ROS2.
- Rework launch files (ROS1 launch -> ROS2 launch in Python).
- Update any scripts (bash/python) that rely on roslaunch/rosrun to use ros2 run / ros2 launch.
- Update CI and documentation to reference ROS2 environment and workspace overlay instructions.

Repository-specific checklist (files found with ROS1 usage)
- fast_planner/bspline/src/non_uniform_bspline.cpp
  - Remove/include replacement: do not include <ros/ros.h> if logging is not needed. If you need logging, include <rclcpp/rclcpp.hpp> and use RCLCPP_ERROR or rclcpp::get_logger("...") style.
  - Replace ROS_ERROR_COND with conditional logging: if (cond) RCLCPP_ERROR(rclcpp::get_logger("non_uniform_bspline"), "...format...", args...);
  - Verify build: update bspline/CMakeLists.txt to find_package(rclcpp REQUIRED) only if source uses rclcpp.

- fast_planner/bspline_opt/
  - Headers: `include/bspline_opt/bspline_optimizer.h` includes <ros/ros.h>
  - Source: `src/bspline_optimizer.cpp` uses ros::NodeHandle in setParam and ros::Time, ROS_INFO/ROS_WARN
  - Changes:
    - Convert `setParam(ros::NodeHandle& nh)` to accept `rclcpp::Node::SharedPtr node` or accept `rclcpp::Node&` and use node->get_parameter or node->declare_parameter.
    - Replace ros::Time with rclcpp::Time and ros::Duration with rclcpp::Duration.
    - Replace logging macros with RCLCPP_* and pass logger or use node->get_logger().
    - Update CMakeLists.txt and package.xml to ament_cmake and find_package(rclcpp REQUIRED)

- fast_planner/poly_traj/src/traj_generator.cpp
  - Heavy use of ros::init, ros::NodeHandle, publishers, subscribers, spin, Time, Duration
  - Changes:
    - Convert main to use rclcpp::init and create a rclcpp::Node (or a class that inherits rclcpp::Node).
    - Replace ros::Publisher and ros::Subscriber with rclcpp::Publisher and rclcpp::Subscription and appropriate create_publisher/create_subscription calls with QoS.
    - Replace ros::spin()/ros::spinOnce() with rclcpp::spin(node) or use an executor for multiple nodes/threads.
    - Replace ros::Duration::sleep() with rclcpp::sleep_for(std::chrono::duration)
    - Replace Time/Duration uses as above.

- fast_planner/path_searching/
  - `include/*` and `src/astar.cpp`, `src/topo_prm.cpp`, `include/path_searching/kinodynamic_astar.h` reference ros::NodeHandle and ros::Time
  - Changes: migrate setParam/ init functions to accept rclcpp::Node::SharedPtr or use parameter handlers; replace ros::Time with rclcpp::Time.

- fast_planner/plan_env/src/obj_generator.cpp
  - Uses ros::Publisher, ros::Timer, createTimer, ros::TimerEvent callbacks, ros::spin
  - Changes: replace with rclcpp timers (create_wall_timer or create_timer) and callback signature changes: timers don't pass ros::TimerEvent — pass closure-captured state or use steady_clock timestamp from rclcpp::Clock if needed.

- fast_planner/plan_manage/
  - `src/fast_planner_node.cpp` uses ros::init, NodeHandle, spin
  - FSM files (topo_replan_fsm.cpp, kino_replan_fsm.cpp) use nh.createTimer and callbacks using ros::TimerEvent — convert to rclcpp timers and convert member function signatures accordingly.
  - Replace ROS_WARN/ERROR with RCLCPP_WARN/ERROR.

- Other changes across repo
  - Replace all occurrences of ROS_INFO, ROS_WARN, ROS_ERROR, ROS_DEBUG, ROS_ERROR_COND: Use RCLCPP_INFO(logger, "..."), or RCLCPP_ERROR, etc.
  - Replace ros::Time::now(), ros::Duration with rclcpp::Clock and rclcpp::Time. For sleeping, use rclcpp::sleep_for. For measuring durations, rclcpp::Time - rclcpp::Time returns rclcpp::Duration with to_seconds().
  - Update any use of ros::TimerEvent: ROS2 timers typically call void() normally (no event arg). If you need timing info, use rclcpp::Clock inside the callback.
  - Replace parameter APIs: nh.param and nh.getParam -> node->get_parameter or node->declare_parameter + node->get_parameter.
  - Update message include paths if any are different in ROS2 (std_msgs, geometry_msgs include paths are similar but the build dependency listing differs).

Build system and packaging changes (per package)
- Update package.xml to ament format and add dependencies: rclcpp, std_msgs, geometry_msgs, builtin_interfaces, etc. Remove <build_depend> rosconsole or roscpp specific tags and add ament_cmake specifics.
- Replace find_package(catkin REQUIRED COMPONENTS ...) with ament_cmake equivalents in CMakeLists.txt. Example minimal changes:
  - find_package(ament_cmake REQUIRED)
  - find_package(rclcpp REQUIRED)
  - find_package(std_msgs REQUIRED)
  - add_executable(...)
  - ament_target_dependencies(<target> rclcpp std_msgs ...)
  - install(TARGETS ... DESTINATION lib/${PROJECT_NAME})
  - ament_package()
- Remove catkin_package and catkin specific macros

Launch files and scripts
- Convert launch files to Python-based ROS2 launch syntax. Replace roslaunch/rosrun usage in scripts with ros2 launch / ros2 run.

Testing and verification
- For each package:
  - Compile and fix compile errors introduced by API changes.
  - Run node locally with ros2 run and verify expected behavior.
  - Replace ros::Duration sleeps used in main loops with rclcpp::sleep_for or timers; ensure correct timing semantics are preserved.

Examples and concrete replacements
- Replace: #include <ros/ros.h>
  With: #include <rclcpp/rclcpp.hpp> (if you need node/logging/time usage) or remove include entirely if not needed

- Replace: ROS_ERROR_COND(ratio > 2.0, "max vel: %lf, max acc: %lf.", max_vel, max_acc);
  With:
  if (ratio > 2.0) RCLCPP_ERROR(rclcpp::get_logger("non_uniform_bspline"), "max vel: %lf, max acc: %lf.", max_vel, max_acc);

- Replace node init in main():
  ros::init(argc, argv, "traj_generator");
  ros::NodeHandle node;
  With:
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("traj_generator");

- Replace publishers/subscribers creation:
  ros::Publisher pub = node.advertise<std_msgs::...>("topic", 10);
  With:
  auto pub = node->create_publisher<std_msgs::msg::...>("topic", 10);

- Replace ros::spin():
  ros::spin();
  With:
  rclcpp::spin(node);

- Replace ros::Timer and callback signatures:
  ros::Timer update_timer = node.createTimer(ros::Duration(1/30.0), updateCallback);
  void updateCallback(const ros::TimerEvent& e) { ... }
  With:
  auto timer = node->create_wall_timer(std::chrono::milliseconds(33), std::bind(&Class::updateCallback, this));
  void updateCallback() { /* use rclcpp::Clock to get time if needed */ }

Checklist - per-file action items (short)
- [ ] fast_planner/bspline/CMakeLists.txt: review and convert to ament_cmake if any source needs rclcpp (or leave as-is if pure library without ROS deps).
- [ ] fast_planner/bspline/src/non_uniform_bspline.cpp: remove <ros/ros.h>, replace ROS_ERROR_COND with RCLCPP or standard error handling.
- [ ] fast_planner/bspline_opt: port setParam and time/logging uses
- [ ] fast_planner/poly_traj: convert node boilerplate (main) to rclcpp and adjust publishers/subscribers
- [ ] fast_planner/plan_env: convert timers and publishers
- [ ] fast_planner/plan_manage: convert NodeHandle-based initializations and timers
- [ ] fast_planner/path_searching: convert timing and logging
- [ ] All: replace ROS logging macros and time/duration APIs
- [ ] All: update package.xml and CMakeLists.txt to ament_cmake and add proper find_package/ament_target_dependencies
- [ ] Launch: rewrite ROS1 launch to ROS2 Python launch files
- [ ] Scripts: replace rosrun/roslaunch usage with ros2 run/ros2 launch

Verification steps
- After code changes, for each package run a build in a ROS2 workspace (colcon build) and fix issues.
- Run nodes via ros2 run or ros2 launch and observe logs to ensure nodes start and publishers/subscribers/timers behave correctly.

If you want, I can next:
- Produce the exact file list of all ROS1 occurrences (full file paths). OR
- Start porting one file (for example `fast_planner/bspline/src/non_uniform_bspline.cpp`) to remove ROS1 dependencies and replace with ROS2 logging.


End of checklist.

Full list of files containing ROS1 usages (found by code search)

Note: this list was generated from a workspace-wide search for common ROS1 symbols and includes. It includes header/source files, package manifests, and generated build artifacts that reference ROS1.

Files (relative paths):
- fast_planner/bspline/src/non_uniform_bspline.cpp
- fast_planner/bspline/CMakeLists.txt
- fast_planner/bspline_opt/include/bspline_opt/bspline_optimizer.h
- fast_planner/bspline_opt/src/bspline_optimizer.cpp
- fast_planner/bspline_opt/package.xml
- fast_planner/poly_traj/src/traj_generator.cpp
- fast_planner/poly_traj/CMakeLists.txt
- fast_planner/path_searching/include/path_searching/astar.h
- fast_planner/path_searching/include/path_searching/kinodynamic_astar.h
- fast_planner/path_searching/src/astar.cpp
- fast_planner/path_searching/src/topo_prm.cpp
- fast_planner/plan_env/src/obj_generator.cpp
- fast_planner/plan_env/include/plan_env/sdf_map.h
- fast_planner/plan_env/src/sdf_map.cpp
- fast_planner/plan_env/include/plan_env/obj_predictor.h
- fast_planner/plan_env/src/obj_predictor.cpp
- fast_planner/plan_env/include/plan_env/edt_environment.h
- fast_planner/plan_manage/src/fast_planner_node.cpp
- fast_planner/plan_manage/src/topo_replan_fsm.cpp
- fast_planner/plan_manage/src/kino_replan_fsm.cpp
- fast_planner/plan_manage/src/planner_manager.cpp
- fast_planner/plan_manage/src/traj_server.cpp
- fast_planner/plan_manage/include/plan_manage/topo_replan_fsm.h
- fast_planner/plan_manage/include/plan_manage/kino_replan_fsm.h
- fast_planner/plan_manage/include/plan_manage/planner_manager.h
- fast_planner/plan_manage/include/plan_manage/plan_container.hpp
- fast_planner/traj_utils/include/traj_utils/planning_visualization.h
- fast_planner/traj_utils/src/planning_visualization.cpp
- fast_planner/bspline_opt/include/bspline_opt/bspline_optimizer.h
- fast_planner/bspline_opt/src/bspline_optimizer.cpp
- fast_planner/bspline_opt/CMakeLists.txt
- fast_planner/poly_traj/src/traj_generator.cpp
- fast_planner/poly_traj/package.xml
- fast_planner/plan_env/package.xml
- fast_planner/plan_manage/package.xml
- fast_planner/path_searching/package.xml
- fast_planner/traj_utils/package.xml

Additional ROS1-using components in the workspace (simulator, utils, plugins):
- uav_simulator/local_sensing/src/euroc.cpp
- uav_simulator/local_sensing/src/pcl_render_node.cpp
- uav_simulator/local_sensing/src/pointcloud_render_node.cpp
- uav_simulator/so3_control/src/SO3Control.cpp
- uav_simulator/so3_disturbance_generator/src/so3_disturbance_generator.cpp
- uav_simulator/so3_control/src/so3_control_nodelet.cpp
- uav_simulator/Utils/waypoint_generator/src/waypoint_generator.cpp
- uav_simulator/Utils/waypoint_generator/src/sample_waypoints.h
- uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp
- uav_simulator/so3_quadrotor_simulator/src/dynamics/Quadrotor.cpp
- uav_simulator/Utils/rviz_plugins/src/* (multiple rviz plugin files reference ROS)
- uav_simulator/map_generator/src/random_forest_sensing.cpp
- uav_simulator/Utils/multi_map_server/include/multi_map_server/Map2D.h
- uav_simulator/Utils/multi_map_server/include/multi_map_server/Map3D.h
- uav_simulator/Utils/uav_utils/include/uav_utils/utils.h
- uav_simulator/Utils/multi_map_server/src/multi_map_visualization.cc

Notes:
- Some files above are generated build artifacts or message-generation boilerplate that reference ROS1 internals; those will often need more careful handling (message definitions may be reusable but build/tooling changes are necessary).
- Use this list as the canonical set of files to inspect and port. I'll help with targeted ports.
