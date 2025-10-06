cmake_minimum_required(VERSION 3.5)
project(plan_manage)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(poly_traj REQUIRED)
# Optional: quadrotor_msgs if available
find_package(quadrotor_msgs QUIET)

# Messages (ROS 2)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

include_directories(
  include
)

add_executable(fast_planner_node
  src/fast_planner_node.cpp
  src/kino_replan_fsm.cpp
  src/topo_replan_fsm.cpp
  src/planner_manager.cpp
)

target_include_directories(fast_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Include generated message headers (rosidl) so same-package targets can
# include <plan_manage/msg/XXX.hpp> while building in the workspace.
target_include_directories(fast_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# If the bspline and bspline_opt packages are present in the workspace source
# tree, add their include directories to the build interface so headers like
# "bspline/non_uniform_bspline.h" and "bspline_opt/bspline_optimizer.h"
# can be found while building in the overlay workspace.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../bspline/include")
  target_include_directories(fast_planner_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../bspline/include>
  )
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/include")
  target_include_directories(fast_planner_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/include>
  )
endif()
## Ensure poly_traj headers are visible when building in the workspace
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../poly_traj/include")
  target_include_directories(fast_planner_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../poly_traj/include>
  )
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/include")
  target_include_directories(fast_planner_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/include>
  )
endif()

ament_target_dependencies(fast_planner_node
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs
  plan_env path_searching bspline bspline_opt traj_utils poly_traj
)
## Ensure linking against message typesupport within the same package
rosidl_target_interfaces(fast_planner_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# traj_server node (may require quadrotor_msgs)
add_executable(traj_server src/traj_server.cpp)

target_include_directories(traj_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(traj_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../bspline/include")
  target_include_directories(traj_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../bspline/include>
  )
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/include")
  target_include_directories(traj_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/include>
  )
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../poly_traj/include")
  target_include_directories(traj_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../poly_traj/include>
  )
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/include")
  target_include_directories(traj_server PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/include>
  )
endif()

if(quadrotor_msgs_FOUND)
  ament_target_dependencies(traj_server rclcpp nav_msgs visualization_msgs quadrotor_msgs bspline bspline_opt)
else()
  ament_target_dependencies(traj_server rclcpp nav_msgs visualization_msgs bspline bspline_opt)
endif()
## Ensure linking against message typesupport within the same package
rosidl_target_interfaces(traj_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

## traj_server also uses bspline headers; link to the bspline libraries if
## they are available in the workspace to expose include directories.
if(TARGET bspline)
  target_link_libraries(traj_server bspline)
endif()
if(TARGET bspline_opt)
  target_link_libraries(traj_server bspline_opt)
endif()
if(TARGET poly_traj)
  target_link_libraries(traj_server poly_traj)
endif()

# Link Eigen
if(TARGET Eigen3::Eigen)
  target_link_libraries(traj_server Eigen3::Eigen)
endif()

# Export message runtime
ament_export_dependencies(rosidl_default_runtime)

install(TARGETS fast_planner_node traj_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()


