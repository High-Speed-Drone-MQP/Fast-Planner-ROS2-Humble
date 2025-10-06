cmake_minimum_required(VERSION 3.5)
project(traj_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bspline REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)
find_package(poly_traj REQUIRED)
find_package(PCL 1.7 REQUIRED)

add_library(traj_utils
  src/planning_visualization.cpp
)

# Public includes for this target only its own include
# Remove sibling include directories to avoid absolute source paths leaking to dependents
set(TRAJ_UTILS_PUBLIC_INCLUDE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

target_include_directories(traj_utils PUBLIC
  ${TRAJ_UTILS_PUBLIC_INCLUDE}
)

if(TARGET Eigen3::Eigen)
  target_link_libraries(traj_utils Eigen3::Eigen)
endif()

if(TARGET PCL::PCL)
  target_link_libraries(traj_utils PCL::PCL)
elseif(DEFINED PCL_LIBRARIES)
  target_link_libraries(traj_utils ${PCL_LIBRARIES})
endif()

ament_target_dependencies(traj_utils rclcpp visualization_msgs geometry_msgs bspline bspline_opt path_searching poly_traj)

install(TARGETS traj_utils
  EXPORT export_traj_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_traj_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp visualization_msgs geometry_msgs bspline bspline_opt path_searching poly_traj Eigen3)

ament_package()