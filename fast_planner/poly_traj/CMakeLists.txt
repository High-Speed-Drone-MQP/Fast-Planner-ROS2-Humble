cmake_minimum_required(VERSION 3.5)
project(poly_traj)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# Optional
find_package(swarmtal_msgs QUIET)

# Library
add_library(poly_traj
  src/polynomial_traj.cpp
)

target_include_directories(poly_traj PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(TARGET Eigen3::Eigen)
  target_link_libraries(poly_traj Eigen3::Eigen)
endif()

# Executable
add_executable(traj_generator src/traj_generator.cpp)

target_include_directories(traj_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(traj_generator rclcpp nav_msgs visualization_msgs)
if(TARGET Eigen3::Eigen)
  target_link_libraries(traj_generator Eigen3::Eigen)
endif()

if(swarmtal_msgs_FOUND)
  ament_target_dependencies(traj_generator swarmtal_msgs)
  target_compile_definitions(traj_generator PRIVATE HAVE_SWARMTAL_MSGS)
endif()

# Link the executable with the library implementing minSnapTraj
target_link_libraries(traj_generator poly_traj)

install(TARGETS poly_traj
  EXPORT export_poly_traj
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS traj_generator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_poly_traj HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 eigen3_cmake_module)

ament_package()
