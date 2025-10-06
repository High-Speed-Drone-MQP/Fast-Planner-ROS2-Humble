cmake_minimum_required(VERSION 3.5)
project(path_searching)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plan_env REQUIRED)

add_library(path_searching
  src/kinodynamic_astar.cpp
  src/astar.cpp
  src/topo_prm.cpp
)

# Public includes for downstream packages
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_searching
  rclcpp
  plan_env
)

# Eigen header-only, link target to export include dirs if available
if(TARGET Eigen3::Eigen)
  target_link_libraries(path_searching Eigen3::Eigen)
endif()

install(TARGETS path_searching
  EXPORT export_path_searching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp plan_env)

ament_package()
