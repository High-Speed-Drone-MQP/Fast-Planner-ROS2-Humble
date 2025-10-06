cmake_minimum_required(VERSION 3.5)
project(bspline)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

add_library(bspline
  src/non_uniform_bspline.cpp
)

# Public includes for downstream packages
target_include_directories(bspline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link to propagate include dirs for Eigen3 and ensure rclcpp is available
ament_target_dependencies(bspline rclcpp)
# Eigen3 is header-only; linking the imported target helps export includes
if(TARGET Eigen3::Eigen)
  target_link_libraries(bspline Eigen3::Eigen)
endif()

install(TARGETS bspline
  EXPORT export_bspline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_bspline HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 eigen3_cmake_module)

ament_package()
