cmake_minimum_required(VERSION 3.5)
project(bspline_opt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
# Depend on plan_env for headers and types
find_package(plan_env REQUIRED)

add_library(bspline_opt
  src/bspline_optimizer.cpp
)

# Public includes for downstream packages
target_include_directories(bspline_opt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bspline_opt rclcpp plan_env)
if(TARGET Eigen3::Eigen)
  target_link_libraries(bspline_opt Eigen3::Eigen)
endif()

install(TARGETS bspline_opt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
