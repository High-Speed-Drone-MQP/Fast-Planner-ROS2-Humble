cmake_minimum_required(VERSION 3.5)
project(plan_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)

add_library(plan_env
  src/sdf_map.cpp
  src/obj_predictor.cpp
  src/edt_environment.cpp
  src/raycast.cpp
)

# Include directories
target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(_plan_env_deps
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  pcl_conversions
)

ament_target_dependencies(plan_env
  ${_plan_env_deps}
)

# Link system libs
if(TARGET Eigen3::Eigen)
  target_link_libraries(plan_env Eigen3::Eigen)
endif()

# PCL and OpenCV
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
list(REMOVE_DUPLICATES PCL_LIBRARIES)
target_link_libraries(plan_env ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Export headers and library target for downstream packages
install(TARGETS plan_env
  EXPORT export_plan_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${_plan_env_deps})

# Build obj_generator node
add_executable(obj_generator src/obj_generator.cpp)
target_include_directories(obj_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(obj_generator
  rclcpp
  geometry_msgs
  visualization_msgs
)
if(TARGET Eigen3::Eigen)
  target_link_libraries(obj_generator Eigen3::Eigen)
endif()

# Install executable
install(TARGETS obj_generator
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
