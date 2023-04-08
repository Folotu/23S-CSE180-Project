cmake_minimum_required(VERSION 3.5)
project(finalproject)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
# Add other dependencies here, e.g. <your_dependency>

# Create the gpt executable
add_executable(gpt src/gpt.cpp)
ament_target_dependencies(gpt
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  laser_geometry
  # Add other dependencies here, e.g. <your_dependency>
)

install(TARGETS
  gpt
  DESTINATION lib/${PROJECT_NAME})

ament_package()
