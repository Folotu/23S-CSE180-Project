cmake_minimum_required(VERSION 3.5)
project(finalproject)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Create an executable for the map subscriber
add_executable(map_subscriber src/map_subscriber.cpp)
ament_target_dependencies(map_subscriber rclcpp nav_msgs)

# Create an executable for the scan subscriber
add_executable(scan_subscriber src/scan_subscriber.cpp)
ament_target_dependencies(scan_subscriber rclcpp sensor_msgs)

# Create an executable for the pose subscriber
add_executable(pose_subscriber src/pose_subscriber.cpp)
ament_target_dependencies(pose_subscriber rclcpp geometry_msgs)

add_executable(tfstatic_subscriber src/tfstatic_subscriber.cpp)
ament_target_dependencies(tfstatic_subscriber rclcpp tf2_msgs)

add_executable(findExtraPost src/findExtraPost.cpp)
ament_target_dependencies(findExtraPost rclcpp nav_msgs geometry_msgs)

# Install the executables
install(TARGETS
  map_subscriber
  scan_subscriber
  pose_subscriber
  tfstatic_subscriber
  findExtraPost
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
