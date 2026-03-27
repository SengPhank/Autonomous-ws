cmake_minimum_required(VERSION 3.8)
project(autonomous_waypoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rover_messages REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)

include_directories(
    ${REPO_ROOT_DIR}
)

# ============== Add Goal driver executable ============== 
add_executable(waypoint_node src/waypoint_node.cpp)
add_executable(queue_waypoint src/queue_waypoint.cpp)
# Manually add rover_messages include directory
target_include_directories(waypoint_node PUBLIC
  ${rover_messages_INCLUDE_DIRS}
)

target_include_directories(queue_waypoint PUBLIC
  ${rover_messages_INCLUDE_DIRS}
)
ament_target_dependencies(waypoint_node
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  rover_messages
  rtabmap_msgs
)

ament_target_dependencies(queue_waypoint
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  rover_messages
  rtabmap_msgs
)
# ============== Install both executables ============== 
install(TARGETS waypoint_node queue_waypoint
  DESTINATION lib/${PROJECT_NAME}
)
# Install
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()