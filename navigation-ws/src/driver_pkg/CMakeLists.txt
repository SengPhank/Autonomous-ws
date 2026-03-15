cmake_minimum_required(VERSION 3.8)
project(driver_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rover_messages REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ============== Add Goal driver executable ============== 
add_executable(goal_driver_node src/goal_driver_node.cpp)
# Manually add rover_messages include directory
target_include_directories(goal_driver_node PUBLIC
  ${rover_messages_INCLUDE_DIRS}
)
ament_target_dependencies(goal_driver_node
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  rover_messages
)

# ============== Install both executables ============== 
install(TARGETS goal_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
# Install
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()