cmake_minimum_required(VERSION 3.8)
project(simulation_launch)

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
find_package(nav_msgs REQUIRED) # This is found correctly

# ==============  Add sim_bridge executable ============== 
add_executable(sim_bridge src/sim_bridge.cpp)

ament_target_dependencies(sim_bridge
  rclcpp
  geometry_msgs
  rover_messages
)

# ============== Install both executables ============== 

# Install directories
install(DIRECTORY gazebo launch urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install C++ scripts
install(TARGETS
  sim_bridge
  DESTINATION lib/${PROJECT_NAME}
)
# Install Python scripts
# Note: Ensure the directory structure matches (PROJECT_NAME subfolder vs src)
install(PROGRAMS
  ${PROJECT_NAME}/zed_mimic_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()