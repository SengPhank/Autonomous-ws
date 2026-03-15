cmake_minimum_required(VERSION 3.10)
project(rover_messages)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Message files
set(msg_files
  "msg/DriveMoveCmd.msg"
  "msg/DriveOpCmd.msg"
  "msg/DriveState.msg"
  "msg/AutonDrive.msg"
  "msg/ManagerCmd.msg"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# Install the Enums.hpp header
install(
  DIRECTORY include/
  DESTINATION include
)

# Export include directories
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)

ament_package()