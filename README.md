# Requirements
- Ubuntu 22.04 (Jammy)
- Ros2 Humble
- C++17

# Installing
## Preperation
```bash
sudo apt update 
source /opt/ros/humble/setup.bash
```
Clone and enter ws
```bash
git clone https://github.com/SengPhank/Autonomous-ws.git --recursive
cd Autonomous-ws
```

## Building
Build messages FIRST!
```bash
cd interfaces-ws
colcon build
source install/setup.bash
cd ..
```

Build and source zed2i wrapper 
```bash
cd AutoNav_modules
sudo apt install ros-humble-rtabmap-ros # Install rtabmap_ros binaries
rosdep update 
rosdep install --from-paths src --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source install/setup.bash
cd ..
```
Build and source autonomous and simulation features
```bash
# Autonomous features
cd navigation-ws
colcon build --symlink
source install/setup.bash
cd ..
# Sim features
cd simulation-ws
colcon build --symlink
source install/setup.bash
cd ..
```
## Launching nodes
You must run each node in their own independent terminal (after sourcing)

Launch zed2i cameras
```bash
ros2 launch autonomous_camera camera.launch.py camera_model:=zed2i
```

Launch features
```bash
cd navigation-ws
ros2 launch autonomous_costmap costmap.launch.py # Costmap
ros2 launch autonomous_waypoint waypoint.launch.py # autonDriver
```

Launch rviz2 for visualisation (OPTIONAL)
```bash
rviz2   
```

## Launching a simulation
TBA