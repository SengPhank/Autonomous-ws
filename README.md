# Requirements
- Ubuntu 22.04 (Jammy)
- Ros2 Humble
- C++17

# Preperation
Source ros
```bash
source /opt/ros/humble/setup.bash
```
Clone and enter ws
```bash
git clone https://github.com/SengPhank/Autonomous-ws.git
cd Autonomous-ws
```

# Building

Build messages
```bash
cd interfaces-ws
colcon build
source install/setup.bash
cd ..
```

Build zed2i wrapper
```bash
cd AutoNav_modules
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
cd ..
```
Build autonomous and simulation features
```bash
# Autonomous features
cd navigation-ws
colcon build --symlink
cd ..
cd simulation-ws
colcon build --symlink
cd ..
```

Source all builds
```bash
# Source features
source AutoNav_modules/install/setup.bash
source navigation-ws/install/setup.bash
source simulation-ws/install/setup.bash
```
# Launching nodes
You must run each node in their own independent terminal (after sourcing)

Launch zed2i cameras
```bash
cd AutoNav_modules
so
ros2 launch mapping_module new_rtab.launch.py camera_model:=zed2i
```
Launch rviz2 for visualisation
```bash
rviz2
```

Launch costmap
```bash
ros2 launch autonomous_costmap costmap.launch.py
```

