## Camera status
If the status ever becamse "bad", immediately stop the rover
/zed/zed_node/pose/status

# Version
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2025 NVIDIA Corporation
Built on Wed_Jan_15_19:20:09_PST_2025
Cuda compilation tools, release 12.8, V12.8.61
Build cuda_12.8.r12.8/compiler.35404655_0

ZED SDK version 5.1.2
CUDA version V12.8.61

# Other
Yaml config changes for ros2 wrapper is inside /dev

# Run 
Run the command below inside AutoNav_modules to remove the build error
```bash
grep -rl "message_filters/.*\.hpp" src/rtabmap_ros/rtabmap_sync | xargs sed -i 's/\(message_filters\/.*\)\.hpp/\1.h/g'
find src/ -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -exec sed -i 's|#include <message_filters/\(.*\)\.hpp>|#include <message_filters/\1.h|g' {} +
```