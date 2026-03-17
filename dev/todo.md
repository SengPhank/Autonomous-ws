## Camera status
If the status ever becamse "bad", immediately stop the rover
/zed/zed_node/pose/status

# Version
ZED SDK version 5.0
CUDA version V12.6.8

# Other
Yaml config changes for ros2 wrapper is inside /dev

# How waypoint drive works
Mode oscillation fix — hysteresis + state machine: Instead of one turn_threshold, there are now two: turn_enter_threshold (0.35 rad, ~20°) to trigger a re-turn, and turn_exit_threshold (0.15 rad, ~8.5°) to exit turning. The state machine enforces that TURNING must complete before DRIVING starts — no more bouncing between modes mid-maneuver.

Turn-then-drive behavior: Every new goal immediately enters TURNING state and stays there until the rover is well-aligned. Only then does it enter DRIVING. If drift during driving exceeds turn_enter_threshold, it stops, re-enters TURNING, corrects, then resumes DRIVING.

Odometry quality gate: Subscribes to /zed/zed_node/pose/status which publishes ZED's tracking confidence (0-100). If confidence drops below min_tracking_confidence (default 50), it enters RECOVERING state, holds position, and waits. When tracking recovers it re-enters TURNING to re-evaluate from the current position. If it doesn't recover within recovery_timeout_sec, it aborts the goal entirely.

One thing to check — you'll need to add zed_msgs as a dependency in your package.xml and CMakeLists.txt for the pose status subscription. Also verify the exact message type with ros2 topic type /zed/zed_node/pose/status since it may differ slightly between ZED wrapper versions.