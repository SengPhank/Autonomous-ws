import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('driver_pkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='driver_pkg',
            executable='goal_driver_node',
            name='goal_driver',
            parameters=[config],
            output='screen'
        )
    ])