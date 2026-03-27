import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('autonomous_waypoint'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='autonomous_waypoint',
            executable='queue_waypoint', # 'waypoint_node'
            name='waypoint_driver',
            parameters=[config],
            output='screen'
        )
    ])