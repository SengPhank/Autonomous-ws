#!/usr/bin/env python3
"""
Launch file for global costmap with config file support.
Usage: ros2 launch autonomous_costmap costmap.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package path
    pkg_autonomous_costmap = FindPackageShare('autonomous_costmap')
    
    # Config file path
    config_file = PathJoinSubstitution([
        pkg_autonomous_costmap,
        'config',
        'costmap_params.yaml'
    ])
    
    # Optional: allow overriding config file path
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=str(config_file),
        description='Path to costmap config YAML file'
    )
    
    # Costmap node
    costmap_node = Node(
        package='autonomous_costmap',
        executable='pcd_to_height_costmap',
        name='global_costmap',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    return LaunchDescription([
        declare_config_file,
        costmap_node,
    ])