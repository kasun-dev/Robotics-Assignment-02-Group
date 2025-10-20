#!/usr/bin/env python3
"""
Launch Nav2 bringup with a map (map parameter).
Usage: ros2 launch turtlebot3_maze_nav nav2.launch.py map:=/path/to/maze.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    set_tb3 = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.getcwd(), 'maps', 'maze.yaml'),
        description='Full path to map yaml file'
    )

    nav2_params = os.path.join(
        os.path.dirname(__file__),
        '..', 'params', 'nav2_params.yaml'
    )

    nav2_bringup = Node(
        package='nav2_bringup',
        executable='nav2_bringup_launch.py',
        output='screen',
        arguments=['--ros-args', '--params-file', nav2_params]
    )

    return LaunchDescription([
        set_tb3,
        map_arg,
        nav2_bringup
    ])
