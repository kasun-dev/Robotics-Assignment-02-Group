#!/usr/bin/env python3
"""
Launch slam_toolbox (online async) for mapping. Use this while teleoperating the robot.
"""
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    set_tb3 = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    slam_node = Node(
        package='slam_toolbox',
        executable='online_async_launch.py',
        name='slam_toolbox',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_tb3,
        slam_node
    ])
