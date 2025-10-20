#!/usr/bin/env python3
"""
Launch Gazebo with the maze.world and spawn TurtleBot3 Burger.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    world_file = os.path.join(pkg_share, 'worlds', 'maze.world')

    # ensure model env for turtlebot3
    set_tb3 = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    gzserver = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # spawn robot using spawn_entity from gazebo_ros package
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'turtlebot3', '-x', '0.0', '-y', '0.0', '-z', '0.01'],
        output='screen'
    )

    # We'll also start robot_state_publisher so robot_description topic is available
    urdf_path = os.path.join(os.getenv('HOME'), 'turtlebot3_ws', 'src', 'turtlebot3', 'turtlebot3_description', 'urdf', 'turtlebot3_burger.urdf')
    # If user has turtlebot3_description installed to ROS2, robot_description can be published from parameter.
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}] if os.path.exists(urdf_path) else []
    )

    return LaunchDescription([
        set_tb3,
        gzserver,
        robot_state_pub,
        spawn_robot,
    ])
