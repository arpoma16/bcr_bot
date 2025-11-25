#!/usr/bin/env python3
import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition

def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    camera_enabled = LaunchConfiguration('camera_enabled')
    stereo_camera_enabled = LaunchConfiguration('stereo_camera_enabled')
    two_d_lidar_enabled = LaunchConfiguration('two_d_lidar_enabled')
    isaac_sim = LaunchConfiguration('isaac_sim')

    bcr_bot_path = get_package_share_directory('bcr_bot')

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            {'robot_description': Command([
                'xacro ', join(bcr_bot_path, 'urdf/bcr_bot_ns.xacro'),
                ' camera_enabled:=', camera_enabled,
                ' stereo_camera_enabled:=', stereo_camera_enabled,
                ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                ' odometry_source:=world',
                ' sim_gz:=false',
                ' robot_namespace:=', robot_namespace,
            ])},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        condition=UnlessCondition(isaac_sim)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', join(bcr_bot_path, 'rviz', 'entire_setup_ns.rviz'),
            '--ros-args', '-r', '/tf:=/bcr_bot/tf', '-r', '/tf_static:=/bcr_bot/tf_static'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='bcr_bot',
            description='Namespace for the robot'
        ),
        DeclareLaunchArgument(
            'camera_enabled',
            default_value='true',
            description='Enable Kinect camera'
        ),
        DeclareLaunchArgument(
            'stereo_camera_enabled',
            default_value='false',
            description='Enable stereo camera'
        ),
        DeclareLaunchArgument(
            'two_d_lidar_enabled',
            default_value='true',
            description='Enable 2D LIDAR'
        ),
        DeclareLaunchArgument(
            'isaac_sim',
            default_value='false',
            description='Set to true when using Isaac Sim'
        ),
        # Nodes
        robot_state_publisher,
        rviz
    ])