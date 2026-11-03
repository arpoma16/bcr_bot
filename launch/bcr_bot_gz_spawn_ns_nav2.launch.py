#!/usr/bin/python3
"""Compatibility wrapper: bcr_bot_gz_spawn.launch.py with Nav2 and stereo resize.

Kept so existing launch files (indra) keep working; it only forwards the arguments.
"""

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

# Same names and defaults this file always had.
DEFAULTS = {
    "camera_enabled": "True",
    "stereo_camera_enabled": "True",
    "two_d_lidar_enabled": "True",
    "position_x": "0.0",
    "position_y": "0.0",
    "orientation_yaw": "0.0",
    "odometry_source": "world",
    "robot_namespace": "bcr_bot",
}


def generate_launch_description():
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory("bcr_bot"), "launch", "bcr_bot_gz_spawn.launch.py")
        ),
        launch_arguments={
            **{key: LaunchConfiguration(key) for key in DEFAULTS},
            "launch_nav2": "True",
            "stereo_resize": "True",
        }.items(),
    )
    return LaunchDescription(
        [DeclareLaunchArgument(key, default_value=value) for key, value in DEFAULTS.items()]
        # Accepted for compatibility; it was never used by the launch.
        + [DeclareLaunchArgument("launch_map_server", default_value="True"), spawn]
    )
