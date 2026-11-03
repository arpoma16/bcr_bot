#!/usr/bin/python3
"""Nav2 for the bcr_bot already spawned by indra (run.yml): factory map, spawn pose and RViz.

Thin wrapper over bcr_bot_nav2.launch.py. Nav2 lives in /<robot_namespace>
(e.g. /bcr_bot/navigate_to_pose, /bcr_bot/amcl_pose); the TF relays come from the spawn.
"""

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

# Must match the spawn pose in indra.launch.py.
DEFAULTS = {
    "robot_namespace": "bcr_bot",
    "position_x": "15.0",
    "position_y": "-24.0",
    "orientation_yaw": "-1.57",
    "use_sim_time": "True",
    "autostart": "True",
    "rviz": "True",
}


def generate_launch_description():
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory("bcr_bot"), "launch", "bcr_bot_nav2.launch.py")
        ),
        launch_arguments={key: LaunchConfiguration(key) for key in DEFAULTS}.items(),
    )
    return LaunchDescription(
        [DeclareLaunchArgument(key, default_value=value) for key, value in DEFAULTS.items()] + [nav2]
    )
