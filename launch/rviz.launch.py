#!/usr/bin/env python3
"""RViz for bcr_bot.

* robot_namespace empty (default, upstream behaviour): robot_state_publisher from the xacro
  plus RViz with rviz/entire_setup.rviz, all global.
* robot_namespace:=<ns>: RViz inside /<ns> with rviz/nav2_namespaced_view.rviz
  (<robot_namespace> replaced by /<ns>) for a robot that is already running (spawn + Nav2),
  so no extra robot_state_publisher. TF is read from /<ns>/tf, like the namespaced Nav2.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc


def _is_true(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def launch_setup(context, *args, **kwargs):
    bcr_bot_path = get_package_share_directory('bcr_bot')
    ns = LaunchConfiguration('robot_namespace').perform(context).strip('/')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if ns:
        # Namespaced robot = simulation: default to sim time unless given explicitly.
        sim_time = _is_true(use_sim_time) if use_sim_time else True
        rviz_config = ReplaceString(
            source_file=os.path.join(bcr_bot_path, 'rviz', 'nav2_namespaced_view.rviz'),
            replacements={'<robot_namespace>': f'/{ns}'},
        )
        return [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            namespace=ns,
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': sim_time}],
            remappings=[
                ('/map', 'map'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/goal_pose', 'goal_pose'),
                ('/clicked_point', 'clicked_point'),
                ('/initialpose', 'initialpose'),
            ],
        )]

    # Without namespace: same as before (own robot_state_publisher + entire_setup.rviz).
    sim_time = _is_true(use_sim_time) if use_sim_time else False
    actions = []
    if not _is_true(LaunchConfiguration('isaac_sim').perform(context)):
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': sim_time},
                        {'robot_description': LaunchConfiguration('robot_description').perform(context)}],
        ))
    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bcr_bot_path, 'rviz', 'entire_setup.rviz')],
    ))
    return actions


def generate_launch_description():
    xacro_path = os.path.join(get_package_share_directory('bcr_bot'), 'urdf', 'bcr_bot.xacro')
    doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom"})

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Empty: upstream RViz with its own robot_state_publisher. '
                        '<ns>: namespaced Nav2 view for a robot already running in /<ns>.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='',
            description='Use simulation time. Empty: false without namespace, true with namespace.'
        ),
        DeclareLaunchArgument(
            'isaac_sim',
            default_value='false',
            description='Set to true when using Isaac Sim (no robot_state_publisher; only without namespace)'
        ),
        DeclareLaunchArgument(
            'robot_description',
            default_value=doc.toxml(),
            description='Robot description in URDF/XACRO format (only without namespace)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
