import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_bcr = get_package_share_directory('bcr_bot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='true')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_bcr, 'config', 'factory_map.yaml'),
            'params_file': os.path.join(pkg_bcr, 'config', 'nav2_params2.yaml'),
        }.items()
    )
    
    # Usando el archivo local de bcr_bot
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d',
            os.path.join(pkg_bcr, 'rviz', 'nav2_default_view.rviz')
        ]
    )
    
    # rviz_launch_cmd = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     arguments=[
    #         '-d',
    #         os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'),
    #         '--ros-args', '-r', '/tf:=/bcr_bot/tf', '-r', '/tf_static:=/bcr_bot/tf_static'
    #     ]
    # )

    # IMPORTANTE: El bridge de Gazebo publica el TF de odometría (bcr_bot/odom -> bcr_bot/base_footprint)
    # en el topic /bcr_bot/tf, pero Nav2 escucha TF en el topic global /tf.
    # Este relay copia los mensajes de /bcr_bot/tf a /tf para que Nav2 pueda ver la odometría.
    tf_relay = Node(
        package='topic_tools',
        executable='relay',
        name='bcr_bot_tf_relay',
        arguments=['/bcr_bot/tf', '/tf'],
        output='screen'
    )

    # Nodo para publicar la pose inicial automáticamente
    initial_pose_node = Node(
        package='bcr_bot',
        executable='set_initial_pose.py',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'x': 15.0,
            'y': -24.0,
            'z': 0.0,
            'yaw': -1.57,
            'delay': 5.0,
            'robot_namespace': '',
        }]
    )

    ld = LaunchDescription()

    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(tf_relay)
    ld.add_action(initial_pose_node)

    return ld

