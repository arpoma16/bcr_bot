import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_bcr = get_package_share_directory('bcr_bot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # slam_toolbox en modo mapping, cargando el mapa existente para continuar
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_bcr, 'config', 'mapper_params_online_async.yaml'),
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d', os.path.join(pkg_bcr, 'rviz', 'map.rviz')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    delayed_slam = TimerAction(
        period=3.0,
        actions=[slam_toolbox_launch_cmd]
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_launch_cmd]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(delayed_slam)
    ld.add_action(delayed_rviz)

    return ld
