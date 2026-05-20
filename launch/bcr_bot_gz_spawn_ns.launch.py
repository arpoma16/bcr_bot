#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command


from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
   
    bcr_bot_path = get_package_share_directory("bcr_bot")

    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=True)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")
    robot_namespace = LaunchConfiguration("robot_namespace", default='bcr_bot')
    urdf = join(bcr_bot_path, 'urdf', 'bcr_bot_ns.xacro')

    robot_description_config = Command( \
                    ['xacro ', urdf,
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' odometry_source:=', odometry_source,
                    ' sim_gz:=', "true",
                    ' robot_namespace:=', robot_namespace,
                    ])
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_namespace,
        parameters=[
                    {'robot_description': robot_description_config },
                    # {"frame_prefix": robot_namespace + "/"},
                    {'use_sim_time': True},
                    ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", ["/", robot_namespace, "/robot_description"],
            "-name", robot_namespace,
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ],
        parameters=[{'use_sim_time': True}]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            ["/", robot_namespace, "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
            ["/", robot_namespace, "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            ["/", robot_namespace, "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
            ["/", robot_namespace, "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
            ["/", robot_namespace, "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image"],
            ["/", robot_namespace, "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"],
            ["/", robot_namespace, "/stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"],
            ["/", robot_namespace, "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            ["/", robot_namespace, "/kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            ["/", robot_namespace, "/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            ["/", robot_namespace, "/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            ["/", robot_namespace, "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
            ["/", robot_namespace, "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
            ["/world/default/model/", robot_namespace, "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"]
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            (["/world/default/model/", robot_namespace, "/joint_state"], [robot_namespace, "/joint_states"]),
            ([robot_namespace, "/odometry"], [robot_namespace, "/odom"]),
            ([robot_namespace, "/camera_info"], [robot_namespace, "/kinect_camera/camera_info"]),
        ]
    )
    
    stereo_left_compressed = Node(
        package='image_transport',
        executable='republish',
        name='stereo_left_compressed_republisher',
        namespace=robot_namespace,
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', 'stereo_camera/left/image_raw'),
            ('out/compressed', 'stereo_camera/left/image_raw/compressed'),
        ],
        parameters=[{'use_sim_time': True}]
    )

    stereo_right_compressed = Node(
        package='image_transport',
        executable='republish',
        name='stereo_right_compressed_republisher',
        namespace=robot_namespace,
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', 'stereo_camera/right/image_raw'),
            ('out/compressed', 'stereo_camera/right/image_raw/compressed'),
        ],
        parameters=[{'use_sim_time': True}]
    )


    return LaunchDescription([
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        robot_state_publisher,
        gz_spawn_entity,
        gz_ros2_bridge,
        stereo_left_compressed,
        stereo_right_compressed
    ])