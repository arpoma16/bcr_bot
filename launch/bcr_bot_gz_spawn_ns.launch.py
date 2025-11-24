#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression
from launch_ros.actions import PushRosNamespace


from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    bcr_bot_path = get_package_share_directory("bcr_bot")
    
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")
    robot_namespace = LaunchConfiguration("robot_namespace", default='bcr_bot')


    # robot_description_content = get_xacro_to_doc(
    #     join(bcr_bot_path, "urdf", "bcr_bot.xacro"),
    #     {"sim_gz": "true",
    #      "two_d_lidar_enabled": "true",
    #      "conveyor_enabled": "false",
    #      "camera_enabled": "true"
    #     }
    # ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', join(bcr_bot_path, 'urdf/bcr_bot.xacro'),
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' odometry_source:=', odometry_source,
                    ' sim_gz:=', "true",
                    ' robot_namespace:=', robot_namespace,
                    ])},
                    {'use_sim_time': True}],
        remappings=[
            ('/joint_states', [robot_namespace, '/joint_states']),
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
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            ["/world/default/model/", robot_namespace, "/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"]
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            (["/world/default/model/", robot_namespace, "/joint_state"], [robot_namespace, "/joint_states"]),
            (["/", robot_namespace, "/odom"], [robot_namespace, "/odom"]),
            (["/", robot_namespace, "/scan"], [robot_namespace, "/scan"]),
            (["/", robot_namespace, "/kinect_camera"], [robot_namespace, "/kinect_camera"]),
            (["/", robot_namespace, "/stereo_camera/left/image_raw"], [robot_namespace, "/stereo_camera/left/image_raw"]),
            (["/", robot_namespace, "/stereo_camera/right/image_raw"], [robot_namespace, "/stereo_camera/right/image_raw"]),
            (["/", robot_namespace, "/imu"], [robot_namespace, "/imu"]),
            (["/", robot_namespace, "/cmd_vel"], [robot_namespace, "/cmd_vel"]),
            (["/", robot_namespace, "/kinect_camera/camera_info"], [robot_namespace, "/kinect_camera/camera_info"]),
            (["/", robot_namespace, "/stereo_camera/left/camera_info"], [robot_namespace, "/stereo_camera/left/camera_info"]),
            (["/", robot_namespace, "/stereo_camera/right/camera_info"], [robot_namespace, "/stereo_camera/right/camera_info"]),
            (["/", robot_namespace, "/kinect_camera/points"], [robot_namespace, "/kinect_camera/points"]),
        ]
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "kinect_camera",
                    "--child-frame-id", [robot_namespace, "/base_footprint/kinect_camera"]
        ],
        parameters=[{'use_sim_time': True}]
    )
    transform_publisher_with_ns = GroupAction([
    PushRosNamespace(robot_namespace),
    transform_publisher
    ])

    return LaunchDescription([
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        robot_state_publisher,
        gz_spawn_entity, transform_publisher_with_ns,gz_ros2_bridge
    ])