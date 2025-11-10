#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
    bcr_bot_path = get_package_share_directory("bcr_bot")
    
    # Namespace único para el robot
    robot_name = LaunchConfiguration("robot_name")
    
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled")
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled")
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled")
    odometry_source = LaunchConfiguration("odometry_source")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[
            {
                'robot_description': Command([
                    'xacro ', join(bcr_bot_path, 'urdf/bcr_bot_ns.xacro'),
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', robot_name,
                    ' sim_gz:=true'
                ])
            },
            {'frame_prefix': [robot_name, '/']}
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", ["/", robot_name, "/robot_description"],
            "-name", robot_name,
            "-allow_renaming", "false",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ],
        output="screen"
    )

    # Bridge para topics de Gazebo CON namespace del robot
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        namespace=robot_name,
        arguments=[
            # Topics de Gazebo que incluyen el nombre del modelo
            ["/model/", robot_name, "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"],
            ["/model/", robot_name, "/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry"],
            ["/model/", robot_name, "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V"],
            ["/model/", robot_name, "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"],
            ["/world/default/model/", robot_name, "/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model"],
            
            # Sensores de cámara
            ["/model/", robot_name, "/kinect_camera@sensor_msgs/msg/Image@gz.msgs.Image"],
            ["/model/", robot_name, "/kinect_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked"],
            ["/model/", robot_name, "/kinect_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
            
            # Stereo camera
            ["/model/", robot_name, "/stereo_camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"],
            ["/model/", robot_name, "/stereo_camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image"],
            ["/model/", robot_name, "/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
            ["/model/", robot_name, "/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"],
            
            # IMU
            ["/model/", robot_name, "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU"],
            
            # Clock (sin namespace, es global)
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            # Remapeo de topics de Gazebo a ROS 2 con namespace
            (["/model/", robot_name, "/cmd_vel"], "cmd_vel"),
            (["/model/", robot_name, "/odometry"], "odom"),
            (["/model/", robot_name, "/tf"], "tf"),
            (["/model/", robot_name, "/scan"], "scan"),
            (["/world/default/model/", robot_name, "/joint_state"], "joint_states"),
            (["/model/", robot_name, "/kinect_camera"], "kinect_camera/image_raw"),
            (["/model/", robot_name, "/kinect_camera/points"], "kinect_camera/points"),
            (["/model/", robot_name, "/kinect_camera/camera_info"], "kinect_camera/camera_info"),
            (["/model/", robot_name, "/stereo_camera/left/image_raw"], "stereo_camera/left/image_raw"),
            (["/model/", robot_name, "/stereo_camera/right/image_raw"], "stereo_camera/right/image_raw"),
            (["/model/", robot_name, "/stereo_camera/left/camera_info"], "stereo_camera/left/camera_info"),
            (["/model/", robot_name, "/stereo_camera/right/camera_info"], "stereo_camera/right/camera_info"),
            (["/model/", robot_name, "/imu"], "imu"),
        ],
        output="screen"
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=robot_name,
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", ["", robot_name, "/kinect_camera"],
            "--child-frame-id", [robot_name, "/base_footprint/kinect_camera"]
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="bcr_bot"),
        DeclareLaunchArgument("camera_enabled", default_value="true"),
        DeclareLaunchArgument("stereo_camera_enabled", default_value="false"),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value="true"),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        
        robot_state_publisher,
        gz_spawn_entity,
        gz_ros2_bridge,
        transform_publisher
    ])