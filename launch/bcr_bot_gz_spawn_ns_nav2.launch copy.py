#!/usr/bin/python3

import os
import tempfile
from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    bcr_bot_path = get_package_share_directory("bcr_bot")

    position_x = LaunchConfiguration("position_x").perform(context)
    position_y = LaunchConfiguration("position_y").perform(context)
    orientation_yaw = LaunchConfiguration("orientation_yaw").perform(context)
    camera_enabled = LaunchConfiguration("camera_enabled").perform(context)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled").perform(context)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled").perform(context)
    odometry_source = LaunchConfiguration("odometry_source").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    launch_map_server = LaunchConfiguration("launch_map_server").perform(context)

    urdf = join(bcr_bot_path, "urdf", "bcr_bot_ns.xacro")

    robot_description_config = Command(
        ["xacro ", urdf,
         " camera_enabled:=", camera_enabled,
         " stereo_camera_enabled:=", stereo_camera_enabled,
         " two_d_lidar_enabled:=", two_d_lidar_enabled,
         " odometry_source:=", odometry_source,
         " sim_gz:=true",
         " robot_namespace:=", robot_namespace,
         ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_namespace,
        parameters=[
            {"robot_description": robot_description_config},
            {"use_sim_time": True},
        ],
        remappings=[
            ("/tf",        f"/{robot_namespace}/tf"),
            ("/tf_static", f"/{robot_namespace}/tf_static"),
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", f"/{robot_namespace}/robot_description",
            "-name", robot_namespace,
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw,
        ],
        parameters=[{"use_sim_time": True}],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=robot_namespace,
        name="gz_bridge",
        arguments=[
            f"/{robot_namespace}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            f"/{robot_namespace}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/{robot_namespace}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"/{robot_namespace}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"/{robot_namespace}/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{robot_namespace}/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{robot_namespace}/stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            f"/{robot_namespace}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"/{robot_namespace}/kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"/{robot_namespace}/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"/{robot_namespace}/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"/{robot_namespace}/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            f"/{robot_namespace}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            f"/world/default/model/{robot_namespace}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        parameters=[{"use_sim_time": True}],
        remappings=[
            (f"/world/default/model/{robot_namespace}/joint_state", f"/{robot_namespace}/joint_states"),
            (f"/{robot_namespace}/odometry", f"/{robot_namespace}/odom"),
            (f"/{robot_namespace}/camera_info", f"/{robot_namespace}/kinect_camera/camera_info"),
        ],
    )

    stereo_resize_container = ComposableNodeContainer(
        name="stereo_resize_container",
        namespace=robot_namespace,
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name="stereo_left_resize",
                namespace=robot_namespace,
                remappings=[
                    ("image/image_raw", "stereo_camera/left/image_raw"),
                    ("image/camera_info", "stereo_camera/left/camera_info"),
                    ("resize/image_raw", "stereo_camera/left/image_resized"),
                    ("resize/camera_info", "stereo_camera/left/image_resized/camera_info"),
                ],
                parameters=[{
                    "use_sim_time": True,
                    "use_scale": False,
                    "width": 640,
                    "height": 640,
                }],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::ResizeNode",
                name="stereo_right_resize",
                namespace=robot_namespace,
                remappings=[
                    ("image/image_raw", "stereo_camera/right/image_raw"),
                    ("image/camera_info", "stereo_camera/right/camera_info"),
                    ("resize/image_raw", "stereo_camera/right/image_resized"),
                    ("resize/camera_info", "stereo_camera/right/image_resized/camera_info"),
                ],
                parameters=[{
                    "use_sim_time": True,
                    "use_scale": False,
                    "width": 640,
                    "height": 640,
                }],
            ),
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    stereo_left_compressed = Node(
        package="image_transport",
        executable="republish",
        name="stereo_left_compressed_republisher",
        namespace=robot_namespace,
        arguments=["raw", "compressed"],
        remappings=[
            ("in", f"/{robot_namespace}/stereo_camera/left/image_resized"),
            ("out/compressed", f"/{robot_namespace}/stereo_camera/left/image_raw/compressed"),
        ],
        parameters=[{"use_sim_time": True}],
    )

    stereo_right_compressed = Node(
        package="image_transport",
        executable="republish",
        name="stereo_right_compressed_republisher",
        namespace=robot_namespace,
        arguments=["raw", "compressed"],
        remappings=[
            ("in", f"/{robot_namespace}/stereo_camera/right/image_resized"),
            ("out/compressed", f"/{robot_namespace}/stereo_camera/right/image_raw/compressed"),
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Relay the robot's namespaced TF to the global /tf so Nav2 can see odometry transforms
    tf_relay = Node(
        package="topic_tools",
        executable="relay",
        name=f"{robot_namespace}_tf_relay",
        arguments=[f"/{robot_namespace}/tf", "/tf"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Relay tf_static so fixed joints (base_footprint→base_link, etc.) reach Nav2
    tf_static_relay = Node(
        package="topic_tools",
        executable="relay",
        name=f"{robot_namespace}_tf_static_relay",
        arguments=[f"/{robot_namespace}/tf_static", "/tf_static"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # AMCL runs under the robot namespace so it subscribes to /<ns>/map.
    # The shared map_server publishes on /map (global). Bridge the two.
    map_relay = Node(
        package="topic_tools",
        executable="relay",
        name=f"{robot_namespace}_map_relay",
        arguments=["/map", f"/{robot_namespace}/map"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    nav2_params_file = os.path.join(bcr_bot_path, "config", "nav2_params_per_robot.yaml")
    with open(nav2_params_file, "r") as f:
        params_content = f.read()
    params_content = params_content.replace("ROBOT_NS", robot_namespace)
    tmp_params = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    tmp_params.write(params_content)
    tmp_params.close()
    configured_params = tmp_params.name

    # Launch the shared map_server only when launch_map_server:=True (first robot).
    bcr_bot_path_str = bcr_bot_path  # already a plain string in this scope
    nav2_shared = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bcr_bot_path_str, "launch", "nav2_shared.launch.py")
        ),
        launch_arguments={"use_sim_time": "True"}.items(),
        condition=IfCondition(PythonExpression([f"'{launch_map_server}'.lower() == 'true'"])),
    )

    # nav2_bringup's navigation_launch.py and localization_launch.py do NOT set
    # a ROS namespace on their nodes (no PushRosNamespace / namespace= kwarg).
    # With multiple robots every node ends up at /bt_navigator, /amcl, etc. and
    # the second robot silently clobbers the first.  We therefore launch each
    # node directly with namespace=robot_namespace so they become
    # /bcr_bot_1/bt_navigator, /bcr_bot_1/amcl, etc.
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings + [
            ("cmd_vel", "cmd_vel_nav"),
            ("cmd_vel_smoothed", "cmd_vel"),
        ],
    )

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        namespace=robot_namespace,
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        namespace=robot_namespace,
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["amcl"],
        }],
    )

    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=robot_namespace,
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": [
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
                "collision_monitor",
            ],
        }],
    )

    initial_pose_node = Node(
        package="bcr_bot",
        executable="set_initial_pose.py",
        name=f"{robot_namespace}_initial_pose_publisher",
        output="screen",
        parameters=[{
            "x": float(position_x),
            "y": float(position_y),
            "z": 0.0,
            "yaw": float(orientation_yaw),
            "delay": 10.0,
            "robot_namespace": robot_namespace,
            "use_sim_time": True,
        }],
    )

    return [
        robot_state_publisher,
        gz_spawn_entity,
        gz_ros2_bridge,
        stereo_resize_container,
        stereo_left_compressed,
        stereo_right_compressed,
        tf_relay,
        tf_static_relay,
        map_relay,
        nav2_shared,
        amcl,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        collision_monitor,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
        initial_pose_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_enabled", default_value="True"),
        DeclareLaunchArgument("stereo_camera_enabled", default_value="True"),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value="True"),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        DeclareLaunchArgument("robot_namespace", default_value="bcr_bot"),
        DeclareLaunchArgument("launch_map_server", default_value="True"),
        OpaqueFunction(function=launch_setup),
    ])
