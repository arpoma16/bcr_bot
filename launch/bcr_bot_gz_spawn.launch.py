#!/usr/bin/python3
"""Spawn bcr_bot in a running Gazebo (gz sim) world, optionally with Nav2.

`robot_namespace` selects the naming scheme:

* ""  (default, upstream behaviour): frames without prefix (base_link, odom), global /tf,
  ROS topics under /bcr_bot/* (scan, odom, cmd_vel, ...). Gazebo model name: bcr_bot.
* "<ns>": every link/joint/frame is prefixed "<ns>/", nodes live in /<ns> and the ROS
  topics are /<ns>/*. The robot TF is kept in /<ns>/tf and /<ns>/tf_static (what the
  namespaced Nav2 reads) and relayed to the global /tf and /tf_static (RViz, other tools).
  This is what allows several robots in the same world.

launch_nav2:=true adds bcr_bot_nav2.launch.py (needs a namespace) with the spawn pose as
initial pose; nav2_map / nav2_params_file override its defaults.
"""

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

DEFAULT_NAME = "bcr_bot"


def _is_true(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _bridge_entries(ns, name):
    """Return (bridge arguments, remappings) for ros_gz_bridge.

    Gazebo topics follow the xacro `prefix` ("" or "<ns>/"); ROS topics are always
    under /<name> (name = ns or "bcr_bot"), except /tf which is global without namespace.
    """
    gz = f"/{ns}/" if ns else "/"
    ros = f"/{name}/"

    # (gazebo topic suffix, "ros_type<direction>gz_type", ROS topic override or None)
    entries = [
        ("cmd_vel", "geometry_msgs/msg/Twist@gz.msgs.Twist", None),
        ("lift_cmd", "std_msgs/msg/Float64]gz.msgs.Double", None),
        ("odometry", "nav_msgs/msg/Odometry[gz.msgs.Odometry", ros + "odom"),
        ("tf", "tf2_msgs/msg/TFMessage[gz.msgs.Pose_V", f"/{ns}/tf" if ns else "/tf"),
        ("scan", "sensor_msgs/msg/LaserScan[gz.msgs.LaserScan", None),
        ("kinect_camera", "sensor_msgs/msg/Image[gz.msgs.Image", None),
        # Estereo desactivado (sustituido por front_camera); descomentar junto con stereo_camera_enabled.
        # ("stereo_camera/left/image_raw", "sensor_msgs/msg/Image[gz.msgs.Image", None),
        # ("stereo_camera/right/image_raw", "sensor_msgs/msg/Image[gz.msgs.Image", None),
        ("front_camera/image_raw", "sensor_msgs/msg/Image[gz.msgs.Image", None),
    ]
    if ns:
        # Historical bridge kept for the namespaced flavour.
        entries.append(("camera_info", "sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        ros + "kinect_camera/camera_info"))
    entries += [
        ("kinect_camera/camera_info", "sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo", None),
        # ("stereo_camera/left/camera_info", "sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo", None),
        # ("stereo_camera/right/camera_info", "sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo", None),
        ("front_camera/camera_info", "sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo", None),
        ("kinect_camera/points", "sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked", None),
        ("imu", "sensor_msgs/msg/Imu[gz.msgs.IMU", None),
    ]

    arguments, remappings = [], []
    for suffix, types, ros_topic in entries:
        gz_topic = gz + suffix
        arguments.append(f"{gz_topic}@{types}")
        ros_topic = ros_topic or ros + suffix
        if ros_topic != gz_topic:
            remappings.append((gz_topic, ros_topic))

    joint_state = f"/world/default/model/{name}/joint_state"
    arguments.append(f"{joint_state}@sensor_msgs/msg/JointState[gz.msgs.Model")
    remappings.append((joint_state, ros + "joint_states"))
    return arguments, remappings


def launch_setup(context, *args, **kwargs):
    bcr_bot_path = get_package_share_directory("bcr_bot")

    def cfg(key):
        return LaunchConfiguration(key).perform(context)

    ns = cfg("robot_namespace")
    name = ns or DEFAULT_NAME
    position_x, position_y, orientation_yaw = cfg("position_x"), cfg("position_y"), cfg("orientation_yaw")
    launch_nav2 = _is_true(cfg("launch_nav2"))
    tf_relay = bool(ns)

    if launch_nav2 and not ns:
        raise RuntimeError("launch_nav2:=true needs a non-empty robot_namespace "
                           "(Nav2 runs inside the robot namespace with prefixed frames).")

    xacro_cmd = ["xacro ", join(bcr_bot_path, "urdf", "bcr_bot.xacro"),
                 " camera_enabled:=", cfg("camera_enabled"),
                 " stereo_camera_enabled:=", cfg("stereo_camera_enabled"),
                 " front_camera_enabled:=", cfg("front_camera_enabled"),
                 " two_d_lidar_enabled:=", cfg("two_d_lidar_enabled"),
                 " odometry_source:=", cfg("odometry_source"),
                 " sim_gz:=true"]
    if ns:
        xacro_cmd += [" robot_namespace:=", ns]

    # With relays the robot's TF lives in /<ns>/tf and is copied to the global /tf below.
    rsp_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")] if tf_relay else []

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=name,
        # str: otherwise launch_ros parses the URDF as YAML and any "word: " in a comment breaks it.
        parameters=[{"robot_description": ParameterValue(Command(xacro_cmd), value_type=str)},
                    {"use_sim_time": True}],
        remappings=rsp_remaps,
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", f"/{name}/robot_description",
            "-name", name,
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw,
        ],
        parameters=[{"use_sim_time": True}],
    )

    bridge_args, bridge_remaps = _bridge_entries(ns, name)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=name,
        name="gz_bridge",
        arguments=bridge_args,
        parameters=[{"use_sim_time": True}],
        remappings=bridge_remaps,
    )

    # Lift platform services (lift_up / lift_down, lift_state) over the bridged lift_cmd.
    lift_controller = Node(
        package="bcr_bot",
        executable="lift_controller.py",
        name="lift_controller",
        namespace=name,
        output="screen",
        parameters=[{"use_sim_time": True,
                     "joint_name": f"{ns}/lift_joint" if ns else "lift_joint"}],
    )

    actions = [robot_state_publisher, gz_spawn_entity, gz_ros2_bridge, lift_controller]

    # The camera is not compressed here: web_video_server subscribes to
    # /<name>/front_camera/image_raw and encodes MJPEG only while a client is watching.

    if tf_relay:
        # Copy the robot's namespaced TF to the global topics so Nav2 and RViz see it.
        # tf_static carries the fixed joints (base_footprint -> base_link, ...).
        for topic in ("tf", "tf_static"):
            actions.append(Node(
                package="topic_tools",
                executable="relay",
                name=f"{ns}_{topic}_relay",
                arguments=[f"/{ns}/{topic}", f"/{topic}"],
                output="screen",
                parameters=[{"use_sim_time": True}],
            ))

    if launch_nav2:
        nav2_args = {
            "robot_namespace": ns,
            "position_x": position_x,
            "position_y": position_y,
            "orientation_yaw": orientation_yaw,
        }
        # Only forward overrides; empty keeps the defaults of bcr_bot_nav2.launch.py.
        for spawn_key, nav2_key in (("nav2_map", "map"), ("nav2_params_file", "params_file")):
            if cfg(spawn_key):
                nav2_args[nav2_key] = cfg(spawn_key)

        # Scoped so one robot's Nav2 arguments do not leak into the next spawn.
        actions.append(GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(bcr_bot_path, "launch", "bcr_bot_nav2.launch.py")),
                launch_arguments=nav2_args.items(),
            ),
        ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="",
                              description="Empty: upstream naming. Otherwise prefix for frames, nodes and topics."),
        DeclareLaunchArgument("camera_enabled", default_value="True",
                              description="Enable the kinect depth camera"),
        DeclareLaunchArgument("stereo_camera_enabled", default_value="False",
                              description="Enable the stereo camera"),
        DeclareLaunchArgument("front_camera_enabled", default_value="False",
                              description="Enable the front RGB camera (640x480)"),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value="True",
                              description="Enable the 2D lidar"),
        DeclareLaunchArgument("position_x", default_value="0.0", description="Spawn x [m]"),
        DeclareLaunchArgument("position_y", default_value="0.0", description="Spawn y [m]"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0", description="Spawn yaw [rad]"),
        DeclareLaunchArgument("odometry_source", default_value="world",
                              description="Odometry source: 'world' (ground truth) or 'encoders'"),
        DeclareLaunchArgument("launch_nav2", default_value="False",
                              description="Also start Nav2 (bcr_bot_nav2.launch.py) with the spawn pose as initial pose."),
        DeclareLaunchArgument("nav2_map", default_value="",
                              description="Map yaml for Nav2 (empty: bcr_bot_nav2.launch.py default)."),
        DeclareLaunchArgument("nav2_params_file", default_value="",
                              description="Nav2 params file (empty: bcr_bot_nav2.launch.py default)."),
        OpaqueFunction(function=launch_setup),
    ])
