#!/usr/bin/python3
"""Nav2 (map server + AMCL + navigation) for one bcr_bot already spawned in Gazebo.

Everything runs inside the robot namespace: nodes /<ns>/..., actions /<ns>/navigate_to_pose,
/<ns>/amcl_pose, and TF read from /<ns>/tf (the spawn launch keeps the robot TF there and
relays it to the global /tf). Several robots can run their own copy with different namespaces.

The params file uses two placeholders (see config/nav2_factory_params.yaml):
<robot_name> (TF frames, replaced here) and <robot_namespace> (topics, replaced by nav2_bringup).
"""

from os.path import join

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString


def _is_true(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def launch_setup(context, *args, **kwargs):
    def cfg(key):
        return LaunchConfiguration(key).perform(context)

    ns = cfg("robot_namespace").strip("/")
    if not ns:
        raise RuntimeError("bcr_bot_nav2.launch.py needs a non-empty robot_namespace: "
                           "the Nav2 frames are '<robot_namespace>/odom', '<robot_namespace>/base_link'.")

    use_sim_time = cfg("use_sim_time")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    params_file = ReplaceString(
        source_file=cfg("params_file"),
        replacements={"<robot_name>": ns},
    )

    actions = [
        # Scoped so the bringup arguments (map, namespace, ...) do not leak into the next robot.
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(nav2_bringup_dir, "launch", "bringup_launch.py")),
                launch_arguments={
                    "namespace": ns,
                    "use_namespace": "True",
                    "use_localization": "True",
                    "slam": "False",
                    "map": cfg("map"),
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": cfg("autostart"),
                }.items(),
            ),
        ]),
    ]

    if _is_true(cfg("set_initial_pose")):
        actions.append(Node(
            package="bcr_bot",
            executable="set_initial_pose.py",
            name=f"{ns}_initial_pose_publisher",
            output="screen",
            parameters=[{
                "x": float(cfg("position_x")),
                "y": float(cfg("position_y")),
                "z": 0.0,
                "yaw": float(cfg("orientation_yaw")),
                "delay": float(cfg("initial_pose_delay")),
                "robot_namespace": ns,
                "use_sim_time": _is_true(use_sim_time),
            }],
        ))

    if _is_true(cfg("rviz")):
        # Same node nav2_bringup/rviz_launch.py builds for a namespace, without its
        # "shutdown everything when RViz closes" handler. bcr_bot keeps its own copy of the view.
        rviz_config = ReplaceString(
            source_file=join(get_package_share_directory("bcr_bot"), "rviz", "nav2_namespaced_view.rviz"),
            replacements={"<robot_namespace>": f"/{ns}"},
        )
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            namespace=ns,
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": _is_true(use_sim_time)}],
            output="screen",
            remappings=[
                ("/map", "map"),
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
                ("/goal_pose", "goal_pose"),
                ("/clicked_point", "clicked_point"),
                ("/initialpose", "initialpose"),
            ],
        ))

    return actions


def generate_launch_description():
    bcr_bot_config = join(get_package_share_directory("bcr_bot"), "config")
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="bcr_bot",
                              description="Namespace of the spawned robot (must not be empty)."),
        DeclareLaunchArgument("map", default_value=join(bcr_bot_config, "factory_map_2.yaml"),
                              description="Map yaml for the map server."),
        DeclareLaunchArgument("params_file", default_value=join(bcr_bot_config, "nav2_factory_params.yaml"),
                              description="Nav2 params with <robot_name>/<robot_namespace> placeholders."),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        DeclareLaunchArgument("autostart", default_value="True"),
        DeclareLaunchArgument("set_initial_pose", default_value="True",
                              description="Send the initial pose to AMCL until it publishes amcl_pose."),
        DeclareLaunchArgument("position_x", default_value="0.0", description="Initial pose x in the map [m]"),
        DeclareLaunchArgument("position_y", default_value="0.0", description="Initial pose y in the map [m]"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0", description="Initial yaw in the map [rad]"),
        DeclareLaunchArgument("initial_pose_delay", default_value="10.0",
                              description="Seconds to wait after AMCL is up before sending the pose."),
        DeclareLaunchArgument("rviz", default_value="False", description="Start a namespaced RViz."),
        OpaqueFunction(function=launch_setup),
    ])
