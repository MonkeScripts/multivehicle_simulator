import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ENU→NED: rotation by π about axis (1,1,0)/√2
ENU_NED_Q = str(math.sqrt(2) / 2)


def launch_setup(context, *args, **kwargs):
    paused = LaunchConfiguration("paused")
    gui = LaunchConfiguration("gui")
    debug = LaunchConfiguration("debug")
    headless = LaunchConfiguration("headless")
    verbose = LaunchConfiguration("verbose")
    world_name = LaunchConfiguration("world_name")

    # Resolve the world file shipped in this package; "empty.sdf" loads the
    # built-in empty world instead.
    if world_name.perform(context) != "empty.sdf":
        w_name = world_name.perform(context)
        world_filename = f"{w_name}.world"
        world_filepath = PathJoinSubstitution(
            [FindPackageShare("multivehicle_sim"), "worlds", world_filename]
        )
        gz_args = [world_filepath]
    else:
        gz_args = [world_name.perform(context)]

    if headless.perform(context) == "true":
        gz_args.append(" -s")
    if paused.perform(context) == "false":
        gz_args.append(" -r")
    if debug.perform(context) == "true":
        gz_args.append(" -v ")
        gz_args.append(verbose.perform(context))

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("gz_args", gz_args),
        ],
        condition=IfCondition(gui),
    )

    # World-level static TF publishers (world → map, and the ENU→NED frames).
    world_tfs = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world2map",
            arguments=[
                "0",
                "0",
                "0",
                "0",
                "0",
                "0",
                "world",
                "map",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world2world_ned",
            arguments=[
                "0",
                "0",
                "0",
                ENU_NED_Q,
                ENU_NED_Q,
                "0",
                "0",
                "world",
                "world_ned",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map2map_ned",
            arguments=[
                "0",
                "0",
                "0",
                ENU_NED_Q,
                ENU_NED_Q,
                "0",
                "0",
                "map",
                "map_ned",
            ],
        ),
    ]

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory("multivehicle_sim"),
                    "config",
                    "gz_bridge.yaml",
                )
            }
        ],
    )

    return [gz_sim_launch, clock_bridge] + world_tfs


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "paused",
            default_value="false",
            description="Start the simulation paused",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Flag to enable the gazebo gui",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Flag to indicate whether to use simulation time",
        ),
        DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Flag to enable the gazebo debug flag",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Flag to enable the gazebo headless mode",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="0",
            description="Adjust level of console verbosity",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="robotx_2026_sg_river",
            description="Gazebo world file to launch",
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
