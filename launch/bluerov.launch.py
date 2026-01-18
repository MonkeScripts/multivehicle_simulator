import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    pkg_multivehicle_sim = get_package_share_directory("multivehicle_sim")
    pkg_orca_bringup = get_package_share_directory("orca_bringup")
    ardusub_params_file = os.path.join(pkg_orca_bringup, "cfg", "sub.parm")
    mavros_params_file = os.path.join(
        pkg_orca_bringup, "params", "sim_mavros_params.yaml"
    )
    orca_params_file = os.path.join(pkg_orca_bringup, "params", "sim_orca_params.yaml")
    rosbag2_record_qos_file = os.path.join(
        pkg_orca_bringup, "params", "rosbag2_record_qos.yaml"
    )
    rviz_file = os.path.join(pkg_orca_bringup, "cfg", "sim_launch.rviz")

    paused = LaunchConfiguration("paused")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    debug = LaunchConfiguration("debug")
    headless = LaunchConfiguration("headless")
    verbose = LaunchConfiguration("verbose")
    namespace = LaunchConfiguration("namespace")
    world_name = LaunchConfiguration("world_name")
    launch_ardusub = LaunchConfiguration("ardusub")
    launch_mavros = LaunchConfiguration("mavros")
    launch_bag = LaunchConfiguration("bag")
    launch_rviz = LaunchConfiguration("rviz")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    if world_name.perform(context) != "empty.sdf":
        w_name = world_name.perform(context)
        world_filename = f"{w_name}.world"
        world_filepath = PathJoinSubstitution(
            [FindPackageShare("multivehicle_sim"), "worlds", world_filename]
        )
        gz_args = [world_filepath]
    else:
        gz_args = [world_name]

    if headless.perform(context) == "true":
        gz_args.append(" -s")
    if paused.perform(context) == "false":
        gz_args.append(" -r")
    if debug.perform(context) == "true":
        gz_args.append(" -v ")
        gz_args.append(verbose.perform(context))

    # Find lat lon to initiate ardusub with
    # world_full_path = os.path.join(pkg_multivehicle_sim, "worlds", world_filename)

    # Launch ArduSub w/ SIM_JSON
    # -w: wipe eeprom
    # --home: start location (lat,lon,alt,yaw). Yaw is provided by Gazebo, so the start yaw value is ignored.
    # ardusub must be on the $PATH, see src/orca4/setup.bash
    ardusub_launch = ExecuteProcess(
        cmd=[
            "ardusub",
            "-S",
            "-w",
            "-M",
            "JSON",
            "--defaults",
            ardusub_params_file,
            "-I0",
            "--home",
            "33.810313,-118.39386700000001,0.0,0",
        ],
        output="screen",
        condition=IfCondition(launch_ardusub),
    )
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

    description_file = PathJoinSubstitution(
        [
            FindPackageShare("multivehicle_sim"),
            "models",
            "bluerov2",
            "model.sdf",
        ]
    )

    gz_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            namespace,
            "-file",
            description_file,
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
        output="both",
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    spawn_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawner,
            on_exit=LogInfo(msg="Robot Model Spawn Process Finished"),
        )
    )

    orca_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_orca_bringup, "launch", "bringup.py")
        ),
        launch_arguments={
            "base": LaunchConfiguration("base"),
            "mavros": launch_mavros,
            "mavros_params_file": mavros_params_file,
            "nav": LaunchConfiguration("nav"),
            "orca_params_file": orca_params_file,
            "slam": LaunchConfiguration("slam"),
        }.items(),
    )
    bag_process = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--qos-profile-overrides-path",
            rosbag2_record_qos_file,
            "--include-hidden-topics",
            "/cmd_vel",
            "/mavros/state",
            "/odom",
            "/rosout",
            "/tf",
            "/tf_static",
        ],
        output="screen",
        condition=IfCondition(launch_bag),
    )

    rviz_process = ExecuteProcess(
        cmd=["rviz2", "-d", rviz_file],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    return [
        gz_sim_launch,
        gz_spawner,
        spawn_exit_handler,
        ardusub_launch,
        orca_bringup_launch,
        bag_process,
        rviz_process,
    ]


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
            default_value="empty.sdf",
            description="Gazebo world file to launch",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="bluerov",
            description="Namespace",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="0.0",
            description="Initial x position",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="0.0",
            description="Initial y position",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="0.0",
            description="Initial z position",
        ),
        DeclareLaunchArgument(
            "roll",
            default_value="0.0",
            description="Initial roll angle",
        ),
        DeclareLaunchArgument(
            "pitch",
            default_value="0.0",
            description="Initial pitch angle",
        ),
        DeclareLaunchArgument(
            "yaw",
            default_value="0.0",
            description="Initial yaw angle",
        ),
        DeclareLaunchArgument(
            "ardusub", default_value="true", description="Launch ArduSUB?"
        ),
        DeclareLaunchArgument("bag", default_value="False", description="Record bag?"),
        DeclareLaunchArgument(
            "base", default_value="true", description="Launch base controller?"
        ),
        DeclareLaunchArgument(
            "mavros", default_value="true", description="Launch mavros?"
        ),
        DeclareLaunchArgument(
            "nav", default_value="false", description="Launch navigation?"
        ),
        DeclareLaunchArgument("rviz", default_value="false", description="Launch rviz?"),
        DeclareLaunchArgument("slam", default_value="false", description="Launch SLAM?"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
