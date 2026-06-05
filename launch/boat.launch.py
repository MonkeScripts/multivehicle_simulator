from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Spawn the BlueBoat USV into an already-running Gazebo world.

    The world itself is started separately by ``world.launch.py`` (matching the
    ``bluerov.launch.py`` flow). This launch only attaches the vehicle via
    ``ros_gz_sim create`` — it does NOT start its own Gazebo instance.
    """

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    description_file = PathJoinSubstitution(
        [
            FindPackageShare("multivehicle_sim"),
            "models",
            "blueboat",
            "model.sdf",
        ]
    )

    # Attach the vehicle to the running world via `ros_gz_sim create`.
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
        parameters=[{"use_sim_time": use_sim_time}],
    )

    spawn_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawner,
            on_exit=LogInfo(msg="Robot Model Spawn Process Finished"),
        )
    )

    return [
        gz_spawner,
        spawn_exit_handler,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Flag to indicate whether to use simulation time",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="blueboat",
            description="Namespace",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="47.8",
            description="Initial x position",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="-415.4",
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
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
