import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    OpaqueFunction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """Spawn the BlueROV2 into an already-running Gazebo world.

    The world itself is started separately by ``world.launch.py``. Here we only
    spawn the vehicle and bring up its support nodes (ArduSub, MAVROS, the
    ROS<->gz bridge). The world file is still read to derive the ArduSub home
    lat/lon from its <spherical_coordinates>.
    """

    pkg_multivehicle_sim = get_package_share_directory("multivehicle_sim")
    ardusub_params_file = os.path.join(pkg_multivehicle_sim, "config", "ardusub.parm")
    mavros_params_file = os.path.join(
        pkg_multivehicle_sim, "mavros_params", "sim_mavros_params.yaml"
    )
    bluerov_gz_bridge_config_file = os.path.join(
        pkg_multivehicle_sim, "config", "bluerov_gz_bridge.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    world_name = LaunchConfiguration("world_name")
    launch_ardusub = LaunchConfiguration("ardusub")
    launch_mavros = LaunchConfiguration("mavros")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    # Derive the ArduSub home (lat,lon,alt,heading) by parsing the same world
    # file world.launch.py loads. The world is NOT started here.
    # Fallback home if the world has no <spherical_coordinates>: the RobotX 2026
    # Singapore River course origin (matches robotx_2026_sg_river.world).
    home_lat = 1.2894320
    home_lon = 103.8553700
    home_alt = 0.0
    home_heading = 0.0
    w_name = world_name.perform(context)
    if w_name != "empty.sdf":
        world_full_path = os.path.join(
            pkg_multivehicle_sim, "worlds", f"{w_name}.world"
        )
        if os.path.exists(world_full_path):
            print(f"Parsing world file for coordinates: {world_full_path}")
            try:
                tree = ET.parse(world_full_path)
                root = tree.getroot()
                world_elem = root.find("world")
                if world_elem is not None:
                    sc = world_elem.find("spherical_coordinates")
                    if sc is not None:
                        lat_elem = sc.find("latitude_deg")
                        if lat_elem is not None:
                            home_lat = float(lat_elem.text)

                        lon_elem = sc.find("longitude_deg")
                        if lon_elem is not None:
                            home_lon = float(lon_elem.text)

                        elev_elem = sc.find("elevation")
                        if elev_elem is not None:
                            home_alt = float(elev_elem.text)

                        head_elem = sc.find("heading_deg")
                        if head_elem is not None:
                            home_heading = float(head_elem.text)

                print(f"[Launch] Found Coordinates: Lat={home_lat}, Lon={home_lon}")

            except Exception as e:
                print(f"XML Parsing failed: {e}. Using defaults.")
        else:
            print("World path not found or empty. Using default coordinates.")

    # Create the string for ArduSub: "Lat,Lon,Alt,Yaw"
    home_str = f"{home_lat},{home_lon},{home_alt},{home_heading}"

    # Launch ArduSub w/ SIM_JSON
    # -w: wipe eeprom
    # --home: start location (lat,lon,alt,yaw). Yaw is provided by Gazebo, so the start yaw value is ignored.
    # ardusub must be on the $PATH.
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
            home_str,
        ],
        output="screen",
        condition=IfCondition(launch_ardusub),
    )

    description_file = PathJoinSubstitution(
        [
            FindPackageShare("multivehicle_sim"),
            "models",
            "bluerov2",
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

    # Translate messages MAV <-> ROS
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        output="screen",
        # mavros_node is actually many nodes, so we can't override the name
        # name='mavros_node',
        parameters=[mavros_params_file],
        condition=IfCondition(launch_mavros),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bluerov_gz_bridge",
        parameters=[{"config_file": bluerov_gz_bridge_config_file}],
    )

    return [
        gz_spawner,
        spawn_exit_handler,
        mavros_node,
        ardusub_launch,
        gz_bridge,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Flag to indicate whether to use simulation time",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="robotx_2026_sg_river",
            description="World the vehicle is spawned into; read only to derive "
            "the ArduSub home coordinates (the world is launched by world.launch.py)",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="bluerov",
            description="Namespace",
        ),
        DeclareLaunchArgument(
            "x",
            default_value="47.824",
            description="Initial x position (default incident-zone centre)",
        ),
        DeclareLaunchArgument(
            "y",
            default_value="-415.373",
            description="Initial y position (default incident-zone centre)",
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
        DeclareLaunchArgument(
            "mavros", default_value="true", description="Launch mavros?"
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
