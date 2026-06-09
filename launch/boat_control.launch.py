"""Bring up the BlueBoat custom control stack (mixer + LOS controller).

Layers (see the GNC plan):
  * blueboat_thrust_mixer        -- cmd_vel -> left/right thrust (ALWAYS on; the
                                    single thruster driver and the nav2 seam).
  * blueboat_odom_to_tf          -- odom->base_link TF (nav2 prep; ALWAYS on).
  * blueboat_waypoint_controller -- LOS guidance + heading/surge -> cmd_vel
                                    (gated by `use_controller`, default true).
  * blueboat_mission             -- demo square course (gated by `use_mission`).

This launch assumes the world and the boat are already up (world.launch.py +
boat.launch.py). Set `spawn:=true` to also include boat.launch.py here.

nav2 migration (later): launch with `use_controller:=false` and remap nav2's
`/cmd_vel` onto `/blueboat/cmd_vel` -- the mixer is unchanged.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("multivehicle_sim")
    control_params = os.path.join(pkg, "config", "blueboat_control.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_controller = LaunchConfiguration("use_controller")
    use_mission = LaunchConfiguration("use_mission")
    spawn = LaunchConfiguration("spawn")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "use_controller",
            default_value="true",
            description="Run the LOS waypoint controller (set false when nav2 "
            "provides /blueboat/cmd_vel instead).",
        ),
        DeclareLaunchArgument(
            "use_mission",
            default_value="false",
            description="Run the demo square-course mission node.",
        ),
        DeclareLaunchArgument(
            "spawn",
            default_value="false",
            description="Also include boat.launch.py to spawn the boat + bridge.",
        ),
    ]

    spawn_boat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("multivehicle_sim"),
                                  "launch", "boat.launch.py"])
        ),
        condition=IfCondition(spawn),
    )

    mixer = Node(
        package="multivehicle_sim",
        executable="blueboat_thrust_mixer.py",
        name="blueboat_thrust_mixer",
        output="screen",
        parameters=[control_params, {"use_sim_time": use_sim_time}],
    )

    # Broadcast the boat's odom->base_link under a per-vehicle TF namespace
    # (blueboat/odom -> blueboat/base_link). The gz OdometryPublisher stamps
    # /blueboat/odom with the generic frames "odom"/"base_link", identical to
    # the BlueROV2's — and MAVROS publishes map->base_link for the ROV. Left
    # un-prefixed, both vehicles fight over one global `base_link` on /tf and
    # every listener spams TF_OLD_DATA. Prefixing the boat's frames keeps the
    # two TF trees disjoint. (Control/mission are topic-based, so nothing
    # consumes these frames yet; this is purely nav2 prep.)
    odom_tf = Node(
        package="multivehicle_sim",
        executable="blueboat_odom_to_tf.py",
        name="blueboat_odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "frame_id": "blueboat/odom",
                "child_frame_id": "blueboat/base_link",
            }
        ],
    )

    controller = Node(
        package="multivehicle_sim",
        executable="blueboat_waypoint_controller.py",
        name="blueboat_waypoint_controller",
        output="screen",
        parameters=[control_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_controller),
    )

    mission = Node(
        package="multivehicle_sim",
        executable="blueboat_mission.py",
        name="blueboat_square_mission",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_mission),
    )

    return LaunchDescription(args + [spawn_boat, mixer, odom_tf, controller, mission])
