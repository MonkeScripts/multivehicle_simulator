import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare("multivehicle_sim").find("multivehicle_sim")

    clock_bridge_config_file = os.path.join(pkg_share, "config", "gz_bridge.yaml")

    return LaunchDescription(
        [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_clock_bridge",
                parameters=[{"config_file": clock_bridge_config_file}],
            ),
            Node(
                package="multivehicle_sim",
                executable="px4_offboard_demo",
                name="px4_offboard_demo",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"vehicle_namespace": "px4_1"},
                    {"vehicle_id": 2},
                ],
            ),
        ]
    )
