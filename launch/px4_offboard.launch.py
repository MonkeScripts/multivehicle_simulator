from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    return LaunchDescription(
        [
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
