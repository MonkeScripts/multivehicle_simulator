import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="multivehicle_sim",
                executable="ground_truth_to_mavros.py",
                name="ground_truth_to_mavros",
                output="screen",
            ),
            Node(
                package="multivehicle_sim",
                executable="bluerov_movement.py",
                name="bluerov_movement",
                output="screen",
            ),
        ]
    )
