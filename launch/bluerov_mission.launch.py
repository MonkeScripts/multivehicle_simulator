from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ground_truth_node = Node(
        package="multivehicle_sim",
        executable="ground_truth_to_mavros.py",
        name="ground_truth_to_mavros",
        output="screen",
    )

    movement_node = Node(
        package="multivehicle_sim",
        executable="bluerov_movement.py",
        name="bluerov_movement",
        output="screen",
    )

    return LaunchDescription(
        [
            ground_truth_node,
            movement_node,
        ]
    )
