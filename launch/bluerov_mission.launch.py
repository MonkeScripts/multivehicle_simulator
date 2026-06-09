from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = {"use_sim_time": True}

    ground_truth_node = Node(
        package="multivehicle_sim",
        executable="ground_truth_to_mavros.py",
        name="ground_truth_to_mavros",
        output="screen",
        parameters=[use_sim_time],
    )

    movement_node = Node(
        package="multivehicle_sim",
        executable="bluerov_movement.py",
        name="bluerov_movement",
        output="screen",
        parameters=[use_sim_time],
    )

    return LaunchDescription(
        [
            ground_truth_node,
            movement_node,
        ]
    )
