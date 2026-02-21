import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_dvl_arg = DeclareLaunchArgument(
        "use_dvl",
        default_value="false",
        description="Use DVL velocity for MAVROS odometry instead of ground truth",
    )

    use_dvl = LaunchConfiguration("use_dvl")

    ground_truth_node = Node(
        package="multivehicle_sim",
        executable="ground_truth_to_mavros.py",
        name="ground_truth_to_mavros",
        output="screen",
        condition=UnlessCondition(use_dvl),
    )

    dvl_node = Node(
        package="multivehicle_sim",
        executable="dvl_to_mavros.py",
        name="dvl_to_mavros",
        output="screen",
        condition=IfCondition(use_dvl),
    )

    movement_node = Node(
        package="multivehicle_sim",
        executable="bluerov_movement.py",
        name="bluerov_movement",
        output="screen",
    )

    return LaunchDescription(
        [
            use_dvl_arg,
            ground_truth_node,
            dvl_node,
            # movement_node,
        ]
    )
