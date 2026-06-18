from launch import LaunchDescription
from launch_ros.actions import Node


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
                    # uXRCE-DDS topic namespace -> /x500/fmu/... . Must match
                    # PX4_UXRCE_DDS_NS=x500 set on the px4 SITL process.
                    {"vehicle_namespace": "x500"},
                    # target_system for VehicleCommand: MAV_SYS_ID = instance+1,
                    # so 2 for `px4 ... -i 1`.
                    {"vehicle_id": 2},
                ],
            ),
        ]
    )
