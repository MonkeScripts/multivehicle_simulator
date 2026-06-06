"""Bridge the PX4 UAV's groundtruth world pose to a nav_msgs/Odometry topic.

gz does publish every model's pose on ``/world/<world>/pose/info`` (gz.msgs.Pose_V); we bridge that to a
TFMessage and let ``gz_pose_to_odom.py`` extract the x500 transform and
republish it as ``/x500/odom`` so the dashboard incident_manager can consume the
UAV pose like the other vehicles.

Note: ``model_name`` must match the name gz assigns the PX4 drone. With
``px4 ... -i 1`` and SYS_AUTOSTART=4001 that is ``x500_1``; adjust if you change
the instance / airframe.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world_name").perform(context)
    model_name = LaunchConfiguration("model_name").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    pose_topic = LaunchConfiguration("pose_topic").perform(context)
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )

    # gz.msgs.Pose_V -> tf2_msgs/msg/TFMessage: each transform's child_frame_id
    # is the model name. Remap the bridge's default output (the gz topic name)
    # onto our pose_topic so the converter can find it.
    gz_pose_topic = f"/world/{world}/pose/info"
    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="uav_pose_info_bridge",
        arguments=[
            f"{gz_pose_topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[(gz_pose_topic, pose_topic)],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    converter = Node(
        package="multivehicle_sim",
        executable="gz_pose_to_odom.py",
        name="gz_pose_to_odom",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pose_topic": pose_topic,
                "model_name": model_name,
                "odom_topic": odom_topic,
            }
        ],
    )

    return [pose_bridge, converter]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Drive node clocks from /clock (sim time).",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="robotx_2026_sg_river",
                description="Running gz-sim world name; used to build the "
                "/world/<world>/pose/info gz topic.",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="x500_1",
                description="gz model name PX4 assigns the drone (x500_<instance>; "
                "x500_1 for `px4 ... -i 1`).",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/x500/odom",
                description="nav_msgs/Odometry output topic for the UAV pose.",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/x500/pose_info",
                description="Intermediate TFMessage topic the gz pose bridge "
                "publishes and the converter subscribes to.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
