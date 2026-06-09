"""Bridge ALL of the PX4 UAV's gz interfaces into ROS with a single bridge.

`model_name` is the gz model PX4 spawns (`<model>_<instance>`):
  - x500_mono_cam_1       for SYS_AUTOSTART=4010 + `px4 -i 1` (front camera)
  - x500_mono_cam_down_1  for SYS_AUTOSTART=4014            (down camera)
  - x500_1                for SYS_AUTOSTART=4001            (no camera; set camera:=false)
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
    camera_ns = LaunchConfiguration("camera_ns").perform(context).rstrip("/")
    use_camera = LaunchConfiguration("camera").perform(context).lower() == "true"
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )

    # gz.msgs.Pose_V (every model's groundtruth pose) -> tf2_msgs/TFMessage.
    # gz_pose_to_odom picks out `model_name` and republishes it as odom_topic.
    gz_pose = f"/world/{world}/pose/info"
    bridge_args = [f"{gz_pose}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"]
    bridge_remaps = [(gz_pose, pose_topic)]

    if use_camera:
        sensor = f"/world/{world}/model/{model_name}/link/camera_link/sensor/imager"
        gz_image, gz_info = f"{sensor}/image", f"{sensor}/camera_info"
        bridge_args += [
            f"{gz_image}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ]
        bridge_remaps += [
            (gz_image, f"{camera_ns}/image"),
            (gz_info, f"{camera_ns}/camera_info"),
        ]

    uav_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="uav_gz_bridge",
        arguments=bridge_args,
        remappings=bridge_remaps,
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    pose_to_odom = Node(
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

    return [uav_bridge, pose_to_odom]


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
                "/world/<world>/... gz topic names.",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="x500_mono_cam_1",
                description="gz model name PX4 assigns the drone "
                "(<model>_<instance>): x500_mono_cam_1 for SYS_AUTOSTART=4010 "
                "+ `-i 1` (front cam); x500_mono_cam_down_1 for 4014 (down cam); "
                "x500_1 for 4001 (no cam, pair with camera:=false).",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/x500/odom",
                description="nav_msgs/Odometry output topic for the UAV pose.",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/x500/pose_info",
                description="Intermediate TFMessage topic the pose bridge "
                "publishes and gz_pose_to_odom subscribes to.",
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="true",
                description="Also bridge the front camera (image + camera_info). "
                "Set false for the plain x500 airframe (4001).",
            ),
            DeclareLaunchArgument(
                "camera_ns",
                default_value="/x500/front_cam",
                description="ROS namespace the camera is remapped under "
                "(-> <camera_ns>/image, <camera_ns>/camera_info).",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
