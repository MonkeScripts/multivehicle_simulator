from multivehicle_sim.bridge import Bridge, BridgeDirection


def robotx_obstacle_pose(model_name):
    return Bridge(
        gz_topic=f"/robotx/{model_name}/pose",
        ros_topic="/robotx/obstacles",
        gz_type="gz.msgs.Odometry",
        ros_type="nav_msgs/msg/Odometry",
        direction=BridgeDirection.GZ_TO_ROS,
    )


def led_change_mode(buoy_name, group_prefix="led"):
    """ROS->gz bridge for a safe_passage_buoy's LedPlugin mode-change topic.

    The LedPlugin (no <led_group_name> set) defaults its group name to
    led_<model instance name>, so a buoy included as <name>sp_entry</name>
    listens on /led_sp_entry/change_led_mode. We bridge that to an identically
    named ROS topic carrying std_msgs/String; led_beacon_driver publishes the
    mode name (e.g. "flash_red", "steady_blue", "off") onto it.
    """
    topic = f"/{group_prefix}_{buoy_name}/change_led_mode"
    return Bridge(
        gz_topic=topic,
        ros_topic=topic,
        gz_type="gz.msgs.StringMsg",
        ros_type="std_msgs/msg/String",
        direction=BridgeDirection.ROS_TO_GZ,
    )


def docking_led_change_mode(group_name):
    topic = f"/{group_name}/change_led_mode"
    return Bridge(
        gz_topic=topic,
        ros_topic=topic,
        gz_type="gz.msgs.StringMsg",
        ros_type="std_msgs/msg/String",
        direction=BridgeDirection.ROS_TO_GZ,
    )
