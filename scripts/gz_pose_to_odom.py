#!/usr/bin/env python3
"""Republish a gz model's groundtruth world pose as nav_msgs/Odometry."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


class GzPoseToOdom(Node):
    def __init__(self):
        super().__init__("gz_pose_to_odom")

        self.declare_parameter("pose_topic", "/x500/pose_info")
        self.declare_parameter("model_name", "x500_1")
        self.declare_parameter("odom_topic", "/x500/odom")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")

        self._model_name = (
            self.get_parameter("model_name").get_parameter_value().string_value
        )
        self._frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self._child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )
        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self._pub = self.create_publisher(Odometry, odom_topic, 10)
        self._sub = self.create_subscription(TFMessage, pose_topic, self._on_tf, 10)
        self.get_logger().info(
            f"Republishing gz model '{self._model_name}' pose from "
            f"'{pose_topic}' to '{odom_topic}'."
        )

    def _on_tf(self, msg: TFMessage):
        for tf in msg.transforms:
            if tf.child_frame_id != self._model_name:
                continue
            odom = Odometry()
            odom.header.stamp = tf.header.stamp
            odom.header.frame_id = self._frame_id
            odom.child_frame_id = self._child_frame_id
            odom.pose.pose.position.x = tf.transform.translation.x
            odom.pose.pose.position.y = tf.transform.translation.y
            odom.pose.pose.position.z = tf.transform.translation.z
            odom.pose.pose.orientation = tf.transform.rotation
            self._pub.publish(odom)
            return


def main(args=None):
    rclpy.init(args=args)
    node = GzPoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
