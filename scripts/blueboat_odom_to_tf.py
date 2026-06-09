#!/usr/bin/env python3
"""Broadcast the odom->base_link transform from a nav_msgs/Odometry topic."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__("blueboat_odom_to_tf")

        self.declare_parameter("odom_topic", "/blueboat/odom")
        self.declare_parameter("frame_id", "")  # "" -> use msg.header.frame_id
        self.declare_parameter("child_frame_id", "")  # "" -> use msg.child_frame_id
        self._frame_id = self.get_parameter("frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value

        self._br = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10
        )

    def _on_odom(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._frame_id or msg.header.frame_id or "odom"
        t.child_frame_id = self._child_frame_id or msg.child_frame_id or "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
