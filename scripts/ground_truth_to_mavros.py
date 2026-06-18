#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomRepublisher(Node):
    def __init__(self):
        super().__init__("odom_republisher")

        self.declare_parameter("odom_topic", "/bluerov/odom")
        self.declare_parameter("out_topic", "/mavros/odometry/out")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")

        odom_topic = self.get_parameter("odom_topic").value
        out_topic = self.get_parameter("out_topic").value
        self._frame_id = self.get_parameter("frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value

        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, out_topic, 10)
        self.get_logger().info(
            f"Odometry republisher started ('{odom_topic}' -> '{out_topic}')."
        )

    def odom_callback(self, msg: Odometry):
        repub_msg = Odometry()
        repub_msg.header.frame_id = self._frame_id
        repub_msg.child_frame_id = self._child_frame_id
        repub_msg.pose = msg.pose
        repub_msg.twist = msg.twist
        self.publisher.publish(repub_msg)
        self.get_logger().debug("Republished odometry message.")


def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
