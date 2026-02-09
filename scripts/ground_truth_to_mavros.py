#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomRepublisher(Node):
    def __init__(self):
        super().__init__("odom_republisher")
        self.subscription = self.create_subscription(
            Odometry, "/bluerov/odom", self.odom_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, "/mavros/odometry/out", 10)
        self.get_logger().info("Odometry republisher node started.")

    def odom_callback(self, msg: Odometry):
        repub_msg = Odometry()
        repub_msg.header.frame_id = "map"
        repub_msg.child_frame_id = "base_link"
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
