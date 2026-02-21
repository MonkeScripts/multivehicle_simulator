#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from dave_interfaces.msg import DVL


class DVLToMavros(Node):
    def __init__(self):
        super().__init__("dvl_to_mavros")
        self.subscription = self.create_subscription(
            DVL, "/dvl/velocity", self.dvl_callback, 10
        )
        self.publisher = self.create_publisher(Odometry, "/mavros/odometry/out", 10)
        self.get_logger().info("DVL to MAVROS odometry node started.")

    def dvl_callback(self, msg: DVL):
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        # Zero position; identity quaternion orientation
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # High covariance on pose so ArduPilot EKF treats position as unreliable
        odom.pose.covariance[0] = 99999.0   # x
        odom.pose.covariance[7] = 99999.0   # y
        odom.pose.covariance[14] = 99999.0  # z
        odom.pose.covariance[21] = 99999.0  # roll
        odom.pose.covariance[28] = 99999.0  # pitch
        odom.pose.covariance[35] = 99999.0  # yaw

        # DVL body-frame velocity
        odom.twist.twist.linear = msg.velocity.twist.linear
        odom.twist.covariance = msg.velocity.covariance

        self.publisher.publish(odom)
        self.get_logger().debug("Published DVL-based odometry to MAVROS.")


def main(args=None):
    rclpy.init(args=args)
    node = DVLToMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
