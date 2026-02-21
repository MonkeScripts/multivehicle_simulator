#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from dave_interfaces.msg import DVL


class DVLToMavros(Node):
    def __init__(self):
        super().__init__("dvl_to_mavros")

        self.latest_odom = None

        # Ground truth pose source
        self.odom_sub = self.create_subscription(
            Odometry, "/bluerov/odom", self.odom_callback, 10
        )

        # DVL velocity source (outputs in base_link FLU via reference_frame in SDF)
        self.declare_parameter("dvl_topic", "/bluerov/dvl/velocity")
        dvl_topic = self.get_parameter("dvl_topic").get_parameter_value().string_value
        self.dvl_sub = self.create_subscription(
            DVL, dvl_topic, self.dvl_callback, 10
        )

        self.odom_pub = self.create_publisher(Odometry, "/mavros/odometry/out", 10)

        self.get_logger().info("DVL to MAVROS node started (GT pose + DVL twist).")

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def dvl_callback(self, msg: DVL):
        if self.latest_odom is None:
            self.get_logger().warn("Waiting for ground truth odometry on /bluerov/odom...")
            return

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"

        # Pose from ground truth
        odom.pose = self.latest_odom.pose

        # Twist directly from DVL (already in base_link FLU per sensor reference_frame)
        odom.twist.twist.linear = msg.velocity.twist.linear
        odom.twist.covariance = msg.velocity.covariance

        self.odom_pub.publish(odom)
        self.get_logger().debug("Published hybrid odometry to MAVROS.")


def main(args=None):
    rclpy.init(args=args)
    node = DVLToMavros()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
