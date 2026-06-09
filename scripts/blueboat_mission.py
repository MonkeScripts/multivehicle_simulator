#!/usr/bin/env python3
"""Demo mission for the BlueBoat USV: drive a square course.

Waits for the first ``/blueboat/odom``, then publishes a square of waypoints
(in the ``odom`` frame, relative to the boat's start position) to
``/blueboat/goal_pose``. The LOS waypoint controller follows the line between
consecutive waypoints; this node just lays down the course and then idles.

"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class BlueBoatSquareMission(Node):
    def __init__(self):
        super().__init__("blueboat_square_mission")

        self.declare_parameter("side", 10.0)  # m, square edge length
        self.declare_parameter("frame_id", "odom")
        self.side = self.get_parameter("side").value
        self.frame_id = self.get_parameter("frame_id").value

        self.goal_pub = self.create_publisher(PoseStamped, "/blueboat/goal_pose", 10)
        self.create_subscription(Odometry, "/blueboat/odom", self.save_odom, 10)
        self._start = None
        self._published = False
        # Publish only once the controller is actually subscribed, so the
        # course isn't dropped before discovery completes (and never twice, or
        # the controller would queue duplicate waypoints).
        self.create_timer(0.2, self.try_publish)
        self.get_logger().info("Waiting for /blueboat/odom to lay down course...")

    def save_odom(self, msg: Odometry):
        if self._start is None:
            self._start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def try_publish(self):
        if self._published or self._start is None:
            return
        if self.goal_pub.get_subscription_count() == 0:
            return
        x0, y0 = self._start
        s = self.side
        # Axis-aligned square in the odom frame, returning to start.
        waypoints = [
            (x0 + s, y0),
            (x0 + s, y0 + s),
            (x0, y0 + s),
            (x0, y0),
        ]
        for wx, wy in waypoints:
            msg_out = PoseStamped()
            msg_out.header.frame_id = self.frame_id
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.pose.position.x = wx
            msg_out.pose.position.y = wy
            msg_out.pose.orientation.w = 1.0
            self.goal_pub.publish(msg_out)
            self.get_logger().info("Queued waypoint (%.2f, %.2f)" % (wx, wy))
        self._published = True
        self.get_logger().info("Course published (%d waypoints)." % len(waypoints))


def main(args=None):
    rclpy.init(args=args)
    node = BlueBoatSquareMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
