#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math


class BlueROVSquareMission(Node):
    def __init__(self):
        super().__init__("bluerov_square_mission")

        mavros_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.target_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", mavros_qos
        )
        self.curr_pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_callback, mavros_qos
        )

        # Service Clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Service state flags
        self.is_armed = False
        self.is_guided_mode = False
        self.services_initialized = False

        # Mission Params
        self.side_length = 5.0
        self.depth = -2.0
        self.threshold = 0.2

        # State & Waypoints (X, Y)
        self.current_pose = None
        self.state = "INIT"
        self.waypoints = [
            (5.0, 0.0),  # Forward
            (5.0, 5.0),  # Right
            (0.0, 5.0),  # Backward
            (0.0, 0.0),  # Back to start
        ]
        self.wp_index = 0

        # Single timer for everything (10Hz)
        self.timer = self.create_timer(0.5, self.main_loop)

    def arm(self):
        """Request arming"""
        if not self.arming_client.service_is_ready():
            self.get_logger().info("Waiting for arming service...")
            return False

        self.get_logger().info("Requesting arm...")
        request = CommandBool.Request()
        request.value = True
        future = self.arming_client.call_async(request)
        future.add_done_callback(self.arm_callback)
        return True

    def arm_callback(self, future):
        """Callback for arming service"""
        try:
            response = future.result()
            if response.success:
                self.is_armed = True
                self.get_logger().info("Vehicle armed successfully")
            else:
                self.get_logger().warn("Failed to arm vehicle")
        except Exception as e:
            self.get_logger().error(f"Arming service call failed: {e}")

    def set_guided_mode(self):
        """Request GUIDED mode"""
        if not self.mode_client.service_is_ready():
            self.get_logger().info("Waiting for SetMode service...")
            return False

        self.get_logger().info("Requesting GUIDED mode...")
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.guided_mode_callback)
        return True

    def guided_mode_callback(self, future):
        """Callback for setting to guided mode"""
        try:
            response = future.result()
            if response.mode_sent:
                self.is_guided_mode = True
                self.get_logger().info("GUIDED mode set successfully")
            else:
                self.get_logger().warn("Failed to set GUIDED mode")
        except Exception as e:
            self.get_logger().error(f"Set mode service call failed: {e}")

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_quaternion_from_yaw(self, yaw):
        """Convert yaw (radians) to a quaternion for PoseStamped."""
        return {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}

    def main_loop(self):
        """Main state machine"""

        if self.state == "INIT":
            # Try to initialize services
            if not self.services_initialized:
                armed = self.is_armed or self.arm()
                guided = self.is_guided_mode or self.set_guided_mode()

                if self.is_armed and self.is_guided_mode:
                    self.services_initialized = True
                    self.state = "START"
                    self.get_logger().info("Initialization complete, starting mission")

        elif self.state == "START":
            if self.current_pose is None:
                self.get_logger().info("Waiting for pose data...")
                return
            self.state = "MOVING"
            self.get_logger().info("Beginning square mission")

        elif self.state == "MOVING":
            tx, ty = self.waypoints[self.wp_index]

            # Calculate Yaw to face the target
            dx = tx - self.current_pose.pose.position.x
            dy = ty - self.current_pose.pose.position.y
            yaw = math.atan2(dy, dx)
            q = self.get_quaternion_from_yaw(yaw)

            # Publish Pose
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = tx
            msg.pose.position.y = ty
            msg.pose.position.z = self.depth
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            self.target_pub.publish(msg)
            self.get_logger().info(f"Publishing waypoint: {msg.pose.position}")

            # Check distance
            dist = math.sqrt(dx**2 + dy**2)
            reach_threshold = dist < self.threshold
            self.get_logger().info(f"dist: {dist}, reach_threshold: {reach_threshold}")

            if dist < self.threshold:
                self.get_logger().info(f"Waypoint {self.wp_index} reached!")
                self.wp_index += 1
                if self.wp_index >= len(self.waypoints):
                    self.state = "FINISHED"

        elif self.state == "FINISHED":
            self.get_logger().info("Mission successful.")
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVSquareMission()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
