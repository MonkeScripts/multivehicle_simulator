#!/usr/bin/env python3

import math

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped


class BlueROVSquareMission(Node):
    """Drive the BlueROV2 in a square relative to its own base_link.

    Each leg is a body-frame FLU offset (forward, left) applied from the pose at
    the START of that leg, while the vehicle HOLDS its initial heading — i.e. it
    strafes the square (forward, left, back, right) instead of yawing to face
    each waypoint. This matches how an ROV actually moves and avoids the
    spinning behaviour you get from publishing absolute map setpoints / facing
    each target.

    Each leg's body-frame offset is transformed into a map-frame
    `setpoint_position/local` target with tf2 (`do_transform_pose_stamped`
    against the live `map -> base_link` transform MAVROS publishes), so the
    square is always relative to the vehicle's own base_link, never the map
    origin.
    """

    # TF frames: MAVROS local_position publishes map -> base_link (tf.send=true).
    MAP_FRAME = "map"
    BASE_FRAME = "base_link"

    def __init__(self):
        super().__init__("bluerov_square_mission")

        mavros_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.target_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", mavros_qos
        )
        self.curr_pose_sub = self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_callback, mavros_qos
        )

        # tf2 buffer/listener — used to transform base_link offsets to map.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Service Clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Service state flags
        self.is_armed = False
        self.is_guided_mode = False
        self.services_initialized = False

        # Mission Params
        self.depth = -2.0            # absolute dive depth (m below surface)
        self.dist_threshold = 0.2

        # Per-leg body-frame FLU offsets (forward+, left+). Each leg is applied
        # from the pose at the start of that leg, with heading held constant ->
        # the vehicle strafes a square and returns to start.
        self.legs = [
            (5.0, 0.0),    # forward 5 m
            (0.0, 5.0),    # left 5 m
            (-5.0, 0.0),   # backward 5 m
            (0.0, -5.0),   # right 5 m
        ]
        self.leg_index = 0

        # Heading held for the whole mission (captured at START).
        self.hold_yaw = None
        # Current leg's absolute map-frame target (computed when the leg begins).
        self.target_x = None
        self.target_y = None

        self.current_pose = None
        self.state = "INIT"

        # IMPORTANT NOTE: DO NOT SET A HIGH SETPOINT PUBLISH RATE.
        # ROV WOULD KEEP REPLANNING ITS TRAJECTORY, CAUSING IT TO MOVE VERY SLOWLY
        self.timer = self.create_timer(1.0, self.main_loop)

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

    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw (radians) from a quaternion."""
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def leg_target_in_map(self, fwd, left):
        """Transform a body-frame FLU offset (forward, left) at the current
        base_link into a map-frame (x, y) target via tf2. Returns None if the
        map -> base_link transform isn't available yet.
        """
        pose_in_base = PoseStamped()
        pose_in_base.header.frame_id = self.BASE_FRAME
        pose_in_base.pose.position.x = fwd
        pose_in_base.pose.position.y = left
        pose_in_base.pose.position.z = 0.0
        pose_in_base.pose.orientation.w = 1.0
        try:
            tf = self.tf_buffer.lookup_transform(
                self.MAP_FRAME,
                self.BASE_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                f"TF {self.MAP_FRAME} <- {self.BASE_FRAME} not ready: {e}"
            )
            return None
        target = do_transform_pose_stamped(pose_in_base, tf)
        return target.pose.position.x, target.pose.position.y

    def main_loop(self):
        """Main state machine"""

        if self.state == "INIT":
            if not self.services_initialized:
                self.is_armed or self.arm()
                self.is_guided_mode or self.set_guided_mode()

                if self.is_armed and self.is_guided_mode:
                    self.services_initialized = True
                    self.state = "START"
                    self.get_logger().info("Initialization complete, starting mission")

        elif self.state == "START":
            if self.current_pose is None:
                self.get_logger().info("Waiting for pose data...")
                return
            # Hold the heading the vehicle starts with for the whole square.
            self.hold_yaw = self.get_yaw_from_quaternion(
                self.current_pose.pose.orientation
            )
            self.leg_index = 0
            self.state = "PLAN_LEG"
            self.get_logger().info(
                f"Beginning body-relative square (hold_yaw={self.hold_yaw:.2f})"
            )

        elif self.state == "PLAN_LEG":
            # Transform this leg's body-frame offset (relative to the CURRENT
            # base_link) into a map-frame target via tf2. Retry next tick if the
            # transform isn't available yet.
            fwd, left = self.legs[self.leg_index]
            target = self.leg_target_in_map(fwd, left)
            if target is None:
                return
            self.target_x, self.target_y = target
            self.state = "MOVING"
            self.get_logger().info(
                f"Leg {self.leg_index} (fwd={fwd}, left={left}) -> "
                f"target ({self.target_x:.2f}, {self.target_y:.2f})"
            )

        elif self.state == "MOVING":
            # Publish the (fixed) leg target, holding the mission heading.
            q = self.get_quaternion_from_yaw(self.hold_yaw)
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = self.target_x
            msg.pose.position.y = self.target_y
            msg.pose.position.z = self.depth
            msg.pose.orientation.x = q["x"]
            msg.pose.orientation.y = q["y"]
            msg.pose.orientation.z = q["z"]
            msg.pose.orientation.w = q["w"]
            self.target_pub.publish(msg)

            dx = self.target_x - self.current_pose.pose.position.x
            dy = self.target_y - self.current_pose.pose.position.y
            dist = math.hypot(dx, dy)
            self.get_logger().info(
                f"Leg {self.leg_index}: dist to target {dist:.2f} m"
            )

            if dist < self.dist_threshold:
                self.get_logger().info(f"Leg {self.leg_index} reached!")
                self.leg_index += 1
                if self.leg_index >= len(self.legs):
                    self.state = "FINISHED"
                else:
                    self.state = "PLAN_LEG"

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
