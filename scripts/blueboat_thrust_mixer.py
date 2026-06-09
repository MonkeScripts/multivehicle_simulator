#!/usr/bin/env python3
"""Differential-thrust allocation (mixer) for the BlueBoat USV.

It consumes ``geometry_msgs/Twist`` on ``/blueboat/cmd_vel`` and emits a per-thruster force (Newtons) on
``/blueboat/thrusters/{left,right}/thrust`` (std_msgs/Float64, bridged ROS->gz).

Interpretation of the Twist:
  * ``linear.x``  -> desired SURGE SPEED (m/s). Converted to a surge force by
    inverting the quadratic hull drag from the model SDF (``xUabsU`` = -58.42),
    so ``F_surge = surge_drag * u_d * |u_d|`` holds that speed in steady state
    (open-loop).
  * ``angular.z`` -> desired YAW RATE (rad/s). Converted to a yaw moment by a
    gain ``yaw_gain`` (N*m per rad/s).

Allocation (twin thrusters a moment arm ``d = track_width/2`` apart):
    F_left  = F_surge/2 - M_z/(2 d)
    F_right = F_surge/2 + M_z/(2 d)
Positive ``angular.z`` (REP-103, CCW) therefore makes the right (starboard)
thruster push harder -> the boat yaws left, as expected.

A watchdog publishes zero thrust if no ``cmd_vel`` arrives within
``cmd_timeout`` seconds, so a dead controller cannot leave the boat at full
throttle.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class BlueBoatThrustMixer(Node):
    def __init__(self):
        super().__init__("blueboat_thrust_mixer")

        # Geometry / actuator limits.
        self.declare_parameter("track_width", 0.59)  # m, port<->stbd spacing
        self.declare_parameter("max_thrust", 30.0)  # N, per-thruster clamp
        # Surge: quadratic-drag feedforward (|xUabsU| from the model SDF).
        self.declare_parameter("surge_drag", 58.42)  # N per (m/s)^2
        self.declare_parameter("surge_lin", 0.0)  # N per (m/s), optional
        self.declare_parameter("v_max", 2.0)  # m/s input clamp
        # Yaw: desired yaw-rate -> yaw moment.
        self.declare_parameter("yaw_gain", 30.0)  # N*m per (rad/s)
        self.declare_parameter("w_max", 1.0)  # rad/s input clamp
        self.declare_parameter("invert_yaw", False)  # flip turn direction
        # Topics / timing.
        self.declare_parameter("cmd_vel_topic", "/blueboat/cmd_vel")
        self.declare_parameter("left_thrust_topic", "/blueboat/thrusters/left/thrust")
        self.declare_parameter("right_thrust_topic", "/blueboat/thrusters/right/thrust")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout", 0.5)  # s, watchdog

        self.track_width = self.get_parameter("track_width").value
        self.max_thrust = self.get_parameter("max_thrust").value
        self.surge_drag = self.get_parameter("surge_drag").value
        self.surge_lin = self.get_parameter("surge_lin").value
        self.v_max = self.get_parameter("v_max").value
        self.yaw_gain = self.get_parameter("yaw_gain").value
        self.w_max = self.get_parameter("w_max").value
        self.invert_yaw = self.get_parameter("invert_yaw").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        rate = self.get_parameter("publish_rate_hz").value

        self.half_track = max(self.track_width / 2.0, 1e-3)

        self.left_pub = self.create_publisher(
            Float64, self.get_parameter("left_thrust_topic").value, 10
        )
        self.right_pub = self.create_publisher(
            Float64, self.get_parameter("right_thrust_topic").value, 10
        )
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value, self.cmd_vel_sub, 10
        )

        self._last_cmd = Twist()
        self._last_cmd_time = None
        self.create_timer(1.0 / rate, self.publish_thrust)

        self.get_logger().info(
            "BlueBoat thrust mixer up: track_width=%.3f m, max_thrust=%.1f N, "
            "surge_drag=%.2f, yaw_gain=%.2f, invert_yaw=%s"
            % (
                self.track_width,
                self.max_thrust,
                self.surge_drag,
                self.yaw_gain,
                self.invert_yaw,
            )
        )

    def cmd_vel_sub(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def publish_thrust(self):
        # Watchdog: zero thrust if the command is stale (or never received).
        stale = self._last_cmd_time is None or (
            (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
            > self.cmd_timeout
        )
        if stale:
            self._send(0.0, 0.0)
            return

        u = _clamp(self._last_cmd.linear.x, self.v_max)
        w = _clamp(self._last_cmd.angular.z, self.w_max)
        if self.invert_yaw:
            w = -w

        # Surge speed -> force via quadratic-drag inversion (+ optional linear).
        f_surge = self.surge_drag * u * abs(u) + self.surge_lin * u
        # Desired yaw rate -> yaw moment.
        m_z = self.yaw_gain * w

        f_left = f_surge / 2.0 - m_z / (2.0 * self.half_track)
        f_right = f_surge / 2.0 + m_z / (2.0 * self.half_track)

        # Proportional clamp: scale both sides together so the turn ratio (and
        # thus the commanded heading) is preserved under saturation.
        peak = max(abs(f_left), abs(f_right), self.max_thrust)
        if peak > self.max_thrust:
            scale = self.max_thrust / peak
            f_left *= scale
            f_right *= scale

        self._send(f_left, f_right)

    def _send(self, left, right):
        self.left_pub.publish(Float64(data=float(left)))
        self.right_pub.publish(Float64(data=float(right)))


def main(args=None):
    rclpy.init(args=args)
    node = BlueBoatThrustMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
