#!/usr/bin/env python3
"""Line-of-Sight (LOS) waypoint controller for the BlueBoat USV.

Guidance + control layers of the BlueBoat GNC stack. It turns a list of
waypoints into a ``geometry_msgs/Twist`` on ``/blueboat/cmd_vel`` for the thrust
mixer to allocate.

Line-of-Sight path following (Fossen):
  Track the *line between consecutive waypoints*, not the raw point, so the boat
  drives the cross-track error ``e`` to zero instead of spinning toward / over-
  shooting each point. For a leg from ``p_prev`` to ``p_tgt`` with path angle
  ``gamma``:
      e        = -(x - x_prev) sin(gamma) + (y - y_prev) cos(gamma)   # +ve = left
      Delta    = clamp(delta_min + k_delta |e|, delta_min, delta_max) # adaptive lookahead
      psi_d    = gamma + atan2(-e, Delta)
  A leg is finished when the boat passes the target (along-track progress >= leg
  length) or enters ``acceptance_radius``.

Control (heading psi & yaw-rate r from the pose/twist; surge u from twist.linear.x):
  * Heading: PID on wrap(psi_d - psi) with derivative-on-measurement (-Kd*r,
    no setpoint kick on waypoint switch) and an anti-windup integrator ->
    desired yaw rate ``angular.z``.
  * Surge:   cruise speed shaped by heading error and by the distance-to-goal slowdown -> desired ``linear.x``:
        u_d = u_cruise * max(0, cos(psi_err))**p * sat(dist_to_goal / r_slow)

Gait (``turn_in_place``) -- stop-and-turn at every waypoint:
  INIT -> ALIGN -> DRIVE -> (ALIGN -> DRIVE)* -> HOLD.
  * ALIGN: rotate in place to the next leg's heading while station-keeping the
    waypoint (surge nulls the along-body position error, braking any arrival
    momentum). Exits once the heading is aligned and the yaw rate has settled.
  * DRIVE: LOS path following; slows approaching *every* waypoint so the boat
    arrives slow enough to stop and turn (not just the final one).
  * HOLD: actively station-keep the final pose instead of going limp -- the boat
    holds position/heading rather than drifting. The USV is underactuated
    (surge + yaw only, no sway), so a pure sideways offset is corrected only by
    re-pointing at the hold point (``repoint_radius``); within
    ``acceptance_radius`` the surge deadbands to avoid thruster jitter.

Inputs:  ``/blueboat/odom`` (nav_msgs/Odometry); ``/blueboat/goal_pose``
         (geometry_msgs/PoseStamped, appended to the path); ``waypoints`` param
         (flat [x0,y0,x1,y1,...]) for a preloaded course.
Outputs: ``/blueboat/cmd_vel`` (geometry_msgs/Twist);
         ``/blueboat/goal_reached`` (std_msgs/Bool, latched true when the final
         waypoint is reached).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool


def wrap_to_pi(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    """Extract yaw (rad) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, limit):
    return max(-limit, min(limit, value))


class BlueBoatWaypointController(Node):
    def __init__(self):
        super().__init__("blueboat_waypoint_controller")

        self.declare_parameter("delta_min", 2.5)  # m, min lookahead
        self.declare_parameter("delta_max", 8.0)  # m, max lookahead
        self.declare_parameter("k_delta", 1.0)  # lookahead growth per |e|
        self.declare_parameter("acceptance_radius", 0.2)  # m, leg-done radius
        self.declare_parameter("u_cruise", 1.0)  # m/s nominal surge
        self.declare_parameter("turn_shaping_p", 1.5)  # cos() exponent
        self.declare_parameter("r_slow", 4.0)  # m, final-goal slowdown
        # --- Turn-in-place / station-keeping gait ----------------------------
        self.declare_parameter("turn_in_place", True)  # stop+rotate at each wp
        self.declare_parameter("arrival_radius", 0.6)  # m, DRIVE->ALIGN switch
        self.declare_parameter("align_tol_deg", 3.0)  # deg, heading-aligned exit
        self.declare_parameter("align_settle_degps", 5.0)  # deg/s, settled exit
        self.declare_parameter("kp_pos", 0.7)  # (m/s)/m, station-keep surge gain
        self.declare_parameter("u_align_max", 0.5)  # m/s, station-keep surge clamp
        self.declare_parameter("repoint_radius", 1.0)  # m, HOLD: drive back if drift>this
        self.declare_parameter("v_settle", 0.1)  # m/s, stop before turning in place
        # --- Heading PID (outputs desired yaw rate, rad/s) -------------------
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("ki_yaw", 0.0)
        self.declare_parameter("kd_yaw", 0.5)
        self.declare_parameter("w_max", 1.0)  # rad/s output clamp
        self.declare_parameter("i_max", 0.5)  # integrator clamp (rad/s)
        self.declare_parameter("control_rate_hz", 20.0)

        self.declare_parameter("odom_topic", "/blueboat/odom")
        self.declare_parameter("cmd_vel_topic", "/blueboat/cmd_vel")
        self.declare_parameter("goal_pose_topic", "/blueboat/goal_pose")

        self.delta_min = self.get_parameter("delta_min").value
        self.delta_max = self.get_parameter("delta_max").value
        self.k_delta = self.get_parameter("k_delta").value
        self.accept_r = self.get_parameter("acceptance_radius").value
        self.u_cruise = self.get_parameter("u_cruise").value
        self.shaping_p = self.get_parameter("turn_shaping_p").value
        self.r_slow = self.get_parameter("r_slow").value
        self.turn_in_place = self.get_parameter("turn_in_place").value
        self.arrival_radius = self.get_parameter("arrival_radius").value
        self.align_tol = math.radians(self.get_parameter("align_tol_deg").value)
        self.align_settle = math.radians(self.get_parameter("align_settle_degps").value)
        self.kp_pos = self.get_parameter("kp_pos").value
        self.u_align_max = self.get_parameter("u_align_max").value
        self.repoint_radius = self.get_parameter("repoint_radius").value
        self.v_settle = self.get_parameter("v_settle").value
        self.kp_yaw = self.get_parameter("kp_yaw").value
        self.ki_yaw = self.get_parameter("ki_yaw").value
        self.kd_yaw = self.get_parameter("kd_yaw").value
        self.w_max = self.get_parameter("w_max").value
        self.i_max = self.get_parameter("i_max").value
        self.dt = 1.0 / self.get_parameter("control_rate_hz").value

        # --- Path state ------------------------------------------------------
        self.waypoints = []
        self.wp_idx = 0
        self.seg_start = None  # (x, y) start of the current leg
        self.first_leg_start = None  # captured from odom when motion begins

        self.odom = None
        self.yaw_integral = 0.0
        self.finished = False
        # Gait state: INIT -> ALIGN -> DRIVE -> (ALIGN -> DRIVE)* -> HOLD.
        self.mode = "INIT"
        self.hold_xy = None  # (x, y) pose held while aligning / station-keeping
        self.target_heading = 0.0  # desired heading (rad) during ALIGN / HOLD

        # --- I/O -------------------------------------------------------------
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reached_pub = self.create_publisher(
            Bool, "/blueboat/goal_reached", latched
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_sub, 10
        )
        self.create_subscription(
            PoseStamped, self.get_parameter("goal_pose_topic").value, self.goal_sub, 10
        )

        self.create_timer(self.dt, self.control_step)
        self.get_logger().info(
            "BlueBoat waypoint controller up: %d preloaded waypoint(s), "
            "u_cruise=%.2f m/s, lookahead=[%.1f, %.1f] m"
            % (len(self.waypoints), self.u_cruise, self.delta_min, self.delta_max)
        )

    def odom_sub(self, msg: Odometry):
        self.odom = msg

    def goal_sub(self, msg: PoseStamped):
        """Append a goal to the path and resume if we had finished."""
        self.waypoints.append((msg.pose.position.x, msg.pose.position.y))
        if self.finished:
            self.finished = False
            self.reached_pub.publish(Bool(data=False))
        self.get_logger().info(
            "Goal appended (%.2f, %.2f); %d waypoint(s) queued"
            % (msg.pose.position.x, msg.pose.position.y, len(self.waypoints))
        )

    def control_step(self):
        if self.odom is None or not self.waypoints:
            return

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        yaw = yaw_from_quaternion(self.odom.pose.pose.orientation)
        r = self.odom.twist.twist.angular.z  # yaw rate (rad/s)

        if self.first_leg_start is None:
            self.first_leg_start = (x, y)

        # INIT: align to the first leg from rest (skip straight to DRIVE if the
        # turn-in-place gait is disabled).
        if self.mode == "INIT":
            self.seg_start = (x, y)
            self.hold_xy = (x, y)
            self.target_heading = self.leg_heading((x, y), self.waypoints[0])
            self.mode = "ALIGN" if self.turn_in_place else "DRIVE"

        # A goal appended after finishing re-opens the course -> resume.
        if self.finished and self.wp_idx < len(self.waypoints):
            self.finished = False
            self.reached_pub.publish(Bool(data=False))
            self.seg_start = (x, y)
            self.hold_xy = (x, y)
            self.target_heading = self.leg_heading((x, y), self.waypoints[self.wp_idx])
            self.mode = "ALIGN" if self.turn_in_place else "DRIVE"

        # All waypoints consumed -> actively station-keep the final pose.
        if self.wp_idx >= len(self.waypoints):
            if not self.finished:
                self.finished = True
                self.reached_pub.publish(Bool(data=True))
                self.get_logger().info("Final waypoint reached. Station-keeping.")
            self.mode = "HOLD"
            self.station_keep(x, y, yaw, r, allow_repoint=True)
            return

        # --- ALIGN: brake to a stop, THEN rotate in place to the next leg ----
        if self.mode == "ALIGN":
            u_now = self.odom.twist.twist.linear.x
            e_x = self.along_body_err(x, y, yaw)
            u_cmd = clamp(self.kp_pos * e_x, self.u_align_max)
            if abs(u_now) > self.v_settle:
                # Still coasting in from the leg: decelerate in a straight line
                # (no yaw) so the boat doesn't swing its nose while moving.
                # The turn only begins once it has actually stopped.
                self.publish_cmd(u_cmd, 0.0)
                return
            w_cmd, psi_err = self.heading_pid(self.target_heading, yaw, r)
            self.publish_cmd(u_cmd, w_cmd)
            if abs(psi_err) < self.align_tol and abs(r) < self.align_settle:
                self.mode = "DRIVE"
                self.yaw_integral = 0.0
                self.get_logger().info("Leg %d aligned -> driving." % self.wp_idx)
            return

        # --- DRIVE: Line-of-Sight path following ----------------------------
        sx, sy = self.seg_start if self.seg_start is not None else self.first_leg_start
        tx, ty = self.waypoints[self.wp_idx]
        seg_dx, seg_dy = tx - sx, ty - sy
        seg_len = math.hypot(seg_dx, seg_dy)
        dist_to_target = math.hypot(tx - x, ty - y)

        if seg_len < 1e-3:
            # Degenerate leg: just aim straight at the target.
            psi_d = math.atan2(ty - y, tx - x)
            along = 0.0
        else:
            gamma = math.atan2(seg_dy, seg_dx)
            # Signed cross-track error (positive to the left of the path).
            e = -(x - sx) * math.sin(gamma) + (y - sy) * math.cos(gamma)
            # Along-track progress along the leg.
            along = (x - sx) * math.cos(gamma) + (y - sy) * math.sin(gamma)
            delta = max(
                self.delta_min,
                min(self.delta_max, self.delta_min + self.k_delta * abs(e)),
            )
            psi_d = gamma + math.atan2(-e, delta)

        # Reached the waypoint: stop and (if enabled) turn in place to the next.
        passed = seg_len > 1e-3 and along >= seg_len
        if dist_to_target < self.arrival_radius or passed:
            self.get_logger().info("Leg %d reached." % self.wp_idx)
            self.seg_start = (tx, ty)
            self.hold_xy = (tx, ty)
            nxt = self.wp_idx + 1
            if nxt < len(self.waypoints):
                self.target_heading = self.leg_heading((tx, ty), self.waypoints[nxt])
            else:
                self.target_heading = yaw  # final: hold the heading we arrived on
            self.wp_idx = nxt
            self.yaw_integral = 0.0
            if self.turn_in_place and nxt < len(self.waypoints):
                self.mode = "ALIGN"
            return  # next tick handles the new leg / HOLD

        # --- Heading PID + surge --------------------------------------------
        w_cmd, psi_err = self.heading_pid(psi_d, yaw, r)
        # Turn-then-go: cut speed while heading error is large.
        turn_factor = max(0.0, math.cos(psi_err)) ** self.shaping_p
        # Slow approaching EVERY waypoint so the boat arrives slow enough to stop.
        slow = min(1.0, dist_to_target / self.r_slow)
        u_cmd = self.u_cruise * turn_factor * slow
        self.publish_cmd(u_cmd, w_cmd)

    # ---------------------------------------------------------------- helpers
    def publish_cmd(self, u, w):
        cmd = Twist()
        cmd.linear.x = float(u)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def heading_pid(self, psi_d, yaw, r):
        """Heading PID -> desired yaw rate. Returns (w_cmd, psi_err)."""
        psi_err = wrap_to_pi(psi_d - yaw)
        if self.ki_yaw > 0.0:
            # Integrate, then clamp so the integral CONTRIBUTION stays <= i_max.
            self.yaw_integral += psi_err * self.dt
            self.yaw_integral = clamp(self.yaw_integral, self.i_max / self.ki_yaw)
        else:
            self.yaw_integral = 0.0
        # Derivative on measurement (-Kd*r) avoids a kick when psi_d jumps.
        w_cmd = clamp(
            self.kp_yaw * psi_err + self.ki_yaw * self.yaw_integral - self.kd_yaw * r,
            self.w_max,
        )
        return w_cmd, psi_err

    def along_body_err(self, x, y, yaw):
        """Position error to hold_xy projected on the body x-axis (+ = ahead)."""
        dx = self.hold_xy[0] - x
        dy = self.hold_xy[1] - y
        return dx * math.cos(yaw) + dy * math.sin(yaw)

    @staticmethod
    def leg_heading(a, b):
        """Heading (rad) of the line from point a to point b."""
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def station_keep(self, x, y, yaw, r, allow_repoint):
        """Hold ``hold_xy``: surge nulls the along-body error while the heading
        holds ``target_heading``. If drift exceeds ``repoint_radius`` the boat
        points back at the hold point (the only way an underactuated USV can
        correct a sideways offset). Within ``acceptance_radius`` the surge
        deadbands so the thrusters don't jitter on station."""
        rng = math.hypot(self.hold_xy[0] - x, self.hold_xy[1] - y)
        if rng < self.accept_r:
            w_cmd, _ = self.heading_pid(self.target_heading, yaw, r)
            self.publish_cmd(0.0, w_cmd)
            return
        if allow_repoint and rng > self.repoint_radius:
            psi_d = math.atan2(self.hold_xy[1] - y, self.hold_xy[0] - x)
        else:
            psi_d = self.target_heading
        w_cmd, _ = self.heading_pid(psi_d, yaw, r)
        u_cmd = clamp(self.kp_pos * self.along_body_err(x, y, yaw), self.u_align_max)
        self.publish_cmd(u_cmd, w_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BlueBoatWaypointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
