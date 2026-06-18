// ============================================================================
// PX4OffboardDemo.hpp  —  offboard control demo for the x500 UAV
//
// Flies a scripted offboard mission on a PX4 vehicle: wait until the autopilot
// is ready, switch to offboard and arm, climb above the launch point, transit
// to a standoff observation pose, sweep a full 360-degree yaw turn in place (so
// a forward-facing camera scans the buoy field), then return to launch and
// land.
//
// The offboard mechanics are the standard PX4 pattern. It is a continuous
// OffboardControlMode + TrajectorySetpoint stream over uXRCE-DDS, with mode and
// arming driven by VehicleCommand. That pattern follows PX4's BSD-3 ROS 2
// examples (and is demonstrated in the Dronecode ROSCon-25 workshop); this
// implementation was written for multivehicle_sim.
//
// SPDX-License-Identifier: Apache-2.0
// ============================================================================
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

// Drives a single PX4 vehicle through a fixed offboard mission. All PX4 topics
// are prefixed with the vehicle namespace (e.g. "/x500/fmu/...") so the node
// can target one instance among several.
class OffboardDemo : public rclcpp::Node {
 public:
  OffboardDemo();

  // Blocking: spins the node and advances the mission until it completes.
  void run();

 private:
  // Mission phases, in the order they execute.
  enum class Phase {
    WaitForReady,   // hold until pre-flight checks pass, then request offboard
    RequestArm,     // offboard accepted -> send the arm command
    AwaitArmed,     // wait for the autopilot to report ARMED
    ClimbOut,       // hold the climb setpoint above launch
    GoToStandoff,   // transit to the standoff observation point
    ScanYaw,        // rotate 360 degrees in place while holding position
    ReturnHome,     // fly back to the launch point
    Descend,        // land and wait for touchdown
    Finished,
  };

  // Subscription callbacks (latch the latest message).
  void onStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

  // Keepalive: PX4 only stays in offboard while this stream is fresh.
  void streamOffboardHeartbeat();

  // Outgoing commands / setpoints.
  void sendCommand(uint32_t command, float param1 = NAN, float param2 = NAN);
  void requestOffboardMode();
  void arm();
  void disarm();
  void commandLand();
  void publishPosition(const Eigen::Vector3f& pos_ned, float yaw);

  // Mission step + small predicates.
  void tick();
  void transitionTo(Phase next, const char* note);
  bool phaseElapsed(double seconds) const;
  bool reached(const Eigen::Vector3f& target, float tol = kReachedTol) const;
  bool yawReached(float target_yaw, float tol = kYawTol) const;
  uint64_t nowMicros() const;

  // ---- parameters ----
  std::string _ns;          // vehicle_namespace
  std::string _prefix;      // "/<ns>" (or "" when unset)
  int _sys_id = 0;          // vehicle_id -> MAVLink target_system

  // ---- ROS entities ----
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _cmd_pub;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _setpoint_pub;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_mode_pub;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _position_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_sub;
  rclcpp::TimerBase::SharedPtr _heartbeat_timer;

  // ---- latest telemetry ----
  px4_msgs::msg::VehicleStatus _status{};
  px4_msgs::msg::VehicleLocalPosition _position{};
  bool _landed = false;

  // ---- mission state ----
  Phase _phase = Phase::WaitForReady;
  rclcpp::Time _phase_since;
  Eigen::Vector3f _home_ned = Eigen::Vector3f::Zero();  // launch point + climb (NED)
  float _scan_yaw_target = 0.0f;   // current yaw setpoint during ScanYaw
  float _scan_yaw_swept = 0.0f;    // total yaw rotated so far

  // ---- mission constants (NED; z is down-positive) ----
  static constexpr float kClimbHeight = 10.0f;     // m above the launch deck
  static constexpr float kStandoffNorth = -14.0f;  // standoff observation point
  static constexpr float kStandoffEast = -25.0f;
  static constexpr float kStandoffDown = -6.0f;
  static constexpr float kReachedTol = 0.1f;       // m, position-reached tolerance
  static constexpr float kYawStep = static_cast<float>(M_PI / 180.0);  // 1 deg
  static constexpr float kYawTol = static_cast<float>(M_PI / 180.0);   // 1 deg
  static constexpr double kReadyHold = 2.0;        // s, settle before offboard
  static constexpr double kClimbHold = 10.0;       // s, hold the climb setpoint
};
