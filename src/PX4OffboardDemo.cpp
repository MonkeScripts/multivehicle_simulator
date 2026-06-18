// ============================================================================
// PX4OffboardDemo.cpp  —  see PX4OffboardDemo.hpp for the mission overview.
// SPDX-License-Identifier: Apache-2.0
// ============================================================================
#include "PX4OffboardDemo.hpp"

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace {
// Wrap an angle into (-pi, pi].
float wrapToPi(float angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

OffboardDemo::OffboardDemo() : Node("px4_offboard_demo") {
  _ns = this->declare_parameter<std::string>("vehicle_namespace", "");
  _sys_id = this->declare_parameter<int>("vehicle_id", 0);
  _prefix = _ns.empty() ? std::string() : "/" + _ns;

  RCLCPP_INFO(get_logger(), "vehicle_namespace='%s' (topic prefix '%s'), target_system=%d",
              _ns.c_str(), _prefix.c_str(), _sys_id);

  _cmd_pub = create_publisher<VehicleCommand>(_prefix + "/fmu/in/vehicle_command", 10);
  _setpoint_pub = create_publisher<TrajectorySetpoint>(_prefix + "/fmu/in/trajectory_setpoint", 10);
  _offboard_mode_pub = create_publisher<OffboardControlMode>(_prefix + "/fmu/in/offboard_control_mode", 10);

  // PX4 publishes telemetry best-effort; match it or subscriptions stay empty.
  _status_sub = create_subscription<VehicleStatus>(
      _prefix + "/fmu/out/vehicle_status_v1", rclcpp::QoS(1).best_effort(),
      std::bind(&OffboardDemo::onStatus, this, _1));
  _position_sub = create_subscription<VehicleLocalPosition>(
      _prefix + "/fmu/out/vehicle_local_position", rclcpp::QoS(10).best_effort(),
      std::bind(&OffboardDemo::onLocalPosition, this, _1));
  _land_sub = create_subscription<VehicleLandDetected>(
      _prefix + "/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
      std::bind(&OffboardDemo::onLandDetected, this, _1));

  // Stream the offboard-mode heartbeat independently of the mission tick.
  _heartbeat_timer = create_wall_timer(100ms, std::bind(&OffboardDemo::streamOffboardHeartbeat, this));

  _phase_since = now();

  // Prime the setpoint stream with a no-op so PX4 sees a setpoint before we ask
  // for offboard mode.
  publishPosition(Eigen::Vector3f(NAN, NAN, NAN), NAN);

  RCLCPP_INFO(get_logger(), "OffboardDemo initialised.");
}

void OffboardDemo::onStatus(const VehicleStatus::SharedPtr msg) { _status = *msg; }

void OffboardDemo::onLocalPosition(const VehicleLocalPosition::SharedPtr msg) { _position = *msg; }

void OffboardDemo::onLandDetected(const VehicleLandDetected::SharedPtr msg) { _landed = msg->landed; }

uint64_t OffboardDemo::nowMicros() const {
  return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
}

void OffboardDemo::streamOffboardHeartbeat() {
  OffboardControlMode mode{};
  mode.timestamp = nowMicros();
  mode.position = true;  // we only ever command position setpoints
  _offboard_mode_pub->publish(mode);
}

void OffboardDemo::publishPosition(const Eigen::Vector3f& pos_ned, float yaw) {
  TrajectorySetpoint sp{};
  sp.timestamp = nowMicros();
  sp.position = {pos_ned.x(), pos_ned.y(), pos_ned.z()};
  sp.yaw = yaw;  // NAN = let PX4 hold the current heading
  _setpoint_pub->publish(sp);
}

void OffboardDemo::sendCommand(uint32_t command, float param1, float param2) {
  VehicleCommand cmd{};
  cmd.timestamp = nowMicros();
  cmd.command = command;
  cmd.param1 = param1;
  cmd.param2 = param2;
  cmd.target_system = static_cast<uint8_t>(_sys_id);
  cmd.target_component = 1;
  cmd.source_system = 1;
  cmd.source_component = 1;
  cmd.from_external = true;
  _cmd_pub->publish(cmd);
}

void OffboardDemo::requestOffboardMode() {
  // base mode = custom (1), custom main mode = OFFBOARD (6).
  sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void OffboardDemo::arm() {
  sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void OffboardDemo::disarm() {
  sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

void OffboardDemo::commandLand() {
  sendCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
}

bool OffboardDemo::phaseElapsed(double seconds) const {
  return (now() - _phase_since).seconds() > seconds;
}

bool OffboardDemo::reached(const Eigen::Vector3f& target, float tol) const {
  return std::fabs(_position.x - target.x()) < tol &&
         std::fabs(_position.y - target.y()) < tol &&
         std::fabs(_position.z - target.z()) < tol;
}

bool OffboardDemo::yawReached(float target_yaw, float tol) const {
  return std::fabs(wrapToPi(target_yaw - _position.heading)) < tol;
}

void OffboardDemo::transitionTo(Phase next, const char* note) {
  _phase = next;
  _phase_since = now();
  RCLCPP_INFO(get_logger(), "%s", note);
}

void OffboardDemo::tick() {
  const Eigen::Vector3f standoff(kStandoffNorth, kStandoffEast, kStandoffDown);

  switch (_phase) {
    case Phase::WaitForReady:
      // Let the autopilot settle, then latch the launch point as home (climbed
      // up by kClimbHeight) and request offboard.
      if (_status.pre_flight_checks_pass && phaseElapsed(kReadyHold)) {
        _home_ned = Eigen::Vector3f(_position.x, _position.y, _position.z - kClimbHeight);
        requestOffboardMode();
        transitionTo(Phase::RequestArm, "Ready -> requesting OFFBOARD");
      }
      break;

    case Phase::RequestArm:
      if (_status.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        arm();
        transitionTo(Phase::AwaitArmed, "OFFBOARD -> arming");
      }
      break;

    case Phase::AwaitArmed:
      if (_status.arming_state == VehicleStatus::ARMING_STATE_ARMED) {
        transitionTo(Phase::ClimbOut, "Armed -> climbing out");
      }
      break;

    case Phase::ClimbOut:
      // Hold the home setpoint; the vehicle climbs to it. Move on after a fixed
      // settle so we reach altitude before transiting.
      publishPosition(_home_ned, NAN);
      if (phaseElapsed(kClimbHold)) {
        transitionTo(Phase::GoToStandoff, "Climb-out -> standoff transit");
      }
      break;

    case Phase::GoToStandoff:
      publishPosition(standoff, NAN);
      if (reached(standoff)) {
        _scan_yaw_target = _position.heading;
        _scan_yaw_swept = 0.0f;
        transitionTo(Phase::ScanYaw, "At standoff -> yaw scan");
      }
      break;

    case Phase::ScanYaw:
      // Hold position and step the yaw target one notch each time we settle on
      // the previous one, until a full revolution has been swept.
      if (yawReached(_scan_yaw_target)) {
        _scan_yaw_target = wrapToPi(_scan_yaw_target + kYawStep);
        _scan_yaw_swept += kYawStep;
      }
      publishPosition(standoff, _scan_yaw_target);
      if (_scan_yaw_swept >= 2.0f * static_cast<float>(M_PI)) {
        transitionTo(Phase::ReturnHome, "Yaw scan complete -> returning home");
      }
      break;

    case Phase::ReturnHome:
      publishPosition(_home_ned, NAN);
      if (reached(_home_ned)) {
        commandLand();
        transitionTo(Phase::Descend, "Home -> landing");
      }
      break;

    case Phase::Descend:
      if (_landed) {
        disarm();
        transitionTo(Phase::Finished, "Touchdown -> disarmed");
      }
      break;

    case Phase::Finished:
      break;
  }
}

void OffboardDemo::run() {
  while (rclcpp::ok() && _phase != Phase::Finished) {
    rclcpp::spin_some(this->get_node_base_interface());
    tick();
    rclcpp::sleep_for(10ms);
  }
  RCLCPP_INFO(get_logger(), "Mission complete.");
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardDemo>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
