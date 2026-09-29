#include "chrono_supervisory_fmu_hw_interface/chrono_supervisory_fmu_system_interface.hpp"

#include <cmath>

namespace chrono_supervisory_fmu_hw_interface
{

using chrono_split_msgs::msg::VehicleCommand;
using chrono_split_msgs::msg::VehicleStatus;

namespace
{
constexpr double kDegPerRad = 180.0 / M_PI;
constexpr double kRadPerDeg = M_PI / 180.0;

rclcpp::Logger logger() { return rclcpp::get_logger("ChronoSupervisoryFmuSystemInterface"); }
}  // namespace

hardware_interface::CallbackReturn ChronoSupervisoryFmuSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional -- absent means false, same convention as
  // ChronoFmuSystemInterface's identical param (axle-divisor math only
  // here, see header comment -- this plugin has no FMU to set a
  // structural flag on).
  auto six_wheel_it = info_.hardware_parameters.find("six_wheel");
  six_wheel_ = (six_wheel_it != info_.hardware_parameters.end() && six_wheel_it->second == "true");

  // Same joint parsing/validation as ChronoFmuSystemInterface's on_init()
  // -- identical URDF joint declarations work for both plugins.
  joints_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() > 1) {
      RCLCPP_ERROR(
        logger(), "joint '%s' must declare at most one command_interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const bool has_command = !joint.command_interfaces.empty();
    const std::string iface =
      has_command ? joint.command_interfaces[0].name : hardware_interface::HW_IF_VELOCITY;
    if (iface != hardware_interface::HW_IF_POSITION && iface != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        logger(), "joint '%s' has unsupported interface '%s' (only position/velocity wired so far)",
        joint.name.c_str(), iface.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (iface == hardware_interface::HW_IF_POSITION) {
      if (
        joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(
          logger(), "joint '%s' must declare exactly one position state_interface",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      bool found_velocity = false;
      bool found_position = false;
      for (const auto & si : joint.state_interfaces) {
        found_velocity |= (si.name == hardware_interface::HW_IF_VELOCITY);
        found_position |= (si.name == hardware_interface::HW_IF_POSITION);
      }
      if (joint.state_interfaces.size() != 2 || !found_velocity || !found_position) {
        RCLCPP_ERROR(
          logger(),
          "joint '%s' must declare exactly velocity + position state_interfaces (position is "
          "dead-reckoned, for RViz)",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    joints_.push_back(JointIO{joint.name, iface, 0.0, 0.0, has_command});
  }

  // Same node-per-plugin/background-spin-thread pattern as
  // ChronoEcuBridgeSystemInterface -- but publishing/subscribing Bridge
  // 2's VehicleCommand/VehicleStatus directly instead of Bridge 1's
  // EcuCommand/EcuStatus (no middle ECU node on this track -- see header
  // comment).
  node_ = std::make_shared<rclcpp::Node>("chrono_supervisory_fmu_hw_interface_node");
  vehicle_command_pub_ =
    node_->create_publisher<VehicleCommand>("vehicle_command", rclcpp::SystemDefaultsQoS());
  vehicle_status_sub_ = node_->create_subscription<VehicleStatus>(
    "vehicle_status", rclcpp::SystemDefaultsQoS(), [this](const VehicleStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(vehicle_status_mutex_);
      latest_vehicle_status_ = *msg;
    });
  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ChronoSupervisoryFmuSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 2);
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, joint.interface, &joint.state);
    if (joint.interface == hardware_interface::HW_IF_VELOCITY) {
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.dead_reckoned_position);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ChronoSupervisoryFmuSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size());
  for (auto & joint : joints_) {
    if (!joint.has_command) {
      continue;
    }
    interfaces.emplace_back(joint.name, joint.interface, &joint.command);
  }
  return interfaces;
}

hardware_interface::return_type ChronoSupervisoryFmuSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  VehicleStatus status;
  {
    std::lock_guard<std::mutex> lock(vehicle_status_mutex_);
    status = latest_vehicle_status_;
  }
  // Same no-slip approximation as ChronoFmuSystemInterface::read() -- no
  // per-wheel omega output exists, speed_mps/kWheelRadius stands in for
  // every velocity-interface joint's state.
  const double wheel_omega = status.speed_mps / kWheelRadius;

  for (auto & joint : joints_) {
    if (joint.interface == hardware_interface::HW_IF_POSITION) {
      const bool is_left = joint.name.find("left") != std::string::npos;
      joint.state = (is_left ? status.steer_fl_deg : status.steer_fr_deg) * kRadPerDeg;
    } else {
      joint.state = wheel_omega;
      joint.dead_reckoned_position += joint.state * period.seconds();
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ChronoSupervisoryFmuSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Steering: independent FL/FR passthrough, no averaging -- same as
  // ChronoFmuSystemInterface's 4c.
  double steer_fl_deg = 0.0;
  double steer_fr_deg = 0.0;
  for (const auto & joint : joints_) {
    if (joint.interface != hardware_interface::HW_IF_POSITION) {
      continue;
    }
    const bool is_left = joint.name.find("left") != std::string::npos;
    (is_left ? steer_fl_deg : steer_fr_deg) = joint.command * kDegPerRad;
  }

  // Traction: same P-loop + deadband + clamp as ChronoFmuSystemInterface's
  // write() / chrono_split_ecu's tick() -- ported verbatim, not re-derived
  // (see either of those files' comments for the full tuning history).
  double vel_cmd_sum = 0.0;
  int vel_cmd_count = 0;
  double vel_measured = 0.0;
  for (const auto & joint : joints_) {
    if (joint.interface == hardware_interface::HW_IF_VELOCITY && joint.has_command) {
      vel_cmd_sum += joint.command;
      ++vel_cmd_count;
      vel_measured = joint.state;
    }
  }
  const double vel_cmd = (vel_cmd_count > 0 ? vel_cmd_sum / vel_cmd_count : 0.0);
  const double vel_error = vel_cmd - vel_measured;
  double total_drive_torque = 0.0;
  if (std::abs(vel_error) >= kVelocityDeadband) {
    total_drive_torque = kVelocityKp * vel_error;
  }
  if (total_drive_torque > kMaxDriveTorqueRear) {
    total_drive_torque = kMaxDriveTorqueRear;
  } else if (total_drive_torque < -kMaxDriveTorqueRear) {
    total_drive_torque = -kMaxDriveTorqueRear;
  }
  // Divide (not duplicate) across driven axles -- same reasoning as
  // ChronoFmuSystemInterface's write() / chrono_split_ecu's tick(). Only
  // affects THIS plugin's own math; the FMU's own six_wheel structural
  // flag is configured separately on PC2 (see header comment).
  const double num_driven_axles = six_wheel_ ? 3.0 : 2.0;
  const double drive_torque = total_drive_torque / num_driven_axles;

  VehicleCommand vc;
  vc.steer_fl_deg = steer_fl_deg;
  vc.steer_fr_deg = steer_fr_deg;
  vc.drive_torque_front_nm = drive_torque;
  vc.drive_torque_mid_nm = six_wheel_ ? drive_torque : 0.0;
  vc.drive_torque_rear_nm = drive_torque;
  vehicle_command_pub_->publish(vc);

  return hardware_interface::return_type::OK;
}

ChronoSupervisoryFmuSystemInterface::~ChronoSupervisoryFmuSystemInterface()
{
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

}  // namespace chrono_supervisory_fmu_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  chrono_supervisory_fmu_hw_interface::ChronoSupervisoryFmuSystemInterface, hardware_interface::SystemInterface)
