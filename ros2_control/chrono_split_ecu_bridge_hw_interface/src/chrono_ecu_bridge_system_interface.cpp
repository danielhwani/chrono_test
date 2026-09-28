#include "chrono_split_ecu_bridge_hw_interface/chrono_ecu_bridge_system_interface.hpp"

namespace chrono_split_ecu_bridge_hw_interface
{

using chrono_split_msgs::msg::EcuCommand;
using chrono_split_msgs::msg::EcuStatus;

hardware_interface::CallbackReturn ChronoEcuBridgeSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Same joint parsing as ChronoFmuSystemInterface -- Bridge 1's units
  // (rad, rad/s) match hardware_interface's own position/velocity
  // conventions exactly, so the existing chrono_vehicle.urdf works
  // unchanged here too.
  joints_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    auto logger = rclcpp::get_logger("ChronoEcuBridgeSystemInterface");
    if (joint.command_interfaces.size() > 1) {
      RCLCPP_ERROR(
        logger, "joint '%s' must declare at most one command_interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const bool has_command = !joint.command_interfaces.empty();
    // Primary interface: from the command_interface if the joint has one,
    // otherwise it must be velocity (state-only joints are only ever wheel
    // joints -- see has_command's comment). Same logic as
    // ChronoFmuSystemInterface's identical block.
    const std::string iface =
      has_command ? joint.command_interfaces[0].name : hardware_interface::HW_IF_VELOCITY;
    if (iface != hardware_interface::HW_IF_POSITION && iface != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        logger, "joint '%s' has unsupported interface '%s' (only position/velocity wired so far)",
        joint.name.c_str(), iface.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Steering (position) joints track exactly one real state (position).
    // Wheel (velocity) joints must declare EXACTLY 2 state_interfaces
    // (velocity, position) -- export_state_interfaces() unconditionally
    // exports both for every velocity-interface joint (the "position" one
    // is a dead-reckoned value purely for RViz, see
    // JointIO::dead_reckoned_position), and resource_manager requires the
    // declared count to match what's actually exported.
    if (iface == hardware_interface::HW_IF_POSITION) {
      if (
        joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_ERROR(
          logger, "joint '%s' must declare exactly one position state_interface",
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
          logger,
          "joint '%s' must declare exactly velocity + position state_interfaces (position is "
          "dead-reckoned, for RViz)",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    joints_.push_back(JointIO{joint.name, iface, 0.0, 0.0, has_command});
  }

  // hardware_interface::SystemInterface has no built-in node in Humble
  // (see header comment) -- create our own and spin it on a background
  // thread so the EcuStatus subscription callback runs independently of
  // controller_manager's own read()/write() calling thread.
  node_ = std::make_shared<rclcpp::Node>("chrono_split_ecu_bridge_hw_interface_node");
  ecu_command_pub_ = node_->create_publisher<EcuCommand>("ecu_command", rclcpp::SystemDefaultsQoS());
  ecu_status_sub_ = node_->create_subscription<EcuStatus>(
    "ecu_status", rclcpp::SystemDefaultsQoS(), [this](const EcuStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(ecu_status_mutex_);
      latest_ecu_status_ = *msg;
    });
  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ChronoEcuBridgeSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 2);
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, joint.interface, &joint.state);
    if (joint.interface == hardware_interface::HW_IF_VELOCITY) {
      // Dead-reckoned position, purely for RViz -- see JointIO's comment.
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.dead_reckoned_position);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ChronoEcuBridgeSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size());
  for (auto & joint : joints_) {
    if (!joint.has_command) {
      continue;  // 4WD front wheel spin joints -- state-only, see JointIO
    }
    interfaces.emplace_back(joint.name, joint.interface, &joint.command);
  }
  return interfaces;
}

hardware_interface::return_type ChronoEcuBridgeSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  EcuStatus status;
  {
    std::lock_guard<std::mutex> lock(ecu_status_mutex_);
    status = latest_ecu_status_;
  }
  for (auto & joint : joints_) {
    if (joint.interface == hardware_interface::HW_IF_POSITION) {
      const bool is_left = joint.name.find("left") != std::string::npos;
      joint.state = is_left ? status.steer_fl_rad : status.steer_fr_rad;
    } else {
      joint.state = status.traction_vel_rad_s;
      // Dead-reckoned, visualization-only -- see JointIO's comment.
      joint.dead_reckoned_position += joint.state * period.seconds();
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ChronoEcuBridgeSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  EcuCommand cmd;
  double vel_sum = 0.0;
  int vel_count = 0;
  for (const auto & joint : joints_) {
    if (joint.interface == hardware_interface::HW_IF_POSITION) {
      const bool is_left = joint.name.find("left") != std::string::npos;
      (is_left ? cmd.steer_fl_rad : cmd.steer_fr_rad) = joint.command;
    } else if (joint.has_command) {
      // has_command required: the 4WD front wheel spin joints also report
      // HW_IF_VELOCITY now but have no real command to average in (their
      // .command is a meaningless default 0.0).
      vel_sum += joint.command;
      ++vel_count;
    }
  }
  // Same averaging precedent as ChronoFmuSystemInterface's 4b/4c: multiple
  // wheels commanded by one axle-level reference collapse into one value
  // here too -- ackermann_steering_controller only ever sends one traction
  // reference in the first place.
  cmd.traction_vel_rad_s = (vel_count > 0 ? vel_sum / vel_count : 0.0);
  ecu_command_pub_->publish(cmd);
  return hardware_interface::return_type::OK;
}

ChronoEcuBridgeSystemInterface::~ChronoEcuBridgeSystemInterface()
{
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

}  // namespace chrono_split_ecu_bridge_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  chrono_split_ecu_bridge_hw_interface::ChronoEcuBridgeSystemInterface, hardware_interface::SystemInterface)
