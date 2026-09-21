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
    if (joint.command_interfaces.size() != 1 || joint.state_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ChronoEcuBridgeSystemInterface"),
        "joint '%s' must declare exactly one command_interface and one state_interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const std::string & iface = joint.command_interfaces[0].name;
    if (iface != hardware_interface::HW_IF_POSITION && iface != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ChronoEcuBridgeSystemInterface"),
        "joint '%s' has unsupported interface '%s' (only position/velocity wired so far)",
        joint.name.c_str(), iface.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joints_.push_back(JointIO{joint.name, iface, 0.0, 0.0});
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
  interfaces.reserve(joints_.size());
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, joint.interface, &joint.state);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ChronoEcuBridgeSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size());
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, joint.interface, &joint.command);
  }
  return interfaces;
}

hardware_interface::return_type ChronoEcuBridgeSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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
    } else {
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
