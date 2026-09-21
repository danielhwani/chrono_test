// New SystemInterface for the split ros2_control / ECU / FMU-dynamics
// architecture -- replaces the ROLE ChronoFmuSystemInterface plays in the
// original single-process design (chrono_ros2_control/, left untouched
// and still fully working standalone), but this class knows NOTHING about
// the FMU. Its only job is Bridge 1: publish EcuCommand from the exported
// command_interfaces, subscribe EcuStatus into the exported
// state_interfaces (chrono_split_msgs, see ../../chrono_split_msgs/).
// The velocity->torque control loop and FMU access live downstream, in
// chrono_split_ecu and chrono_split_dynamics_node.
//
// Uses the exact same joint parsing / JointIO pattern as
// ChronoFmuSystemInterface (same URDF works unchanged -- Bridge 1's units
// match hardware_interface's own position/velocity conventions exactly).
// Unlike that class, this one genuinely has no logic left to defer: field
// mapping is this class's entire job (per the message design), so it's
// implemented for real here, not stubbed.
//
// hardware_interface::SystemInterface has no built-in ROS node in Humble
// (confirmed against the real installed header -- no get_node()/get_logger()
// on the base class, only in newer ros2_control releases), so this class
// creates its own internal rclcpp::Node and spins it on a background
// thread to process the EcuStatus subscription callback asynchronously;
// read()/write() (called from controller_manager's own RT thread) only
// ever touch the latched latest_ecu_status_ under ecu_status_mutex_.
#ifndef CHRONO_SPLIT_ECU_BRIDGE_HW_INTERFACE__CHRONO_ECU_BRIDGE_SYSTEM_INTERFACE_HPP_
#define CHRONO_SPLIT_ECU_BRIDGE_HW_INTERFACE__CHRONO_ECU_BRIDGE_SYSTEM_INTERFACE_HPP_

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "chrono_split_msgs/msg/ecu_command.hpp"
#include "chrono_split_msgs/msg/ecu_status.hpp"

namespace chrono_split_ecu_bridge_hw_interface
{

class ChronoEcuBridgeSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ~ChronoEcuBridgeSystemInterface() override;

private:
  struct JointIO
  {
    std::string name;
    std::string interface;  // hardware_interface::HW_IF_POSITION or HW_IF_VELOCITY
    double command = 0.0;
    double state = 0.0;
  };
  std::vector<JointIO> joints_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  rclcpp::Publisher<chrono_split_msgs::msg::EcuCommand>::SharedPtr ecu_command_pub_;
  rclcpp::Subscription<chrono_split_msgs::msg::EcuStatus>::SharedPtr ecu_status_sub_;

  std::mutex ecu_status_mutex_;
  chrono_split_msgs::msg::EcuStatus latest_ecu_status_;
};

}  // namespace chrono_split_ecu_bridge_hw_interface

#endif  // CHRONO_SPLIT_ECU_BRIDGE_HW_INTERFACE__CHRONO_ECU_BRIDGE_SYSTEM_INTERFACE_HPP_
