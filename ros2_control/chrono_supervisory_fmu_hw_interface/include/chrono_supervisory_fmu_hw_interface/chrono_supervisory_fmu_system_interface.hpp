// 분산 버전 (distributed version): runs on the supervisory-controller
// machine, alongside ros2_control and whatever autonomy stack sits on
// top -- the "PC1" role in this track's 2-machine framing. The FMU
// dynamics run on a separate machine ("PC2") -- chrono_fmu_dynamics_node,
// reused completely unchanged (it was already a generic,
// control-logic-free FMU wrapper, exactly what that role needs). ONE
// bridge, not two: chrono_split_msgs VehicleCommand/VehicleStatus (the
// same Bridge 2 protocol/units the split-version's
// chrono_split_ecu<->chrono_fmu_dynamics_node link already uses -- deg,
// N*m, axle-level).
//
// Why no middle "virtual ECU" node here, unlike 스플릿 버전
// (chrono_split_ecu_bridge_hw_interface + chrono_split_ecu +
// chrono_fmu_dynamics_node, 2 bridges/3 processes): that design
// deliberately mirrors a real vehicle's CAN-connected ECU boundary, which
// only pays off once/if a real ECU could someday replace chrono_split_ecu.
// This track has no such hardware boundary to mirror -- only a network
// boundary between the supervisory controller and the physics -- so the
// velocity->torque control logic (P-loop, deadband, 4WD/6x6 axle fan-out)
// lives directly in THIS plugin, ported verbatim from
// ChronoFmuSystemInterface's write()/read(), instead of being delegated
// to a separate ECU process. Net effect: same joint parsing/URDF/control
// constants as chrono_ros2_control, same node-per-plugin/background-spin-thread
// mechanism as chrono_split_ecu_bridge_hw_interface
// (hardware_interface::SystemInterface has no built-in node in Humble --
// see that class's header comment for the full story), talking
// chrono_split_ecu's own units directly instead of Bridge 1's
// hardware_interface-native ones.
//
// six_wheel_ here is READ ONLY for this plugin's own axle-divisor math
// (num_driven_axles in write()) -- it does NOT set the FMU's structural
// six_wheel flag (this plugin has no FMU access at all). That flag is
// the dynamics machine's own chrono_fmu_dynamics_node's concern, set
// via ITS OWN six_wheel ROS2 node parameter when launched there -- the
// two sides must be configured to agree (both six_wheel or both not),
// same requirement as matching URDF/dynamics-node six_wheel settings
// already have on the split track.
#ifndef CHRONO_SUPERVISORY_FMU_HW_INTERFACE__CHRONO_SUPERVISORY_FMU_SYSTEM_INTERFACE_HPP_
#define CHRONO_SUPERVISORY_FMU_HW_INTERFACE__CHRONO_SUPERVISORY_FMU_SYSTEM_INTERFACE_HPP_

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "chrono_split_msgs/msg/vehicle_command.hpp"
#include "chrono_split_msgs/msg/vehicle_status.hpp"

namespace chrono_supervisory_fmu_hw_interface
{

class ChronoSupervisoryFmuSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ~ChronoSupervisoryFmuSystemInterface() override;

private:
  // Identical shape/reasoning to ChronoFmuSystemInterface::JointIO -- see
  // that header for the has_command/dead_reckoned_position comments in
  // full (front/mid wheel spin joints are state-only; position on
  // velocity-interface joints is a dead-reckoned, RViz-only value).
  struct JointIO
  {
    std::string name;
    std::string interface;
    double command = 0.0;
    double state = 0.0;
    bool has_command = true;
    double dead_reckoned_position = 0.0;
  };
  std::vector<JointIO> joints_;

  // Read once from the URDF's optional <param name="six_wheel"> in
  // on_init() -- axle-divisor math only, see header comment above.
  bool six_wheel_ = false;

  // hardware_interface::SystemInterface has no built-in node in Humble --
  // same pattern as ChronoEcuBridgeSystemInterface.
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  rclcpp::Publisher<chrono_split_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<chrono_split_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

  std::mutex vehicle_status_mutex_;
  chrono_split_msgs::msg::VehicleStatus latest_vehicle_status_;

  // Matches vehicle_native.cpp's WHEEL_RADIUS -- see
  // ChronoFmuSystemInterface's identical constant/comment.
  static constexpr double kWheelRadius = 0.32;
  // 4b/4WD/6x6 control constants, unchanged from ChronoFmuSystemInterface
  // (ported verbatim, not re-derived) -- see that header for the tuning
  // rationale/history.
  static constexpr double kVelocityKp = 80.0;
  static constexpr double kMaxDriveTorqueRear = 800.0;
  static constexpr double kVelocityDeadband = 0.1;
};

}  // namespace chrono_supervisory_fmu_hw_interface

#endif  // CHRONO_SUPERVISORY_FMU_HW_INTERFACE__CHRONO_SUPERVISORY_FMU_SYSTEM_INTERFACE_HPP_
