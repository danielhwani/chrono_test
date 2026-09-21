// Scaffolding only (per the user's explicit request: structure/wiring
// first, control logic later as its own step -- same "verify wiring, then
// fill logic" pattern this whole project has followed). This is the
// "virtual ECU" node in the split architecture:
//
//   ros2_control <-- EcuStatus/EcuCommand --> [this node] <-- VehicleStatus/VehicleCommand --> FMU dynamics node
//
// Bridge 1 (EcuCommand/EcuStatus, chrono_vehicle_msgs) uses
// hardware_interface-style units (rad, rad/s). Bridge 2
// (VehicleCommand/VehicleStatus) uses the FMU's native units (deg, N*m).
// This node is where that conversion belongs -- NOT in the new
// SystemInterface (chrono_ecu_bridge_hw_interface), which is meant to stay
// a thin translation layer with zero FMU-specific knowledge, and NOT in
// the FMU dynamics node, which should stay a thin fmu_client wrapper with
// zero control-loop knowledge. Concretely, still TODO here (deliberately
// left as a stub, not implemented yet):
//   - the 4b velocity->torque P loop + deadband (currently lives in
//     chrono_ros2_control's chrono_fmu_system_interface.cpp -- that file
//     is untouched/still working standalone; this is where that same
//     logic will move to for the split architecture)
//   - fan-out of one traction torque value to front/mid/rear
//     drive_torque_*_nm (mid unused until six_wheel is wired up)
//   - the rad<->deg unit conversions for steering
//
// Right now this just proves the pub/sub wiring compiles and runs: it
// echoes the latest EcuCommand straight through to VehicleCommand
// unconverted (wrong units, correctness not the point yet), and the
// latest VehicleStatus straight through to EcuStatus unconverted.
#include <rclcpp/rclcpp.hpp>

#include "chrono_vehicle_msgs/msg/ecu_command.hpp"
#include "chrono_vehicle_msgs/msg/ecu_status.hpp"
#include "chrono_vehicle_msgs/msg/vehicle_command.hpp"
#include "chrono_vehicle_msgs/msg/vehicle_status.hpp"

using chrono_vehicle_msgs::msg::EcuCommand;
using chrono_vehicle_msgs::msg::EcuStatus;
using chrono_vehicle_msgs::msg::VehicleCommand;
using chrono_vehicle_msgs::msg::VehicleStatus;

class ChronoVehicleEcuNode : public rclcpp::Node
{
public:
  ChronoVehicleEcuNode() : Node("chrono_vehicle_ecu")
  {
    ecu_command_sub_ = create_subscription<EcuCommand>(
      "ecu_command", rclcpp::SystemDefaultsQoS(),
      [this](const EcuCommand::SharedPtr msg) { latest_ecu_command_ = *msg; });
    vehicle_status_sub_ = create_subscription<VehicleStatus>(
      "vehicle_status", rclcpp::SystemDefaultsQoS(),
      [this](const VehicleStatus::SharedPtr msg) { latest_vehicle_status_ = *msg; });

    ecu_status_pub_ = create_publisher<EcuStatus>("ecu_status", rclcpp::SystemDefaultsQoS());
    vehicle_command_pub_ =
      create_publisher<VehicleCommand>("vehicle_command", rclcpp::SystemDefaultsQoS());

    // 500 Hz, matching native_vehicle_fmu's own step size (see
    // ChronoFmuSystemInterface's kStepSize) -- a plain wall timer here has
    // different jitter characteristics than controller_manager's dedicated
    // RT thread, worth measuring once real logic lands in this callback.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&ChronoVehicleEcuNode::tick, this));
  }

private:
  void tick()
  {
    // TODO: replace with the real P-loop/deadband/unit-conversion logic
    // (see the file header comment). Placeholder pass-through only, to
    // prove the pub/sub wiring itself works end to end.
    VehicleCommand vc;
    vc.steer_fl_deg = latest_ecu_command_.steer_fl_rad;
    vc.steer_fr_deg = latest_ecu_command_.steer_fr_rad;
    vc.drive_torque_front_nm = 0.0;
    vc.drive_torque_mid_nm = 0.0;
    vc.drive_torque_rear_nm = 0.0;
    vehicle_command_pub_->publish(vc);

    EcuStatus es;
    es.steer_fl_rad = latest_vehicle_status_.steer_fl_deg;
    es.steer_fr_rad = latest_vehicle_status_.steer_fr_deg;
    es.traction_vel_rad_s = latest_vehicle_status_.speed_mps;
    ecu_status_pub_->publish(es);
  }

  rclcpp::Subscription<EcuCommand>::SharedPtr ecu_command_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<EcuStatus>::SharedPtr ecu_status_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  EcuCommand latest_ecu_command_;
  VehicleStatus latest_vehicle_status_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChronoVehicleEcuNode>());
  rclcpp::shutdown();
  return 0;
}
