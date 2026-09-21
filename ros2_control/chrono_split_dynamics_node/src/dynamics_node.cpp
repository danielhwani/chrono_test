// Scaffolding only (see chrono_split_ecu/src/ecu_node.cpp's header
// comment for the overall split-architecture picture and why logic is
// deliberately deferred to a later step). This is the FMU-wrapped
// dynamics node: subscribes VehicleCommand (from the ECU node, Bridge 2),
// publishes VehicleStatus.
//
// TODO, deliberately not implemented yet:
//   - fmu_client_open() on a configurable fmu_dir (native_vehicle_fmu),
//     mirroring exactly what ChronoFmuSystemInterface::on_init() already
//     does in chrono_ros2_control/src/chrono_fmu_system_interface.cpp --
//     that file is untouched/still working, this is where the same
//     fmu_client call pattern moves to for the split architecture (as a
//     plain node instead of a hardware_interface plugin)
//   - resolving VRs (steer_fl/fr_deg_in, drive_torque_front/mid/rear,
//     steer_FL/FR_deg, speed_mps) via fmu_client_find_vr()
//   - each tick: fmu_client_set_real() from the latest VehicleCommand,
//     fmu_client_do_step(), fmu_client_get_real() into VehicleStatus
//   - fmu_client_close() on shutdown
//
// Right now this just proves the pub/sub wiring compiles and runs: ticks
// on a timer and publishes a constant VehicleStatus, ignoring whatever
// VehicleCommand arrives.
#include <rclcpp/rclcpp.hpp>

#include "chrono_split_msgs/msg/vehicle_command.hpp"
#include "chrono_split_msgs/msg/vehicle_status.hpp"

using chrono_split_msgs::msg::VehicleCommand;
using chrono_split_msgs::msg::VehicleStatus;

class ChronoSplitDynamicsNode : public rclcpp::Node
{
public:
  ChronoSplitDynamicsNode() : Node("chrono_split_dynamics_node")
  {
    vehicle_command_sub_ = create_subscription<VehicleCommand>(
      "vehicle_command", rclcpp::SystemDefaultsQoS(),
      [this](const VehicleCommand::SharedPtr msg) { latest_vehicle_command_ = *msg; });
    vehicle_status_pub_ =
      create_publisher<VehicleStatus>("vehicle_status", rclcpp::SystemDefaultsQoS());

    // 500 Hz -- matches native_vehicle_fmu/modelDescription.xml's
    // DefaultExperiment stepSize (0.002s), same as chrono_split_ecu.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&ChronoSplitDynamicsNode::tick, this));
  }

private:
  void tick()
  {
    // TODO: replace with real fmu_client_set_real/do_step/get_real calls
    // (see file header). Placeholder constant output only, to prove the
    // pub/sub wiring itself works end to end.
    (void)latest_vehicle_command_;
    VehicleStatus vs;
    vs.steer_fl_deg = 0.0;
    vs.steer_fr_deg = 0.0;
    vs.speed_mps = 0.0;
    vehicle_status_pub_->publish(vs);
  }

  rclcpp::Subscription<VehicleCommand>::SharedPtr vehicle_command_sub_;
  rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleCommand latest_vehicle_command_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChronoSplitDynamicsNode>());
  rclcpp::shutdown();
  return 0;
}
