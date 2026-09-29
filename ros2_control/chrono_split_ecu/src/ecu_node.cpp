// "Virtual ECU" node in the split architecture:
//
//   ros2_control <-- EcuStatus/EcuCommand --> [this node] <-- VehicleStatus/VehicleCommand --> FMU dynamics node
//
// Bridge 1 (EcuCommand/EcuStatus, chrono_split_msgs) uses
// hardware_interface-style units (rad, rad/s). Bridge 2
// (VehicleCommand/VehicleStatus) uses the FMU's native units (deg, N*m).
// This node owns that conversion -- NOT chrono_split_ecu_bridge_hw_interface
// (a thin translation layer with zero FMU-specific knowledge) and NOT
// chrono_fmu_dynamics_node (a thin fmu_client wrapper with zero
// control-loop knowledge).
//
// Control logic ported verbatim from chrono_ros2_control's
// chrono_fmu_system_interface.cpp (steps 4b/4c of the in-process version's
// prep plan) -- that file is untouched, this is the same logic relocated
// to run in its own process instead of inside a hardware_interface plugin:
//   - steering: straight rad<->deg passthrough, no averaging (EcuCommand
//     already carries independent FL/FR angles -- 4c's design)
//   - traction: plain P controller (kVelocityKp) with a deadband
//     (kVelocityDeadband) against natural suspension noise near rest (4b's
//     step-7 finding -- a runaway was found and fixed with the deadband;
//     same constants reused here rather than re-derived) and a torque
//     clamp (kMaxDriveTorqueRear)
//   - 4WD: the P-loop's total torque is now split across both axles
//     (matching chrono_ros2_control's own 4WD upgrade, same session) --
//     not a second, independent front loop, since
//     ackermann_steering_controller only ever provides the one traction
//     reference this is computed from. Division, not duplication: a first
//     attempt sent the FULL computed torque to both axles unchanged, which
//     doubles the effective torque-per-unit-error a 2-axle vehicle
//     experiences vs. the 1-axle case kVelocityKp/kVelocityDeadband were
//     tuned against (silently doubling the loop's effective gain) --
//     live-tested and confirmed to reproduce the exact step-7
//     noise-resonance runaway from a standstill, identically on both the
//     in-process and split versions (not a split-architecture-specific
//     bug). A real 4WD driveline splits a torque demand across axles
//     (transfer case), it doesn't duplicate it.
//   - six_wheel: an optional ROS2 node parameter (default false, launch
//     argument -- see the split launch file), matching
//     chrono_ros2_control's URDF <param name="six_wheel"> in spirit (this
//     node has no URDF hardware_parameters to read since it's a plain ROS2
//     node, not a hardware_interface component). When true, the SAME
//     computed torque also goes to drive_torque_mid_nm, and the total is
//     divided by 3 axles instead of 2 -- same divide-not-duplicate
//     reasoning as 4WD above.
//   - vel_measured is low-pass filtered (kVelFilterAlpha) before entering
//     the P computation -- ported from ChronoFmuSystemInterface's
//     identical fix, added after a 6x6 sharp-turn closed-loop oscillation
//     investigation found gain-lowering alone shrank but never eliminated
//     it, while filtering let it genuinely damp out (see README).
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "chrono_split_msgs/msg/ecu_command.hpp"
#include "chrono_split_msgs/msg/ecu_status.hpp"
#include "chrono_split_msgs/msg/vehicle_command.hpp"
#include "chrono_split_msgs/msg/vehicle_status.hpp"

using chrono_split_msgs::msg::EcuCommand;
using chrono_split_msgs::msg::EcuStatus;
using chrono_split_msgs::msg::VehicleCommand;
using chrono_split_msgs::msg::VehicleStatus;

namespace
{
constexpr double kDegPerRad = 180.0 / M_PI;
constexpr double kRadPerDeg = M_PI / 180.0;
// Matches vehicle_native.cpp's WHEEL_RADIUS -- see
// chrono_fmu_system_interface.hpp's identical constant/comment.
constexpr double kWheelRadius = 0.32;
// 4b's P gain and torque clamp, unchanged from chrono_fmu_system_interface.hpp.
constexpr double kVelocityKp = 80.0;
constexpr double kMaxDriveTorqueRear = 800.0;
// Step 7's deadband fix, unchanged from chrono_fmu_system_interface.hpp.
constexpr double kVelocityDeadband = 0.1;
// Low-pass filter on vel_measured before it enters the P computation --
// ported from ChronoFmuSystemInterface's identical fix (6x6 sharp-turn
// closed-loop oscillation investigation, see README). Unfiltered
// vel_measured is still what gets reported back as EcuStatus's
// traction_vel_rad_s (see tick()) -- the filter is purely an internal
// control-loop detail, not a change to what's externally observable.
constexpr double kVelFilterAlpha = 0.01;
}  // namespace

class ChronoSplitEcuNode : public rclcpp::Node
{
public:
  ChronoSplitEcuNode() : Node("chrono_split_ecu")
  {
    six_wheel_ = declare_parameter<bool>("six_wheel", false);

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
    // chrono_fmu_system_interface.hpp's kStepSize) -- a plain wall timer
    // here has different jitter characteristics than controller_manager's
    // dedicated RT thread, worth measuring once this is cross-checked
    // against the in-process version.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&ChronoSplitEcuNode::tick, this));
  }

private:
  void tick()
  {
    // Steering: straight passthrough with unit conversion, no averaging --
    // EcuCommand already carries independent FL/FR angles.
    const double steer_fl_deg = latest_ecu_command_.steer_fl_rad * kDegPerRad;
    const double steer_fr_deg = latest_ecu_command_.steer_fr_rad * kDegPerRad;

    // Traction: same no-slip approximation as chrono_fmu_system_interface's
    // read() (the FMU has no true per-wheel omega output), then the same
    // P loop + deadband as its write().
    const double vel_measured = latest_vehicle_status_.speed_mps / kWheelRadius;
    vel_measured_filtered_ += kVelFilterAlpha * (vel_measured - vel_measured_filtered_);
    const double vel_error = latest_ecu_command_.traction_vel_rad_s - vel_measured_filtered_;
    double total_drive_torque = 0.0;
    if (std::abs(vel_error) >= kVelocityDeadband) {
      total_drive_torque = kVelocityKp * vel_error;
    }
    if (total_drive_torque > kMaxDriveTorqueRear) {
      total_drive_torque = kMaxDriveTorqueRear;
    } else if (total_drive_torque < -kMaxDriveTorqueRear) {
      total_drive_torque = -kMaxDriveTorqueRear;
    }
    // 4WD/6x6: divided (not duplicated) across driven axles -- see file
    // header. num_driven_axles matches chrono_fmu_system_interface's own
    // six_wheel_ ? 3.0 : 2.0.
    const double num_driven_axles = six_wheel_ ? 3.0 : 2.0;
    const double drive_torque = total_drive_torque / num_driven_axles;

    VehicleCommand vc;
    vc.steer_fl_deg = steer_fl_deg;
    vc.steer_fr_deg = steer_fr_deg;
    vc.drive_torque_front_nm = drive_torque;
    vc.drive_torque_mid_nm = six_wheel_ ? drive_torque : 0.0;
    vc.drive_torque_rear_nm = drive_torque;
    vehicle_command_pub_->publish(vc);

    EcuStatus es;
    es.steer_fl_rad = latest_vehicle_status_.steer_fl_deg * kRadPerDeg;
    es.steer_fr_rad = latest_vehicle_status_.steer_fr_deg * kRadPerDeg;
    es.traction_vel_rad_s = vel_measured;
    ecu_status_pub_->publish(es);
  }

  rclcpp::Subscription<EcuCommand>::SharedPtr ecu_command_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Publisher<EcuStatus>::SharedPtr ecu_status_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  EcuCommand latest_ecu_command_;
  VehicleStatus latest_vehicle_status_;
  bool six_wheel_ = false;
  double vel_measured_filtered_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChronoSplitEcuNode>());
  rclcpp::shutdown();
  return 0;
}
