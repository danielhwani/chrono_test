// Step 4a of the ros2_control prep plan (see README.md / memory
// fmu-next-steps): basic wiring for ChronoFmuSystemInterface, a
// hardware_interface::SystemInterface that drives fmu/cpp/native_vehicle_fmu
// through fmu_client (fmu/cpp/native_fmu/driver/fmu_client.h -- the same
// library fmu_driver and fmu_client_cpp_check already use and validated
// bit-exact against). Scoped to exactly what chrono_vehicle.urdf (step 3)
// declares: 2 front steering joints (position) + 2 rear wheel joints
// (velocity), 4-wheel car only -- not six_wheel (deferred on purpose, see
// memory: finish the 4-wheel pipeline through step 7 first).
//
// 4c (front steering input width) is now done: the URDF's two independent
// front steering command_interfaces are sent straight through to the FMU's
// independent_front_steer=1 / steer_fl_deg_in / steer_fr_deg_in inputs
// (added to both FMUs specifically for this -- see README's 4c section),
// instead of being averaged into the old shared steer_deg input. That
// input still exists on the FMU (used by fmu_driver/other direct callers)
// but this plugin no longer touches it.
//
// 4b (rear wheel velocity->torque) is now done: a plain proportional
// controller (kVelocityKp, clamped to kMaxDriveTorqueRear) converts the
// rear axle's commanded angular velocity into drive_torque_rear. Gain and
// clamp were picked empirically via the check harness (see README), not
// derived from first principles -- revisit if step 7's real
// controller_manager run shows oscillation/instability. No integral or
// derivative term by design (the plan's own scope was "e.g. a P
// controller"). Deliberately architected the same way as steer_deg's
// averaging and the 4WD/6x6 drive_torque_front/mid/rear inputs: ONE
// axle-level value goes into the FMU, and apply_differential() inside the
// FMU still owns the L/R split -- this plugin never talks to individual
// wheels, only axles, matching that established pattern.
#ifndef CHRONO_ROS2_CONTROL__CHRONO_FMU_SYSTEM_INTERFACE_HPP_
#define CHRONO_ROS2_CONTROL__CHRONO_FMU_SYSTEM_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

extern "C" {
#include "fmu_client.h"
}

namespace chrono_ros2_control
{

class ChronoFmuSystemInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ~ChronoFmuSystemInterface() override;

private:
  // One entry per URDF joint, in info_.joints order. command/state are the
  // actual storage StateInterface/CommandInterface hold pointers into --
  // export_*_interfaces() must not be called more than once per joint
  // vector reallocation, so joints_ is sized once in on_init() and never
  // resized after.
  struct JointIO
  {
    std::string name;
    std::string interface;  // hardware_interface::HW_IF_POSITION or HW_IF_VELOCITY
    double command = 0.0;
    double state = 0.0;
  };
  std::vector<JointIO> joints_;

  FmuClient * fmu_ = nullptr;

  // Value references, resolved once in on_init() via fmu_client_find_vr()
  // rather than hardcoded, so a future modelDescription.xml VR renumbering
  // doesn't silently break this file.
  FmuValueReference vr_drive_torque_rear_{};
  FmuValueReference vr_steer_fl_deg_{};   // output: steer_FL_deg (actual, read back)
  FmuValueReference vr_steer_fr_deg_{};   // output: steer_FR_deg (actual, read back)
  FmuValueReference vr_speed_mps_{};
  FmuValueReference vr_independent_front_steer_{};  // input: set to 1.0 once, in on_init()
  FmuValueReference vr_steer_fl_deg_in_{};          // input: FL commanded angle
  FmuValueReference vr_steer_fr_deg_in_{};          // input: FR commanded angle

  double sim_time_ = 0.0;
  // Matches native_vehicle_fmu/modelDescription.xml's DefaultExperiment
  // stepSize -- the FMU was only ever validated at this step size.
  static constexpr double kStepSize = 0.002;
  // Matches vehicle_native.cpp's WHEEL_RADIUS; used only to turn speed_mps
  // into an approximate no-slip wheel angular velocity for the rear wheel
  // state_interfaces, since the FMU has no per-wheel omega output (see the
  // read() comment).
  static constexpr double kWheelRadius = 0.32;

  // 4b: rear axle velocity->torque P controller. N*m per (rad/s) of error;
  // empirically picked (see README's 4b section for the tuning run), not
  // derived analytically.
  static constexpr double kVelocityKp = 80.0;
  // Clamp on the commanded torque -- guards the FMU's fixed-iteration-count
  // NSC solver against a runaway P term (this project already saw the
  // solver's approximate solution get perturbed by a much smaller
  // constraint change once, see 4WD's zero-torque-front-motor finding in
  // memory). Roughly 3x the FMU's own default drive_torque_rear (260.0).
  static constexpr double kMaxDriveTorqueRear = 800.0;
};

}  // namespace chrono_ros2_control

#endif  // CHRONO_ROS2_CONTROL__CHRONO_FMU_SYSTEM_INTERFACE_HPP_
