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
// Two known gaps, deliberately NOT hidden behind a fake-looking
// implementation -- see the matching comments in the .cpp:
//   - 4b (not done yet): rear wheel command_interfaces are `velocity`, but
//     the FMU's drive_torque_rear input is a torque. No velocity->torque
//     control loop exists yet.
//   - 4c (not done yet): the URDF has two independent front steering
//     command_interfaces, but the FMU has one shared steer_deg input. The
//     two commands are averaged for now, not treated as the final design.
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
  FmuValueReference vr_steer_deg_{};
  FmuValueReference vr_drive_torque_rear_{};
  FmuValueReference vr_steer_fl_deg_{};
  FmuValueReference vr_steer_fr_deg_{};
  FmuValueReference vr_speed_mps_{};

  double sim_time_ = 0.0;
  // Matches native_vehicle_fmu/modelDescription.xml's DefaultExperiment
  // stepSize -- the FMU was only ever validated at this step size.
  static constexpr double kStepSize = 0.002;
  // Matches vehicle_native.cpp's WHEEL_RADIUS; used only to turn speed_mps
  // into an approximate no-slip wheel angular velocity for the rear wheel
  // state_interfaces, since the FMU has no per-wheel omega output (see the
  // read() comment).
  static constexpr double kWheelRadius = 0.32;
};

}  // namespace chrono_ros2_control

#endif  // CHRONO_ROS2_CONTROL__CHRONO_FMU_SYSTEM_INTERFACE_HPP_
