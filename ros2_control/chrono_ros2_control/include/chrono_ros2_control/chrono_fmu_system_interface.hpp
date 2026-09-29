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
//
// 4WD (rear-only -> 4WD upgrade, same session as the split-architecture
// track) is now also done: on_init() sets four_wheel_drive=1.0 once, and
// write() fans the SAME P-loop torque value (drive_torque_rear's own
// computed value, not a second independent loop) into drive_torque_front
// too -- exactly the "single traction reference fanned out internally"
// design decided long before this plugin existed, since
// ackermann_steering_controller never gave a separate front-traction
// command in the first place. apply_differential() still separately owns
// L/R split on each axle; this plugin still only ever talks in axle-level
// quantities, now two axles instead of one. IMPORTANT: the torque is
// DIVIDED across driven axles, not duplicated onto each -- a first attempt
// duplicated it, which doubles the effective torque-per-unit-error a
// 2-axle vehicle experiences vs. the 1-axle case kVelocityKp/kVelocityDeadband
// were tuned against, and reproduced (far worse) the exact step-7
// noise-resonance runaway from a standstill. A real 4WD driveline's
// transfer case splits a torque demand across axles, it doesn't clone it.
//
// 6x6 (same session): an OPTIONAL <param name="six_wheel">true</param>
// (absent/false by default -- every existing 4-wheel/2-axle behavior stays
// bit-exact) turns on native_vehicle_fmu's six_wheel structural flag and
// makes write() fan the P-loop's torque across 3 axles (divide by
// num_driven_axles_, not a hardcoded /2.0) instead of 2. See
// chrono_vehicle_6x6.urdf for the sibling URDF this is meant to pair with.
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
    // false for a state-only joint (no <command_interface> in the URDF --
    // added for the 4WD front wheel spin joints, which report velocity
    // but must never be written to: ackermann_steering_controller has no
    // front-traction command path, and 4WD's torque is already driven
    // from the rear-axle command. export_command_interfaces() skips these
    // entirely, and write()'s command-averaging loop must not include
    // them (their .command stays a meaningless default 0.0 forever).
    bool has_command = true;
    // Dead-reckoned position for velocity-only (wheel) joints, purely for
    // RViz visualization -- integrated from the state velocity each read()
    // (position += velocity * period.seconds()), exported as an ADDITIONAL
    // state_interface alongside the real velocity one. Never fed back into
    // any control decision (ackermann_steering_controller never reads wheel
    // position). Added because robot_state_publisher needs a numeric
    // position for every non-fixed joint to compute its link's TF, even a
    // continuous one -- without this, wheel joints (velocity-state-only by
    // design, see has_command's comment) reported NaN position, and
    // robot_state_publisher couldn't place the wheel links at all ("No
    // transform from [wheel] to [base_link]" in RViz, found live when the
    // 6x6 build was first opened in RViz).
    double dead_reckoned_position = 0.0;
  };
  std::vector<JointIO> joints_;

  FmuClient * fmu_ = nullptr;

  // Value references, resolved once in on_init() via fmu_client_find_vr()
  // rather than hardcoded, so a future modelDescription.xml VR renumbering
  // doesn't silently break this file.
  FmuValueReference vr_drive_torque_rear_{};
  FmuValueReference vr_drive_torque_front_{};       // input: 4WD fan-out target (see write())
  FmuValueReference vr_four_wheel_drive_{};         // input: set to 1.0 once, in on_init()
  FmuValueReference vr_drive_torque_mid_{};         // input: 6x6 fan-out target (see write())
  FmuValueReference vr_six_wheel_{};                // input: set to 1.0 once IFF six_wheel_ (see on_init())
  FmuValueReference vr_steer_fl_deg_{};   // output: steer_FL_deg (actual, read back)
  FmuValueReference vr_steer_fr_deg_{};   // output: steer_FR_deg (actual, read back)
  FmuValueReference vr_speed_mps_{};
  FmuValueReference vr_independent_front_steer_{};  // input: set to 1.0 once, in on_init()
  FmuValueReference vr_steer_fl_deg_in_{};          // input: FL commanded angle
  FmuValueReference vr_steer_fr_deg_in_{};          // input: FR commanded angle

  // Read once from the URDF's optional <param name="six_wheel">, in
  // on_init() -- absent/anything-but-"true" means false, preserving every
  // existing 4-wheel/2-axle behavior exactly (see header comment).
  bool six_wheel_ = false;

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
  // Step 7 finding: at rest (vel_cmd=0), the vehicle's own suspension
  // settling produces small natural noise in vel_measured (~0.01-0.07
  // rad/s, observed via an A/B test with drive_torque_rear forced to 0).
  // A pure P term with no deadband reacts to that noise at full gain and
  // pumps energy into it instead of damping it, causing an exponential
  // runaway from a standstill -- reproduced and root-caused live against
  // a real controller_manager, not seen in the 4b bench test (which only
  // ever tested driving toward a large nonzero target, never holding
  // near-zero). Set safely above the observed noise floor; real velocity
  // commands (normally several rad/s) are far above this and unaffected.
  static constexpr double kVelocityDeadband = 0.1;

  // Low-pass-filtered vel_measured -- added after the 6x6 sharp-turn
  // closed-loop oscillation investigation (see README): gain-lowering
  // alone (80->20->5) shrank the oscillation's amplitude proportionally
  // but never eliminated the real-wheel sign-flipping itself, while a
  // filter on the feedback (at the ORIGINAL kVelocityKp=80, gain-lowering
  // was tried combined with the filter too and made things WORSE --
  // amplitude stopped decaying toward zero and just hovered near a fixed
  // nonzero level instead) let the same oscillation genuinely damp out
  // within ~8-10s of a step change instead of persisting indefinitely.
  // Unlike a D-term (rejected earlier for amplifying noise -- see the
  // step-7 deadband comment above), a low-pass filter suppresses
  // high-frequency noise in the feedback signal before it reaches the P
  // computation, rather than differentiating (and thus amplifying) it.
  // Simple exponential moving average:
  // filtered += kVelFilterAlpha * (raw - filtered) each write(). This
  // does change convergence speed slightly even for previously-bit-exact
  // scenarios (e.g. the 4-wheel/6x6 straight-line bench numbers) --
  // expected and accepted, not a bug; see README for the exact deltas.
  double vel_measured_filtered_ = 0.0;
  static constexpr double kVelFilterAlpha = 0.01;
};

}  // namespace chrono_ros2_control

#endif  // CHRONO_ROS2_CONTROL__CHRONO_FMU_SYSTEM_INTERFACE_HPP_
