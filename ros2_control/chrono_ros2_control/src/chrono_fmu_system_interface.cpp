#include "chrono_ros2_control/chrono_fmu_system_interface.hpp"

#include <cmath>

#include "rclcpp/logging.hpp"

namespace chrono_ros2_control
{

namespace
{
constexpr double kDegPerRad = 180.0 / M_PI;
constexpr double kRadPerDeg = M_PI / 180.0;

rclcpp::Logger logger() { return rclcpp::get_logger("ChronoFmuSystemInterface"); }

bool find_vr_or_fail(const FmuClient * fmu, const char * name, FmuValueReference * out)
{
  if (!fmu_client_find_vr(fmu, name, out)) {
    RCLCPP_ERROR(logger(), "FMU has no variable named '%s'", name);
    return false;
  }
  return true;
}
}  // namespace

hardware_interface::CallbackReturn ChronoFmuSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto it = info_.hardware_parameters.find("fmu_dir");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_ERROR(logger(), "hardware parameter 'fmu_dir' is required (see chrono_vehicle.urdf)");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string fmu_dir = it->second;

  // Optional -- absent (as in chrono_vehicle.urdf) means false, preserving
  // every existing 4-wheel/2-axle behavior exactly. "true" (as in
  // chrono_vehicle_6x6.urdf) turns on the mid axle.
  auto six_wheel_it = info_.hardware_parameters.find("six_wheel");
  six_wheel_ = (six_wheel_it != info_.hardware_parameters.end() && six_wheel_it->second == "true");

  // fmu_client_open_begin()/_finish() (not the single-call fmu_client_open())
  // because four_wheel_drive is a genuinely structural FMU parameter --
  // native_vehicle_fmu reads it exactly once, inside its own
  // fmi2ExitInitializationMode handler, to decide whether a front drive
  // motor gets built at all. Setting it after a single-call open() (which
  // used to run Enter/ExitInitializationMode back to back) is silently too
  // late -- discovered exactly that way: four_wheel_drive=1.0 set right
  // after the old fmu_client_open() returned produced bit-identical output
  // to four_wheel_drive=0.0. See fmu_client.h's updated header comment.
  fmu_ = fmu_client_open_begin(fmu_dir.c_str(), info_.name.c_str());
  if (!fmu_) {
    RCLCPP_ERROR(logger(), "fmu_client_open_begin('%s') failed", fmu_dir.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  bool ok = true;
  ok &= find_vr_or_fail(fmu_, "drive_torque_rear", &vr_drive_torque_rear_);
  ok &= find_vr_or_fail(fmu_, "drive_torque_front", &vr_drive_torque_front_);
  ok &= find_vr_or_fail(fmu_, "four_wheel_drive", &vr_four_wheel_drive_);
  ok &= find_vr_or_fail(fmu_, "drive_torque_mid", &vr_drive_torque_mid_);
  ok &= find_vr_or_fail(fmu_, "six_wheel", &vr_six_wheel_);
  ok &= find_vr_or_fail(fmu_, "steer_FL_deg", &vr_steer_fl_deg_);
  ok &= find_vr_or_fail(fmu_, "steer_FR_deg", &vr_steer_fr_deg_);
  ok &= find_vr_or_fail(fmu_, "speed_mps", &vr_speed_mps_);
  ok &= find_vr_or_fail(fmu_, "independent_front_steer", &vr_independent_front_steer_);
  ok &= find_vr_or_fail(fmu_, "steer_fl_deg_in", &vr_steer_fl_deg_in_);
  ok &= find_vr_or_fail(fmu_, "steer_fr_deg_in", &vr_steer_fr_deg_in_);
  if (!ok) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Flags set here, still in initialization mode -- four_wheel_drive and
  // six_wheel MUST be set before open_finish() (both structural, see
  // above); independent_front_steer doesn't strictly need to be (it's
  // checked every step(), not build-time), but setting it in the same
  // place is simpler than splitting into "before finish"/"after finish"
  // calls for no real benefit. six_wheel only sent as 1.0 when the URDF's
  // six_wheel param asked for it -- otherwise left at the FMU's own
  // default (0.0), matching four_wheel_drive's now-established pattern.
  FmuValueReference vr_flags[3] = {
    vr_independent_front_steer_, vr_four_wheel_drive_, vr_six_wheel_};
  double flag_values[3] = {1.0, 1.0, six_wheel_ ? 1.0 : 0.0};
  if (!fmu_client_set_real(fmu_, vr_flags, 3, flag_values)) {
    RCLCPP_ERROR(
      logger(), "fmu_client_set_real(independent_front_steer, four_wheel_drive, six_wheel) failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!fmu_client_open_finish(fmu_)) {
    RCLCPP_ERROR(logger(), "fmu_client_open_finish('%s') failed", fmu_dir.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() > 1) {
      RCLCPP_ERROR(
        logger(), "joint '%s' must declare at most one command_interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const bool has_command = !joint.command_interfaces.empty();
    // Primary interface: from the command_interface if the joint has one,
    // otherwise it must be velocity (state-only joints are only ever wheel
    // joints, which track velocity -- see has_command's comment).
    const std::string iface =
      has_command ? joint.command_interfaces[0].name : hardware_interface::HW_IF_VELOCITY;
    if (iface != hardware_interface::HW_IF_POSITION && iface != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        logger(), "joint '%s' has unsupported interface '%s' (only position/velocity wired so far)",
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
          logger(), "joint '%s' must declare exactly one position state_interface",
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
          logger(),
          "joint '%s' must declare exactly velocity + position state_interfaces (position is "
          "dead-reckoned, for RViz)",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    joints_.push_back(JointIO{joint.name, iface, 0.0, 0.0, has_command});
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ChronoFmuSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 2);
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, joint.interface, &joint.state);
    if (joint.interface == hardware_interface::HW_IF_VELOCITY) {
      // Dead-reckoned position, purely for RViz -- see JointIO's comment.
      // Only wheel (velocity-interface) joints need this; steering joints
      // already export a real, FMU-tracked position as their one state.
      interfaces.emplace_back(
        joint.name, hardware_interface::HW_IF_POSITION, &joint.dead_reckoned_position);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> ChronoFmuSystemInterface::export_command_interfaces()
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

hardware_interface::return_type ChronoFmuSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  FmuValueReference vrs[3] = {vr_steer_fl_deg_, vr_steer_fr_deg_, vr_speed_mps_};
  double values[3] = {0.0, 0.0, 0.0};
  if (!fmu_client_get_real(fmu_, vrs, 3, values)) {
    RCLCPP_ERROR(logger(), "fmu_client_get_real failed");
    return hardware_interface::return_type::ERROR;
  }
  const double steer_fl_deg = values[0];
  const double steer_fr_deg = values[1];
  const double speed_mps = values[2];
  // No per-wheel angular velocity output exists in the FMU yet (see
  // modelDescription.xml) -- this is a no-slip approximation
  // (speed_mps / wheel_radius), same for both rear wheels. Revisit if the
  // FMU ever exposes real per-wheel omega.
  const double wheel_omega = speed_mps / kWheelRadius;

  for (auto & joint : joints_) {
    if (joint.interface == hardware_interface::HW_IF_POSITION) {
      const bool is_left = joint.name.find("left") != std::string::npos;
      joint.state = (is_left ? steer_fl_deg : steer_fr_deg) * kRadPerDeg;
    } else {
      joint.state = wheel_omega;
      // Dead-reckoned, visualization-only -- see JointIO's comment.
      joint.dead_reckoned_position += joint.state * period.seconds();
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ChronoFmuSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 4c: send each front wheel's independently-commanded steering angle
  // straight through to the FMU's steer_fl_deg_in/steer_fr_deg_in (set to
  // take priority over steer_deg via independent_front_steer=1 in
  // on_init()) -- no averaging, the controller's own Ackermann correction
  // (different FL/FR angles) survives intact.
  double steer_fl_deg = 0.0;
  double steer_fr_deg = 0.0;
  for (const auto & joint : joints_) {
    if (joint.interface != hardware_interface::HW_IF_POSITION) {
      continue;
    }
    const bool is_left = joint.name.find("left") != std::string::npos;
    (is_left ? steer_fl_deg : steer_fr_deg) = joint.command * kDegPerRad;
  }

  // 4b: rear wheel command_interfaces are `velocity` (joint.command, rad/s);
  // drive_torque_rear is a torque input. Both rear wheels' commanded
  // velocities funnel into one axle-level target the same way the two
  // front steering commands funnel into one steer_deg above --
  // apply_differential() inside the FMU still owns the L/R split, this
  // plugin only ever talks in axle-level quantities. Measured velocity is
  // read from joints_ state (set by read() from the previous cycle's
  // speed_mps -- both rear wheel states are already the same no-slip
  // approximation, so either one works as the axle's measured velocity).
  // has_command additionally required here (not just interface==VELOCITY)
  // -- the 4WD front wheel spin joints also report HW_IF_VELOCITY state
  // now, but have no real command to average in (their .command is a
  // meaningless default 0.0 that would otherwise silently dilute the P
  // loop's target).
  double vel_cmd_sum = 0.0;
  int vel_cmd_count = 0;
  double vel_measured = 0.0;
  for (const auto & joint : joints_) {
    if (joint.interface == hardware_interface::HW_IF_VELOCITY && joint.has_command) {
      vel_cmd_sum += joint.command;
      ++vel_cmd_count;
      vel_measured = joint.state;
    }
  }
  const double vel_cmd = (vel_cmd_count > 0 ? vel_cmd_sum / vel_cmd_count : 0.0);
  // Filter vel_measured before computing error -- see kVelFilterAlpha's
  // header comment for why (6x6 sharp-turn closed-loop oscillation fix).
  vel_measured_filtered_ += kVelFilterAlpha * (vel_measured - vel_measured_filtered_);
  const double vel_error = vel_cmd - vel_measured_filtered_;
  // Step 7 finding, root-caused via an A/B test (forcing drive_torque_rear
  // to exactly 0 and watching vel_measured stay bounded at ~0.01-0.07 rad/s
  // -- normal suspension settling noise -- vs. the unmodified P loop
  // exponentially diverging from a standstill with vel_cmd=0, e.g.
  // 0.03->1.5->8.6->32+ rad/s over a few seconds despite drive_torque_rear
  // saturating at -800 the whole time): a pure P term with no deadband
  // reacts to that small natural noise with full gain, and ends up pumping
  // energy into it instead of damping it -- classic high-gain-on-noise
  // resonance, not a measurement bug (vel_measured's approximation itself
  // -- speed_mps / kWheelRadius -- was already known/documented; the loop
  // built on top of it was the actual problem). kVelocityDeadband is set
  // safely above the observed noise floor so real commands (normally
  // several rad/s) are unaffected, while near-zero noise now yields zero
  // torque instead of feeding back on itself.
  double total_drive_torque = 0.0;
  if (std::abs(vel_error) >= kVelocityDeadband) {
    total_drive_torque = kVelocityKp * vel_error;
  }
  if (total_drive_torque > kMaxDriveTorqueRear) {
    total_drive_torque = kMaxDriveTorqueRear;
  } else if (total_drive_torque < -kMaxDriveTorqueRear) {
    total_drive_torque = -kMaxDriveTorqueRear;
  }

  // 4WD bug found and fixed (same session): the FIRST version of this sent
  // the full total_drive_torque to BOTH axles unchanged, which doubles the
  // effective torque-per-unit-error a real 2-axle vehicle experiences
  // compared to the 1-axle case kVelocityKp/kVelocityDeadband were tuned
  // and validated against -- equivalent to silently doubling Kp. That
  // re-triggered (worse than before) the exact step-7 noise-resonance
  // runaway from a standstill, live-tested and confirmed identically on
  // BOTH the in-process and split versions (not an architecture-specific
  // bug). A real 4WD driveline splits a given torque demand across axles
  // (transfer case), it doesn't duplicate it -- dividing here instead of
  // duplicating keeps the total commanded tractive effort, and therefore
  // the closed loop's effective gain, identical to the already-validated
  // 2WD case regardless of how many axles are actually driven. 6x6 divides
  // by 3 instead of 2 for exactly the same reason -- num_driven_axles_
  // must track how many axles are actually driven, not a hardcoded /2.0.
  const double num_driven_axles = six_wheel_ ? 3.0 : 2.0;
  const double drive_torque = total_drive_torque / num_driven_axles;

  // apply_differential() still separately handles L/R split on each axle;
  // this plugin only ever talks in axle-level quantities. drive_torque_mid
  // only sent (nonzero) when six_wheel_ -- otherwise the FMU has no mid
  // axle at all and simply ignores whatever value sits in that input.
  FmuValueReference vrs[5] = {
    vr_steer_fl_deg_in_, vr_steer_fr_deg_in_, vr_drive_torque_rear_, vr_drive_torque_front_,
    vr_drive_torque_mid_};
  double values[5] = {
    steer_fl_deg, steer_fr_deg, drive_torque, drive_torque, six_wheel_ ? drive_torque : 0.0};
  if (!fmu_client_set_real(fmu_, vrs, 5, values)) {
    RCLCPP_ERROR(
      logger(), "fmu_client_set_real(steer_fl/fr_deg_in, drive_torque_rear/front/mid) failed");
    return hardware_interface::return_type::ERROR;
  }

  // TODO(step 7): this always advances the FMU by the fixed kStepSize
  // (0.002s), ignoring the real `period` controller_manager passes into
  // write(). Fine for this file's own local test harness (which drives the
  // loop at exactly 2ms itself), but if the real controller_manager's
  // actual cycle time ever drifts from 2ms, the FMU's simulated time and
  // wall-clock time will diverge. Revisit once step 7 runs this against a
  // real controller_manager and its real timing is known -- either use
  // `period` directly as the step size, or sub-step to keep FMU time
  // locked to wall-clock time regardless of controller_manager's rate.
  if (!fmu_client_do_step(fmu_, sim_time_, kStepSize)) {
    RCLCPP_ERROR(logger(), "fmu_client_do_step failed at t=%f", sim_time_);
    return hardware_interface::return_type::ERROR;
  }
  sim_time_ += kStepSize;

  return hardware_interface::return_type::OK;
}

ChronoFmuSystemInterface::~ChronoFmuSystemInterface()
{
  fmu_client_close(fmu_);
}

}  // namespace chrono_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  chrono_ros2_control::ChronoFmuSystemInterface, hardware_interface::SystemInterface)
