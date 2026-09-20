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

  fmu_ = fmu_client_open(fmu_dir.c_str(), info_.name.c_str());
  if (!fmu_) {
    RCLCPP_ERROR(logger(), "fmu_client_open('%s') failed", fmu_dir.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  bool ok = true;
  ok &= find_vr_or_fail(fmu_, "drive_torque_rear", &vr_drive_torque_rear_);
  ok &= find_vr_or_fail(fmu_, "steer_FL_deg", &vr_steer_fl_deg_);
  ok &= find_vr_or_fail(fmu_, "steer_FR_deg", &vr_steer_fr_deg_);
  ok &= find_vr_or_fail(fmu_, "speed_mps", &vr_speed_mps_);
  ok &= find_vr_or_fail(fmu_, "independent_front_steer", &vr_independent_front_steer_);
  ok &= find_vr_or_fail(fmu_, "steer_fl_deg_in", &vr_steer_fl_deg_in_);
  ok &= find_vr_or_fail(fmu_, "steer_fr_deg_in", &vr_steer_fr_deg_in_);
  if (!ok) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 4c: tell the FMU to take FL/FR steer angles independently (see header
  // comment) instead of the old shared steer_deg -- set once here, never
  // touched again, same "structural-ish toggle set once at init" pattern
  // as four_wheel_drive/six_wheel (even though this one is actually
  // checked every step() on the FMU side, not build-time).
  FmuValueReference vr_flag = vr_independent_front_steer_;
  double one = 1.0;
  if (!fmu_client_set_real(fmu_, &vr_flag, 1, &one)) {
    RCLCPP_ERROR(logger(), "fmu_client_set_real(independent_front_steer) failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.reserve(info_.joints.size());
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.state_interfaces.size() != 1) {
      RCLCPP_ERROR(
        logger(), "joint '%s' must declare exactly one command_interface and one state_interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const std::string & iface = joint.command_interfaces[0].name;
    if (iface != hardware_interface::HW_IF_POSITION && iface != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(
        logger(), "joint '%s' has unsupported interface '%s' (only position/velocity wired so far)",
        joint.name.c_str(), iface.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joints_.push_back(JointIO{joint.name, iface, 0.0, 0.0});
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ChronoFmuSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size());
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, joint.interface, &joint.state);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> ChronoFmuSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size());
  for (auto & joint : joints_) {
    interfaces.emplace_back(joint.name, joint.interface, &joint.command);
  }
  return interfaces;
}

hardware_interface::return_type ChronoFmuSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
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
  double vel_cmd_sum = 0.0;
  int vel_cmd_count = 0;
  double vel_measured = 0.0;
  for (const auto & joint : joints_) {
    if (joint.interface == hardware_interface::HW_IF_VELOCITY) {
      vel_cmd_sum += joint.command;
      ++vel_cmd_count;
      vel_measured = joint.state;
    }
  }
  const double vel_cmd = (vel_cmd_count > 0 ? vel_cmd_sum / vel_cmd_count : 0.0);
  const double vel_error = vel_cmd - vel_measured;
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
  double drive_torque_rear = 0.0;
  if (std::abs(vel_error) >= kVelocityDeadband) {
    drive_torque_rear = kVelocityKp * vel_error;
  }
  if (drive_torque_rear > kMaxDriveTorqueRear) {
    drive_torque_rear = kMaxDriveTorqueRear;
  } else if (drive_torque_rear < -kMaxDriveTorqueRear) {
    drive_torque_rear = -kMaxDriveTorqueRear;
  }

  FmuValueReference vrs[3] = {vr_steer_fl_deg_in_, vr_steer_fr_deg_in_, vr_drive_torque_rear_};
  double values[3] = {steer_fl_deg, steer_fr_deg, drive_torque_rear};
  if (!fmu_client_set_real(fmu_, vrs, 3, values)) {
    RCLCPP_ERROR(logger(), "fmu_client_set_real(steer_fl/fr_deg_in, drive_torque_rear) failed");
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
