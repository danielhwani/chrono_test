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
  ok &= find_vr_or_fail(fmu_, "steer_deg", &vr_steer_deg_);
  ok &= find_vr_or_fail(fmu_, "drive_torque_rear", &vr_drive_torque_rear_);
  ok &= find_vr_or_fail(fmu_, "steer_FL_deg", &vr_steer_fl_deg_);
  ok &= find_vr_or_fail(fmu_, "steer_FR_deg", &vr_steer_fr_deg_);
  ok &= find_vr_or_fail(fmu_, "speed_mps", &vr_speed_mps_);
  if (!ok) {
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
  // 4c placeholder: average the two independently-commanded front steering
  // angles into the FMU's single steer_deg input. Not the final design --
  // see the header comment and README.
  double steer_sum_rad = 0.0;
  int steer_count = 0;
  for (const auto & joint : joints_) {
    if (joint.interface == hardware_interface::HW_IF_POSITION) {
      steer_sum_rad += joint.command;
      ++steer_count;
    }
  }
  const double steer_deg =
    (steer_count > 0 ? (steer_sum_rad / steer_count) : 0.0) * kDegPerRad;

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
  double drive_torque_rear = kVelocityKp * (vel_cmd - vel_measured);
  if (drive_torque_rear > kMaxDriveTorqueRear) {
    drive_torque_rear = kMaxDriveTorqueRear;
  } else if (drive_torque_rear < -kMaxDriveTorqueRear) {
    drive_torque_rear = -kMaxDriveTorqueRear;
  }

  FmuValueReference vrs[2] = {vr_steer_deg_, vr_drive_torque_rear_};
  double values[2] = {steer_deg, drive_torque_rear};
  if (!fmu_client_set_real(fmu_, vrs, 2, values)) {
    RCLCPP_ERROR(logger(), "fmu_client_set_real(steer_deg, drive_torque_rear) failed");
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
