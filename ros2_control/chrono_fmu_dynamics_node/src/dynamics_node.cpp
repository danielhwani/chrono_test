// Generic FMU-wrapped dynamics node, shared by BOTH the split (스플릿
// 버전, chrono_split_ecu upstream) and distributed (분산 버전,
// chrono_supervisory_fmu_hw_interface upstream) ros2_control tracks --
// package renamed from chrono_split_dynamics_node once the distributed
// track started reusing it too, since it was never actually split-track-
// specific. Subscribes Bridge 2's VehicleCommand, publishes
// VehicleStatus, wraps fmu_client exactly the way chrono_ros2_control's
// chrono_fmu_system_interface.cpp does (same open/find_vr/set_real/
// do_step/get_real call pattern, same VRs, same step size) -- that file
// is untouched, this is a plain ROS2 node instead of a hardware_interface
// plugin. No control logic, no knowledge of which track's upstream
// node/plugin is talking to it -- this is exactly what let it be reused
// unchanged for the distributed track.
//
// Forwards all three axle torques (front/mid/rear) unconditionally --
// six_wheel stays at the FMU's own default-off value unless this node's
// own six_wheel parameter says otherwise. Being a complete, generic FMU
// wrapper here (rather than only wiring what the upstream side happens to
// send) is what made this node reusable across two different upstream
// designs with zero changes.
//
// four_wheel_drive=1.0 IS set here (matching chrono_ros2_control's own
// 4WD upgrade, same session) via fmu_client_open_begin()/_finish() rather
// than the single-call fmu_client_open() -- four_wheel_drive is a
// genuinely structural FMU parameter, read exactly once inside
// native_vehicle_fmu's own fmi2ExitInitializationMode handler to decide
// whether a front drive motor gets built at all. Setting it after a
// single-call open() would be silently too late to have any effect --
// exactly the bug found and fixed in chrono_ros2_control first (see
// fmu_client.h's header comment for the full story).
//
// fmu_dir is a ROS2 node parameter (not a URDF hardware_parameter, since
// this isn't a hardware_interface component) -- set via
// --ros-args -p fmu_dir:=<path> or a launch file's parameters.
//
// six_wheel is ALSO a ROS2 node parameter (default false), unlike
// four_wheel_drive above -- it's conditional because, unlike 4WD (now
// always on for this track), turning it on changes the FMU's whole
// chassis/wheelbase geometry, which would silently break the 4-wheel URDF
// if forced on unconditionally. Set via fmu_client_open_begin()/_finish()
// same as four_wheel_drive, for the same structural-parameter reason.
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "chrono_split_msgs/msg/vehicle_command.hpp"
#include "chrono_split_msgs/msg/vehicle_status.hpp"

extern "C" {
#include "fmu_client.h"
}

using chrono_split_msgs::msg::VehicleCommand;
using chrono_split_msgs::msg::VehicleStatus;

namespace
{
// Matches native_vehicle_fmu/modelDescription.xml's DefaultExperiment
// stepSize -- same constant/rationale as
// chrono_fmu_system_interface.hpp's kStepSize.
constexpr double kStepSize = 0.002;
}  // namespace

class ChronoFmuDynamicsNode : public rclcpp::Node
{
public:
  ChronoFmuDynamicsNode() : Node("chrono_fmu_dynamics_node")
  {
    const std::string fmu_dir = declare_parameter<std::string>("fmu_dir", "");
    if (fmu_dir.empty()) {
      throw std::invalid_argument("required parameter 'fmu_dir' not set");
    }
    const bool six_wheel = declare_parameter<bool>("six_wheel", false);

    fmu_ = fmu_client_open_begin(fmu_dir.c_str(), get_name());
    if (!fmu_) {
      throw std::runtime_error("fmu_client_open_begin('" + fmu_dir + "') failed");
    }

    bool ok = true;
    ok &= find_vr_or_throw("drive_torque_front", &vr_drive_torque_front_);
    ok &= find_vr_or_throw("drive_torque_mid", &vr_drive_torque_mid_);
    ok &= find_vr_or_throw("drive_torque_rear", &vr_drive_torque_rear_);
    ok &= find_vr_or_throw("steer_fl_deg_in", &vr_steer_fl_deg_in_);
    ok &= find_vr_or_throw("steer_fr_deg_in", &vr_steer_fr_deg_in_);
    ok &= find_vr_or_throw("independent_front_steer", &vr_independent_front_steer_);
    ok &= find_vr_or_throw("four_wheel_drive", &vr_four_wheel_drive_);
    ok &= find_vr_or_throw("six_wheel", &vr_six_wheel_);
    ok &= find_vr_or_throw("steer_FL_deg", &vr_steer_fl_deg_);
    ok &= find_vr_or_throw("steer_FR_deg", &vr_steer_fr_deg_);
    ok &= find_vr_or_throw("speed_mps", &vr_speed_mps_);
    if (!ok) {
      throw std::runtime_error("one or more required FMU variables not found");
    }

    // Same as chrono_fmu_system_interface's on_init(): take FL/FR steer
    // angles independently instead of the old shared steer_deg, turn on
    // four_wheel_drive, and turn on six_wheel IFF the six_wheel parameter
    // asked for it -- all set once, still in initialization mode (see file
    // header comment for why four_wheel_drive/six_wheel specifically must
    // be set before open_finish()).
    FmuValueReference vr_flags[3] = {
      vr_independent_front_steer_, vr_four_wheel_drive_, vr_six_wheel_};
    double flag_values[3] = {1.0, 1.0, six_wheel ? 1.0 : 0.0};
    if (!fmu_client_set_real(fmu_, vr_flags, 3, flag_values)) {
      throw std::runtime_error(
        "fmu_client_set_real(independent_front_steer, four_wheel_drive, six_wheel) failed");
    }

    if (!fmu_client_open_finish(fmu_)) {
      throw std::runtime_error("fmu_client_open_finish('" + fmu_dir + "') failed");
    }

    vehicle_command_sub_ = create_subscription<VehicleCommand>(
      "vehicle_command", rclcpp::SystemDefaultsQoS(),
      [this](const VehicleCommand::SharedPtr msg) { latest_vehicle_command_ = *msg; });
    vehicle_status_pub_ =
      create_publisher<VehicleStatus>("vehicle_status", rclcpp::SystemDefaultsQoS());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&ChronoFmuDynamicsNode::tick, this));
  }

  ~ChronoFmuDynamicsNode() override { fmu_client_close(fmu_); }

private:
  bool find_vr_or_throw(const char * name, FmuValueReference * out)
  {
    if (!fmu_client_find_vr(fmu_, name, out)) {
      RCLCPP_ERROR(get_logger(), "FMU has no variable named '%s'", name);
      return false;
    }
    return true;
  }

  void tick()
  {
    FmuValueReference set_vrs[5] = {
      vr_steer_fl_deg_in_, vr_steer_fr_deg_in_, vr_drive_torque_front_, vr_drive_torque_mid_,
      vr_drive_torque_rear_};
    double set_values[5] = {
      latest_vehicle_command_.steer_fl_deg, latest_vehicle_command_.steer_fr_deg,
      latest_vehicle_command_.drive_torque_front_nm, latest_vehicle_command_.drive_torque_mid_nm,
      latest_vehicle_command_.drive_torque_rear_nm};
    if (!fmu_client_set_real(fmu_, set_vrs, 5, set_values)) {
      RCLCPP_ERROR(get_logger(), "fmu_client_set_real failed");
      return;
    }

    if (!fmu_client_do_step(fmu_, sim_time_, kStepSize)) {
      RCLCPP_ERROR(get_logger(), "fmu_client_do_step failed at t=%f", sim_time_);
      return;
    }
    sim_time_ += kStepSize;

    FmuValueReference get_vrs[3] = {vr_steer_fl_deg_, vr_steer_fr_deg_, vr_speed_mps_};
    double get_values[3] = {0.0, 0.0, 0.0};
    if (!fmu_client_get_real(fmu_, get_vrs, 3, get_values)) {
      RCLCPP_ERROR(get_logger(), "fmu_client_get_real failed");
      return;
    }

    VehicleStatus vs;
    vs.steer_fl_deg = get_values[0];
    vs.steer_fr_deg = get_values[1];
    vs.speed_mps = get_values[2];
    vehicle_status_pub_->publish(vs);
  }

  FmuClient * fmu_ = nullptr;
  FmuValueReference vr_drive_torque_front_{};
  FmuValueReference vr_drive_torque_mid_{};
  FmuValueReference vr_drive_torque_rear_{};
  FmuValueReference vr_steer_fl_deg_in_{};
  FmuValueReference vr_steer_fr_deg_in_{};
  FmuValueReference vr_independent_front_steer_{};
  FmuValueReference vr_four_wheel_drive_{};
  FmuValueReference vr_six_wheel_{};
  FmuValueReference vr_steer_fl_deg_{};
  FmuValueReference vr_steer_fr_deg_{};
  FmuValueReference vr_speed_mps_{};

  double sim_time_ = 0.0;

  rclcpp::Subscription<VehicleCommand>::SharedPtr vehicle_command_sub_;
  rclcpp::Publisher<VehicleStatus>::SharedPtr vehicle_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleCommand latest_vehicle_command_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ChronoFmuDynamicsNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("chrono_fmu_dynamics_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
