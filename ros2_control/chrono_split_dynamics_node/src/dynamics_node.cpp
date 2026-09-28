// FMU-wrapped dynamics node in the split architecture: subscribes
// VehicleCommand (from chrono_split_ecu, Bridge 2), publishes
// VehicleStatus. Wraps fmu_client exactly the way
// chrono_ros2_control's chrono_fmu_system_interface.cpp does (same
// open/find_vr/set_real/do_step/get_real call pattern, same VRs, same
// step size) -- that file is untouched, this is a plain ROS2 node instead
// of a hardware_interface plugin.
//
// Unlike the ECU node, this one forwards all three axle torques
// (front/mid/rear) unconditionally -- four_wheel_drive/six_wheel stay at
// the FMU's own default-off values (never set here, matching
// chrono_ros2_control exactly for the planned cross-check), so
// drive_torque_front/_mid are simply ignored by the FMU whenever no
// front/mid drive motor was built. Being a complete, generic FMU wrapper
// here (rather than only wiring what today's ECU happens to send) keeps
// this node reusable once 4WD/six_wheel are wired up on the ECU side --
// no changes would be needed here for that.
//
// fmu_dir is a ROS2 node parameter (not a URDF hardware_parameter, since
// this isn't a hardware_interface component) -- set via
// --ros-args -p fmu_dir:=<path> or a launch file's parameters.
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

class ChronoSplitDynamicsNode : public rclcpp::Node
{
public:
  ChronoSplitDynamicsNode() : Node("chrono_split_dynamics_node")
  {
    const std::string fmu_dir = declare_parameter<std::string>("fmu_dir", "");
    if (fmu_dir.empty()) {
      throw std::invalid_argument("required parameter 'fmu_dir' not set");
    }

    fmu_ = fmu_client_open(fmu_dir.c_str(), get_name());
    if (!fmu_) {
      throw std::runtime_error("fmu_client_open('" + fmu_dir + "') failed");
    }

    bool ok = true;
    ok &= find_vr_or_throw("drive_torque_front", &vr_drive_torque_front_);
    ok &= find_vr_or_throw("drive_torque_mid", &vr_drive_torque_mid_);
    ok &= find_vr_or_throw("drive_torque_rear", &vr_drive_torque_rear_);
    ok &= find_vr_or_throw("steer_fl_deg_in", &vr_steer_fl_deg_in_);
    ok &= find_vr_or_throw("steer_fr_deg_in", &vr_steer_fr_deg_in_);
    ok &= find_vr_or_throw("independent_front_steer", &vr_independent_front_steer_);
    ok &= find_vr_or_throw("steer_FL_deg", &vr_steer_fl_deg_);
    ok &= find_vr_or_throw("steer_FR_deg", &vr_steer_fr_deg_);
    ok &= find_vr_or_throw("speed_mps", &vr_speed_mps_);
    if (!ok) {
      throw std::runtime_error("one or more required FMU variables not found");
    }

    // Same as chrono_fmu_system_interface's on_init(): take FL/FR steer
    // angles independently instead of the old shared steer_deg, set once.
    double one = 1.0;
    if (!fmu_client_set_real(fmu_, &vr_independent_front_steer_, 1, &one)) {
      throw std::runtime_error("fmu_client_set_real(independent_front_steer) failed");
    }

    vehicle_command_sub_ = create_subscription<VehicleCommand>(
      "vehicle_command", rclcpp::SystemDefaultsQoS(),
      [this](const VehicleCommand::SharedPtr msg) { latest_vehicle_command_ = *msg; });
    vehicle_status_pub_ =
      create_publisher<VehicleStatus>("vehicle_status", rclcpp::SystemDefaultsQoS());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&ChronoSplitDynamicsNode::tick, this));
  }

  ~ChronoSplitDynamicsNode() override { fmu_client_close(fmu_); }

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
    rclcpp::spin(std::make_shared<ChronoSplitDynamicsNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("chrono_split_dynamics_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
