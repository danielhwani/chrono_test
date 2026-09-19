// Step 4a terminal verification -- same spirit as
// fmu/cpp/native_fmu/driver/fmu_client_cpp_check.cpp (step 2): a standalone
// program that exercises the real class against the real URDF, without
// needing a full colcon package or a live controller_manager (that's step
// 7). Not part of the plugin itself -- never built into the .so pluginlib
// will load.
//
// What this actually proves: chrono_fmu_system_interface.cpp compiles and
// links against the real hardware_interface headers/libs, on_init() can
// parse chrono_vehicle.urdf's real <ros2_control> block via
// hardware_interface::parse_control_resources_from_urdf() (not a hand-built
// fake HardwareInfo) and open the real FMU through it, and write()/read()
// drive the FMU and report state across many steps without erroring.
#include <cstdio>
#include <fstream>
#include <sstream>

#include "chrono_ros2_control/chrono_fmu_system_interface.hpp"
#include "hardware_interface/component_parser.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <path-to-chrono_vehicle.urdf>\n", argv[0]);
    return 1;
  }

  std::ifstream file(argv[1]);
  if (!file) {
    std::fprintf(stderr, "could not open %s\n", argv[1]);
    return 1;
  }
  std::stringstream ss;
  ss << file.rdbuf();
  const std::string urdf = ss.str();

  auto hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf);
  if (hardware_infos.empty()) {
    std::fprintf(stderr, "no <ros2_control> hardware found in %s\n", argv[1]);
    return 1;
  }
  std::printf(
    "parsed hardware '%s' (%zu joints) from URDF\n", hardware_infos[0].name.c_str(),
    hardware_infos[0].joints.size());

  chrono_ros2_control::ChronoFmuSystemInterface iface;
  if (iface.on_init(hardware_infos[0]) != hardware_interface::CallbackReturn::SUCCESS) {
    std::fprintf(stderr, "on_init failed\n");
    return 1;
  }
  std::printf("on_init OK\n");

  auto state_interfaces = iface.export_state_interfaces();
  auto command_interfaces = iface.export_command_interfaces();
  std::printf(
    "exported %zu state_interfaces, %zu command_interfaces\n", state_interfaces.size(),
    command_interfaces.size());
  for (auto & si : state_interfaces) {
    std::printf("  state: %s\n", si.get_name().c_str());
  }
  for (auto & ci : command_interfaces) {
    std::printf("  command: %s\n", ci.get_name().c_str());
  }

  // command 0.3 rad (~17 deg) on the left steering joint only, to check
  // read() after write() actually reflects it (via the 4c average, so
  // ~0.15 rad on both wheels' reported state -- not a bug, the documented
  // placeholder behavior).
  //
  // 5.0 rad/s on BOTH rear wheels (the common straight-driving case -- an
  // asymmetric L/R command would just average into one axle target too,
  // same placeholder limitation as steer_deg, not a new one) to exercise
  // 4b's velocity->torque P loop and watch it converge (or not).
  for (auto & ci : command_interfaces) {
    if (ci.get_name().find("front_left_steering_joint") != std::string::npos) {
      ci.set_value(0.3);
    } else if (ci.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
      ci.set_value(5.0);
    }
  }

  rclcpp::Time t(0, 0, RCL_ROS_TIME);
  rclcpp::Duration period = rclcpp::Duration::from_seconds(0.002);
  const int n_steps = 3000;  // 6.0s of sim time, matching validate scripts' scale
  const int print_every = 500;  // 1.0s -- trace convergence, don't just show the endpoint
  for (int i = 0; i < n_steps; ++i) {
    if (iface.write(t, period) != hardware_interface::return_type::OK) {
      std::fprintf(stderr, "write() failed at step %d\n", i);
      return 1;
    }
    if (iface.read(t, period) != hardware_interface::return_type::OK) {
      std::fprintf(stderr, "read() failed at step %d\n", i);
      return 1;
    }
    if ((i + 1) % print_every == 0) {
      std::printf("t=%.1fs:", (i + 1) * 0.002);
      for (auto & si : state_interfaces) {
        if (si.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
          std::printf("  %s=%f", si.get_name().c_str(), si.get_value());
        }
      }
      std::printf("\n");
    }
  }

  std::printf("after %d steps (%.3fs sim time):\n", n_steps, n_steps * 0.002);
  for (auto & si : state_interfaces) {
    std::printf("  %s = %f\n", si.get_name().c_str(), si.get_value());
  }

  return 0;
}
