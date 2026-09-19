// Step 5 terminal verification -- proves pluginlib's REAL discovery+factory
// mechanism (ament index -> plugin.xml -> dlopen -> RTTI-based class
// factory) can find and construct ChronoFmuSystemInterface by its
// registered name, exactly the path controller_manager will use in step 7.
// Deliberately does NOT #include chrono_fmu_system_interface.hpp directly
// (that's what chrono_fmu_system_interface_check.cpp already tests, and it
// bypasses pluginlib's discovery/factory machinery entirely -- a class
// missing PLUGINLIB_EXPORT_CLASS would still pass that check but fail
// here, which is exactly what happened once during step 5's development).
#include <cstdio>

#include "pluginlib/class_loader.hpp"
#include "hardware_interface/system_interface.hpp"

int main()
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto instance = loader.createSharedInstance("chrono_ros2_control/ChronoFmuSystemInterface");
  std::printf("pluginlib successfully created an instance: %s\n", typeid(*instance).name());
  return 0;
}
