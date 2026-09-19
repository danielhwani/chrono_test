# Step 5 of the ros2_control prep plan: a real ament_cmake package (colcon-
# buildable), replacing step 4's local-verification-only CMakeLists.txt.
# Structure/macro usage cross-checked against the real, official
# ros2_control_demos example_2 package (github.com/ros-controls/
# ros2_control_demos, humble branch, example_2/CMakeLists.txt +
# package.xml) rather than guessed -- pluginlib_export_plugin_description_file()
# is the only thing that actually registers the plugin; package.xml needs
# no matching <hardware_interface plugin=.../> export tag (confirmed by
# that real package.xml having none).
cmake_minimum_required(VERSION 3.16)
project(chrono_ros2_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# fmu_client (fmu/cpp/native_fmu/driver/) lives outside this package's own
# tree -- it's a small shared C library other consumers in this repo
# (fmu_driver, fmu_client_cpp_check) already link the same way, not
# duplicated here. Not a separate ROS package (this repo isn't otherwise a
# ROS workspace), so it's pulled in as a plain source file + include dir
# rather than via find_package().
set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../fmu/cpp/native_fmu/driver)

add_library(chrono_fmu_system_interface SHARED
  src/chrono_fmu_system_interface.cpp
  ${DRIVER_DIR}/fmu_client.c
)
target_compile_features(chrono_fmu_system_interface PUBLIC cxx_std_17)
target_include_directories(chrono_fmu_system_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${DRIVER_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(chrono_fmu_system_interface PUBLIC dl)
ament_target_dependencies(chrono_fmu_system_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Export hardware plugin -- matches the <plugin>chrono_ros2_control/ChronoFmuSystemInterface</plugin>
# string already in ros2_control/urdf/chrono_vehicle.urdf (step 3).
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES ${DRIVER_DIR}/fmu_client.h
  DESTINATION include
)
install(
  TARGETS chrono_fmu_system_interface
  EXPORT export_chrono_fmu_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Kept from step 4's local-verification CMakeLists.txt -- not a colcon
# test, just a standalone terminal-verification tool (see README's 4a/4b/4c
# sections for example runs). Always built, not gated behind BUILD_TESTING.
add_executable(chrono_fmu_system_interface_check
  test/chrono_fmu_system_interface_check.cpp
)
target_link_libraries(chrono_fmu_system_interface_check PUBLIC
  chrono_fmu_system_interface
)
ament_target_dependencies(chrono_fmu_system_interface_check PUBLIC hardware_interface)

ament_export_targets(export_chrono_fmu_system_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
