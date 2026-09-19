# Step 4a local build/verification only -- NOT the real ament package yet
# (that's step 5: package.xml + plugin.xml + colcon). This just needs
# find_package() to pull in hardware_interface's real exported CMake target
# (hardware_interface::hardware_interface) with its full transitive link
# deps, instead of hand-guessing -l flags one linker error at a time.
cmake_minimum_required(VERSION 3.16)
project(chrono_ros2_control_step4a)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)

set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../fmu/cpp/native_fmu/driver)

add_library(chrono_fmu_system_interface SHARED
  src/chrono_fmu_system_interface.cpp
  ${DRIVER_DIR}/fmu_client.c
)
target_include_directories(chrono_fmu_system_interface PUBLIC
  include
  ${DRIVER_DIR}
)
target_link_libraries(chrono_fmu_system_interface
  hardware_interface::hardware_interface
  dl
)

add_executable(chrono_fmu_system_interface_check
  test/chrono_fmu_system_interface_check.cpp
)
target_link_libraries(chrono_fmu_system_interface_check
  chrono_fmu_system_interface
  hardware_interface::hardware_interface
)
