cmake_minimum_required(VERSION 3.16)
project(chrono_remote_fmu_hw_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  chrono_split_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(chrono_remote_fmu_system_interface SHARED
  src/chrono_remote_fmu_system_interface.cpp
)
target_compile_features(chrono_remote_fmu_system_interface PUBLIC cxx_std_17)
target_include_directories(chrono_remote_fmu_system_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(chrono_remote_fmu_system_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Matches the <plugin>chrono_remote_fmu_hw_interface/ChronoRemoteFmuSystemInterface</plugin>
# string a URDF using this plugin needs.
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS chrono_remote_fmu_system_interface
  EXPORT export_chrono_remote_fmu_system_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_chrono_remote_fmu_system_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
