cmake_minimum_required(VERSION 3.16)
project(chrono_split_dynamics_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(chrono_split_msgs REQUIRED)

# TODO once fmu_client is actually wired in (see src/dynamics_node.cpp):
# add fmu/cpp/native_fmu/driver/fmu_client.c as a source here and its
# directory to target_include_directories, exactly like
# chrono_ros2_control/CMakeLists.txt's DRIVER_DIR does, plus -ldl.
add_executable(chrono_split_dynamics_node src/dynamics_node.cpp)
target_compile_features(chrono_split_dynamics_node PUBLIC cxx_std_17)
ament_target_dependencies(chrono_split_dynamics_node PUBLIC rclcpp chrono_split_msgs)

install(TARGETS chrono_split_dynamics_node DESTINATION lib/${PROJECT_NAME})

ament_package()
