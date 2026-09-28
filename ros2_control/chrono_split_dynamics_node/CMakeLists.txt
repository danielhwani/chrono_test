cmake_minimum_required(VERSION 3.16)
project(chrono_split_dynamics_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(chrono_split_msgs REQUIRED)

# fmu_client (fmu/cpp/native_fmu/driver/) lives outside this package's own
# tree -- same small shared C library fmu_driver/fmu_client_cpp_check/
# chrono_ros2_control already link, not duplicated here. See
# chrono_ros2_control/CMakeLists.txt's DRIVER_DIR for the identical pattern.
set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../fmu/cpp/native_fmu/driver)

add_executable(chrono_split_dynamics_node src/dynamics_node.cpp ${DRIVER_DIR}/fmu_client.c)
target_compile_features(chrono_split_dynamics_node PUBLIC cxx_std_17)
target_include_directories(chrono_split_dynamics_node PUBLIC ${DRIVER_DIR})
target_link_libraries(chrono_split_dynamics_node PUBLIC dl)
ament_target_dependencies(chrono_split_dynamics_node PUBLIC rclcpp chrono_split_msgs)

install(TARGETS chrono_split_dynamics_node DESTINATION lib/${PROJECT_NAME})

ament_package()
