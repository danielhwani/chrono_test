cmake_minimum_required(VERSION 3.16)
project(chrono_vehicle_ecu)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(chrono_vehicle_msgs REQUIRED)

add_executable(chrono_vehicle_ecu_node src/ecu_node.cpp)
target_compile_features(chrono_vehicle_ecu_node PUBLIC cxx_std_17)
ament_target_dependencies(chrono_vehicle_ecu_node PUBLIC rclcpp chrono_vehicle_msgs)

install(TARGETS chrono_vehicle_ecu_node DESTINATION lib/${PROJECT_NAME})

ament_package()
