cmake_minimum_required(VERSION 3.16)
project(chrono_split_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EcuCommand.msg"
  "msg/EcuStatus.msg"
  "msg/VehicleCommand.msg"
  "msg/VehicleStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
