# Bridge 1 (ros2_control -> virtual/real ECU). Stands in for what a real
# vehicle's CAN bus would carry from the supervisory controller down to the
# ECU -- units match hardware_interface's command_interface conventions
# (rad, rad/s), NOT the FMU's native units (deg, N*m). The ECU is the layer
# responsible for converting this into VehicleCommand.msg (see that file).
float64 steer_fl_rad
float64 steer_fr_rad
float64 traction_vel_rad_s
