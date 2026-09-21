# Bridge 2 (the FMU-wrapped dynamics node -> ECU). Raw FMU outputs, no
# conversion applied -- the ECU is responsible for turning speed_mps into
# whatever traction_vel_rad_s (EcuStatus.msg) needs.
float64 steer_fl_deg
float64 steer_fr_deg
float64 speed_mps
