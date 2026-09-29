# Symmetric with VehicleCommand.msg -- chrono_fmu_dynamics_node's own
# status, shared by both 스플릿 버전 (received by chrono_split_ecu) and
# 분산 버전 (received directly by ChronoSupervisoryFmuSystemInterface, no
# ECU node on that track). Raw FMU outputs, no conversion applied -- the
# receiver is responsible for turning speed_mps into whatever velocity
# units it needs (e.g. EcuStatus.msg's traction_vel_rad_s on 스플릿 버전).
float64 steer_fl_deg
float64 steer_fr_deg
float64 speed_mps
