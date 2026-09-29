# The bridge to chrono_fmu_dynamics_node -- shared by both 스플릿 버전
# (sent by chrono_split_ecu, its Bridge 2 of two) and 분산 버전 (sent
# directly by chrono_supervisory_fmu_hw_interface's
# ChronoSupervisoryFmuSystemInterface, its only bridge -- no ECU node on
# that track). Native FMU units (deg, N*m), axle-level torques only --
# L/R split is the FMU's own apply_differential()'s job, not the
# sender's. Front/mid/rear mirrors native_vehicle_fmu's
# drive_torque_front/_mid/_rear inputs exactly. drive_torque_mid_nm is
# unused (ignored by the FMU) unless six_wheel is enabled -- same
# additive/off-by-default pattern the FMU itself already uses.
float64 steer_fl_deg
float64 steer_fr_deg
float64 drive_torque_front_nm
float64 drive_torque_mid_nm
float64 drive_torque_rear_nm
