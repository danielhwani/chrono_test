# Bridge 2 (ECU -> the FMU-wrapped dynamics node). Native FMU units (deg,
# N*m), axle-level torques only -- L/R split is the FMU's own
# apply_differential()'s job, not the ECU's. Front/mid/rear mirrors
# native_vehicle_fmu's drive_torque_front/_mid/_rear inputs exactly.
# drive_torque_mid_nm is unused (ignored by the FMU) until six_wheel is
# enabled -- included now for the planned 6x6 extension, same
# additive/off-by-default pattern the FMU itself already uses.
float64 steer_fl_deg
float64 steer_fr_deg
float64 drive_torque_front_nm
float64 drive_torque_mid_nm
float64 drive_torque_rear_nm
