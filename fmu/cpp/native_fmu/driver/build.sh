#!/usr/bin/env bash
# Build fmu_driver: a C program that dlopen()s an FMI2 Co-Simulation .so and
# drives it directly (no fmpy, no Python at all). fmu_client.c has the
# FMI-loading logic (shared with the planned ros2_control plugin); this
# just compiles it alongside fmu_driver.c's CLI/mode-loop code. Needs -ldl
# for dlopen/dlsym.
set -euo pipefail
cd "$(dirname "$0")"

gcc -O2 -o fmu_driver fmu_driver.c fmu_client.c -ldl
echo "Built fmu/cpp/native_fmu/driver/fmu_driver"
echo "Point it at an *extracted* FMU directory (modelDescription.xml + binaries/linux64/*.so), e.g.:"
echo "  ./fmu_driver .. bench 10 0.002                      # our own native_fmu/ (default outputs: h,v)"
echo "  ./fmu_driver ../../../modelica/extracted paced 10 0.002   # any FMU with Real h/v, e.g. Modelica-built"
echo "  ./fmu_driver ../../native_vehicle_fmu csv 5 0.002 --set steer_deg=15 --set drive_torque_rear=260 \\"
echo "               --outputs chassis_x,chassis_y,yaw_deg,speed_mps   # any FMU, arbitrary inputs/outputs"
