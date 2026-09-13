#!/usr/bin/env bash
# Build fmu_driver: a C program that dlopen()s an FMI2 Co-Simulation .so and
# drives it directly (no fmpy, no Python at all). Needs -ldl for dlopen/dlsym.
set -euo pipefail
cd "$(dirname "$0")"

gcc -O2 -o fmu_driver fmu_driver.c -ldl
echo "Built fmu/cpp/native_fmu/driver/fmu_driver"
echo "Run it against the model .so built by ../build.sh, e.g.:"
echo "  ./fmu_driver ../binaries/linux64/bouncing_ball_native.so bench 10 0.002"
