#!/usr/bin/env bash
# Build fmu_driver: a C program that dlopen()s an FMI2 Co-Simulation .so and
# drives it directly (no fmpy, no Python at all). Needs -ldl for dlopen/dlsym.
set -euo pipefail
cd "$(dirname "$0")"

gcc -O2 -o fmu_driver fmu_driver.c -ldl
echo "Built fmu/cpp/native_fmu/driver/fmu_driver"
echo "Point it at an *extracted* FMU directory (modelDescription.xml + binaries/linux64/*.so), e.g.:"
echo "  ./fmu_driver .. bench 10 0.002                      # our own native_fmu/"
echo "  ./fmu_driver ../../../modelica/extracted paced 10 0.002   # any FMU with Real h/v, e.g. Modelica-built"
