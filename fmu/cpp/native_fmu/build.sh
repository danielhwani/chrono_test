#!/usr/bin/env bash
# Build the native FMI2 C API bouncing-ball FMU: compile the C source into a
# shared library, drop it in the FMU's standard binaries/ layout, and zip the
# whole directory into a .fmu package (an FMU is just a zip with this layout:
# modelDescription.xml + binaries/<platform>/<modelIdentifier>.so).
set -euo pipefail
cd "$(dirname "$0")"

MODEL_ID=bouncing_ball_native

gcc -shared -fPIC -O2 -o "binaries/linux64/${MODEL_ID}.so" "sources/${MODEL_ID}.c"
echo "Built binaries/linux64/${MODEL_ID}.so"

rm -f "${MODEL_ID}.fmu"
zip -q -r "${MODEL_ID}.fmu" modelDescription.xml binaries/
echo "Packaged ${MODEL_ID}.fmu"
