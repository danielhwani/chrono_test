#!/usr/bin/env bash
# Build the native FMI2 C++ vehicle FMU (see sources/vehicle_native.cpp).
# Mirrors fmu/cpp/native_fmu/build_chrono.sh's flags -- same conda `chrono`
# env ships Chrono's C++ headers/libs, Eigen, and its bundled Bullet.
set -euo pipefail
cd "$(dirname "$0")"

CHRONO_ENV="${CHRONO_ENV:-$HOME/miniconda3/envs/chrono}"
MODEL_ID=vehicle_native

if [ ! -f "$CHRONO_ENV/include/chrono/physics/ChSystemNSC.h" ]; then
    echo "Chrono C++ headers not found under $CHRONO_ENV/include/chrono -- set CHRONO_ENV=/path/to/conda/envs/chrono" >&2
    exit 1
fi

mkdir -p binaries/linux64

g++ -std=c++17 -shared -fPIC -O2 \
    -I"$CHRONO_ENV/include" \
    -I"$CHRONO_ENV/include/eigen3" \
    -I"$CHRONO_ENV/include/chrono/collision/bullet" \
    -o "binaries/linux64/${MODEL_ID}.so" \
    "sources/${MODEL_ID}.cpp" \
    -L"$CHRONO_ENV/lib" -lChrono_core \
    -Wl,-rpath,"$CHRONO_ENV/lib"
echo "Built binaries/linux64/${MODEL_ID}.so"

rm -f "${MODEL_ID}.fmu"
zip -q -r "${MODEL_ID}.fmu" modelDescription.xml binaries/
echo "Packaged ${MODEL_ID}.fmu (native_vehicle_fmu/ itself is already the unzipped layout -- point fmu_driver at it directly)"
