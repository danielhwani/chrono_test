#!/usr/bin/env bash
# Build the native FMI2 C++ API bouncing-ball FMU whose fmi2DoStep is
# backed by a real chrono::ChSystemNSC (see sources/bouncing_ball_native_chrono.cpp)
# -- a separate artifact from ./build.sh's hand-physics bouncing_ball_native.so,
# left untouched. Needs the Chrono C++ headers/libs that ship inside the
# `chrono` conda env (not just the pychrono Python bindings): point
# CHRONO_ENV at it if it's not at the default path below.
set -euo pipefail
cd "$(dirname "$0")"

CHRONO_ENV="${CHRONO_ENV:-$HOME/miniconda3/envs/chrono}"
MODEL_ID=bouncing_ball_native_chrono

if [ ! -f "$CHRONO_ENV/include/chrono/physics/ChSystemNSC.h" ]; then
    echo "Chrono C++ headers not found under $CHRONO_ENV/include/chrono -- set CHRONO_ENV=/path/to/conda/envs/chrono" >&2
    exit 1
fi

g++ -std=c++17 -shared -fPIC -O2 \
    -I"$CHRONO_ENV/include" \
    -I"$CHRONO_ENV/include/eigen3" \
    -I"$CHRONO_ENV/include/chrono/collision/bullet" \
    -o "chrono_variant/binaries/linux64/${MODEL_ID}.so" \
    "sources/${MODEL_ID}.cpp" \
    -L"$CHRONO_ENV/lib" -lChrono_core \
    -Wl,-rpath,"$CHRONO_ENV/lib"
echo "Built chrono_variant/binaries/linux64/${MODEL_ID}.so"

rm -f "${MODEL_ID}.fmu"
(cd chrono_variant && zip -q -r "../${MODEL_ID}.fmu" modelDescription.xml binaries/)
echo "Packaged ${MODEL_ID}.fmu (chrono_variant/ itself is already the unzipped layout -- point fmu_driver at it directly)"
