#!/usr/bin/env bash
# Compile BouncingBallModelica.mo into an FMI2 Co-Simulation FMU with
# OpenModelica's omc, then unzip it next to itself (fmu_driver.c wants an
# already-extracted FMU directory: modelDescription.xml + binaries/linux64/).
set -euo pipefail
cd "$(dirname "$0")"

omc build.mos
rm -rf extracted
mkdir -p extracted
unzip -q BouncingBallModelica.fmu -d extracted
echo "Built BouncingBallModelica.fmu and unzipped it into extracted/"
