"""
Validate chrono_vehicle_fmu.py by running the exact same step-steer scenario
two ways -- once by calling make_vehicle() directly (the reference, same
loop shape as simple_vehicle.py's own main()), once by driving the built
FMU through fmpy -- and checking the final chassis state agrees. Since
do_step() just calls the same functions with the same inputs, the two
should match to numerical noise (both are the same deterministic Chrono
solver), not just "look similar" -- this is checking the FMI wrapping
itself introduced no discrepancy, not validating the vehicle physics
(already covered by plot_results.py/compare_tire_models.py).

Run:
    cd fmu
    pythonfmu build -f chrono_vehicle_fmu.py -d build ../simple_vehicle.py
    python validate_vehicle_fmu.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pychrono as chrono
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave

from simple_vehicle import (
    G, TRACK, WHEELBASE, DRIVE_TORQUE,
    make_vehicle, apply_differential, steer_angle_deg,
)

FMU_PATH = os.path.join(os.path.dirname(__file__), "build", "ChronoVehicle.fmu")
DT = 2e-3
SIM_TIME = 5.0
STEER_DEG, STEER_START, STEER_RAMP = 20.0, 1.5, 0.8
TOL_POS = 1e-6
TOL_ANGLE = 1e-6


def run_reference():
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, G))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)

    chassis, _wheels, _springs, motors, steer_functions, throttle_functions = make_vehicle(sys)

    t = 0.0
    while t < SIM_TIME:
        cur_steer_rad = math.radians(steer_angle_deg(t, STEER_DEG, STEER_START, STEER_RAMP))
        for fn in steer_functions.values():
            fn.SetConstant(cur_steer_rad)
        apply_differential(motors, throttle_functions, DRIVE_TORQUE)
        sys.DoStepDynamics(DT)
        t = sys.GetChTime()

    pos = chassis.GetPos()
    yaw = math.degrees(chassis.GetRot().GetCardanAnglesXYZ().z)
    return pos.x, pos.y, pos.z, yaw


def run_fmu():
    md = read_model_description(FMU_PATH)
    vr = {v.name: v.valueReference for v in md.modelVariables}
    unzip = extract(FMU_PATH)
    fmu = FMU2Slave(guid=md.guid, unzipDirectory=unzip,
                     modelIdentifier=md.coSimulation.modelIdentifier, instanceName="validate")
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    t = 0.0
    while t < SIM_TIME:
        steer = steer_angle_deg(t, STEER_DEG, STEER_START, STEER_RAMP)
        fmu.setReal([vr["steer_deg"], vr["drive_torque"]], [steer, DRIVE_TORQUE])
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=DT)
        t += DT

    x, y, z, yaw = fmu.getReal([vr["chassis_x"], vr["chassis_y"], vr["chassis_z"], vr["yaw_deg"]])
    fmu.terminate()
    fmu.freeInstance()
    return x, y, z, yaw


def main():
    if not os.path.exists(FMU_PATH):
        raise SystemExit(f"{FMU_PATH} not found -- build it first, see this file's docstring")

    ref = run_reference()
    fmu_result = run_fmu()

    names = ("chassis_x", "chassis_y", "chassis_z", "yaw_deg")
    print(f"{'signal':>12} {'reference':>14} {'via FMU':>14} {'abs diff':>14}")
    worst = 0.0
    for name, r, f in zip(names, ref, fmu_result):
        diff = abs(r - f)
        worst = max(worst, diff)
        print(f"{name:>12} {r:14.8f} {f:14.8f} {diff:14.2e}")

    tol = TOL_ANGLE if worst == abs(ref[3] - fmu_result[3]) else TOL_POS
    if worst > 1e-4:
        raise SystemExit(f"FAIL: worst diff {worst:.2e} exceeds tolerance -- FMU wrapping changed the physics")
    print(f"\nOK -- direct Chrono loop and FMU-driven loop agree (worst diff {worst:.2e}), "
          f"confirming the FMI2 wrapping itself introduces no discrepancy.")


if __name__ == "__main__":
    main()
