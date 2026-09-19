"""
Validate fmu/cpp/native_vehicle_fmu/vehicle_native.fmu (native C++, calls
chrono::ChSystemNSC directly) against fmu/build/ChronoVehicle.fmu
(chrono_vehicle_fmu.py, pythonfmu) -- same scenario through both, via fmpy.
Both call the same underlying Chrono solver, so a match isn't just "close",
it's expected to be exact to the printed digits; a real discrepancy would
mean something was mistranslated porting the model to C++.

Run:
    cd fmu
    pythonfmu build -f chrono_vehicle_fmu.py -d build ../simple_vehicle.py
    (cd cpp/native_vehicle_fmu && ./build.sh)
    python validate_native_vehicle_fmu.py
"""
import os

from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave

HERE = os.path.dirname(os.path.abspath(__file__))
PYTHONFMU_PATH = os.path.join(HERE, "build", "ChronoVehicle.fmu")
NATIVE_PATH = os.path.join(HERE, "cpp", "native_vehicle_fmu", "vehicle_native.fmu")

DT = 0.005
SIM_TIME = 6.0
STEER_DEG, STEER_START = 20.0, 1.0
DRIVE_TORQUE = 260.0
DRIVE_TORQUE_FRONT = 150.0
OUTPUTS = ("chassis_x", "chassis_y", "chassis_z", "yaw_deg", "speed_mps")


def run(fmu_path, four_wheel_drive=False):
    md = read_model_description(fmu_path)
    variables = {v.name: v for v in md.modelVariables}
    vr = {name: v.valueReference for name, v in variables.items()}
    unzip = extract(fmu_path)
    fmu = FMU2Slave(guid=md.guid, unzipDirectory=unzip,
                     modelIdentifier=md.coSimulation.modelIdentifier, instanceName="validate")
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    if four_wheel_drive:
        # the pythonfmu FMU declares this as Boolean, the hand-rolled native
        # one only has Real variables at all -- set it with whichever setter
        # matches what this FMU actually declared, instead of hardcoding one.
        if variables["four_wheel_drive"].type == "Boolean":
            fmu.setBoolean([vr["four_wheel_drive"]], [True])
        else:
            fmu.setReal([vr["four_wheel_drive"]], [1.0])
    fmu.exitInitializationMode()

    t = 0.0
    while t < SIM_TIME:
        steer = STEER_DEG if t > STEER_START else 0.0
        front_torque = DRIVE_TORQUE_FRONT if four_wheel_drive else 0.0
        fmu.setReal([vr["steer_deg"], vr["drive_torque"], vr["drive_torque_front"]],
                    [steer, DRIVE_TORQUE, front_torque])
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=DT)
        t += DT

    result = fmu.getReal([vr[n] for n in OUTPUTS])
    fmu.terminate()
    fmu.freeInstance()
    return result


def compare(label, four_wheel_drive):
    py_result = run(PYTHONFMU_PATH, four_wheel_drive)
    native_result = run(NATIVE_PATH, four_wheel_drive)

    print(f"--- {label} ---")
    print(f"{'signal':>12} {'pythonfmu':>14} {'native C++':>14} {'abs diff':>14}")
    worst = 0.0
    for name, p, n in zip(OUTPUTS, py_result, native_result):
        diff = abs(p - n)
        worst = max(worst, diff)
        print(f"{name:>12} {p:14.8f} {n:14.8f} {diff:14.2e}")
    print()
    return worst


def main():
    for path in (PYTHONFMU_PATH, NATIVE_PATH):
        if not os.path.exists(path):
            raise SystemExit(f"{path} not found -- build it first, see this file's docstring")

    worst_2wd = compare("rear-only drive (default)", four_wheel_drive=False)
    worst_4wd = compare("four_wheel_drive=True (rear=260, front=150 N*m)", four_wheel_drive=True)
    worst = max(worst_2wd, worst_4wd)

    if worst > 1e-4:
        raise SystemExit(f"FAIL: worst diff {worst:.2e} exceeds tolerance -- "
                          f"the C++ port diverged from the pythonfmu reference")
    print(f"OK -- pythonfmu and native C++ agree in both configurations (worst diff {worst:.2e}), "
          f"confirming the C++ port (including the four_wheel_drive addition) is a faithful "
          f"translation of the same model.")


if __name__ == "__main__":
    main()
