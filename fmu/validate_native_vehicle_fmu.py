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
DRIVE_TORQUE_MID = 200.0
OUTPUTS = ("chassis_x", "chassis_y", "chassis_z", "yaw_deg", "speed_mps")


def _set_bool_or_real(fmu, variables, vr, name, value):
    # the pythonfmu FMU declares these as Boolean, the hand-rolled native one
    # only has Real variables at all -- set each with whichever setter
    # matches what this FMU actually declared, instead of hardcoding one.
    if variables[name].type == "Boolean":
        fmu.setBoolean([vr[name]], [bool(value)])
    else:
        fmu.setReal([vr[name]], [1.0 if value else 0.0])


def run(fmu_path, four_wheel_drive=False, six_wheel=False):
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
        _set_bool_or_real(fmu, variables, vr, "four_wheel_drive", True)
    if six_wheel:
        _set_bool_or_real(fmu, variables, vr, "six_wheel", True)
    fmu.exitInitializationMode()

    t = 0.0
    while t < SIM_TIME:
        steer = STEER_DEG if t > STEER_START else 0.0
        front_torque = DRIVE_TORQUE_FRONT if four_wheel_drive else 0.0
        mid_torque = DRIVE_TORQUE_MID if six_wheel else DRIVE_TORQUE
        fmu.setReal([vr["steer_deg"], vr["drive_torque_rear"], vr["drive_torque_front"], vr["drive_torque_mid"]],
                    [steer, DRIVE_TORQUE, front_torque, mid_torque])
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=DT)
        t += DT

    result = fmu.getReal([vr[n] for n in OUTPUTS])
    fmu.terminate()
    fmu.freeInstance()
    return result


def compare(label, **kwargs):
    py_result = run(PYTHONFMU_PATH, **kwargs)
    native_result = run(NATIVE_PATH, **kwargs)

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

    worst_2wd = compare("4-wheel, rear-only drive (default)", four_wheel_drive=False, six_wheel=False)
    worst_4wd = compare("4-wheel, four_wheel_drive=True (rear=260, front=150 N*m)",
                         four_wheel_drive=True, six_wheel=False)
    worst_6w = compare("6-wheel truck, mid+rear driven (default drive_torque_mid)",
                        four_wheel_drive=False, six_wheel=True)
    worst_6x6 = compare("6x6 (six_wheel + four_wheel_drive, rear=260 mid=200 front=150 N*m)",
                         four_wheel_drive=True, six_wheel=True)
    worst = max(worst_2wd, worst_4wd, worst_6w, worst_6x6)

    if worst > 1e-4:
        raise SystemExit(f"FAIL: worst diff {worst:.2e} exceeds tolerance -- "
                          f"the C++ port diverged from the pythonfmu reference")
    print(f"OK -- pythonfmu and native C++ agree in all four configurations (worst diff {worst:.2e}), "
          f"confirming the C++ port (four_wheel_drive and six_wheel both) is a faithful "
          f"translation of the same model.")


if __name__ == "__main__":
    main()
