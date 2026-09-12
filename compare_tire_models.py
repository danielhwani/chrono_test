"""
Run the same scripted steering maneuver twice -- once with the rigid
(Coulomb-friction) wheel/ground contact, once with the empirical slip-based
tire model -- and overlay the results in one plot.

Reuses simple_vehicle.make_vehicle/apply_differential/apply_tire_forces/
steer_angle_deg/ackermann_wheel_angles_deg directly (headless, no CSV
round-trip, doesn't touch your vehicle_log.csv), so both runs share
identical setup except for --tire-model.

Run:
    python compare_tire_models.py
    python compare_tire_models.py --steer-deg 30 --steer-start 1.5 --steer-ramp 0.8 --time 6
    python compare_tire_models.py --six-wheel --ackermann
"""
import argparse
import math

import matplotlib.pyplot as plt
import numpy as np
import pychrono as chrono

import simple_vehicle as sv

COLORS = {"rigid": "tab:blue", "empirical": "tab:orange"}


def run_simulation(tire_model, args):
    sys_ = chrono.ChSystemNSC()
    sys_.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys_.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, sv.G))
    sys_.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys_.GetSolver().AsIterative().SetMaxIterations(150)

    chassis, wheels, springs, motors, steer_functions, throttle_functions = sv.make_vehicle(
        sys_, six_wheel=args.six_wheel
    )
    ackermann_wheelbase = sv.WHEELBASE_6W if args.six_wheel else sv.WHEELBASE
    tire_accumulators = (
        sv.setup_empirical_tire_wheels(wheels) if tire_model == "empirical" else None
    )

    log = {"time": [], "x": [], "y": [], "yaw_deg": [], "roll_deg": [], "speed_mps": []}

    t = 0.0
    while t < args.time:
        cur_steer_deg = sv.steer_angle_deg(t, args.steer_deg, args.steer_start, args.steer_ramp)
        if args.ackermann:
            left_deg, right_deg = sv.ackermann_wheel_angles_deg(
                cur_steer_deg, ackermann_wheelbase, sv.TRACK
            )
            if "FL" in steer_functions:
                steer_functions["FL"].SetConstant(math.radians(left_deg))
            if "FR" in steer_functions:
                steer_functions["FR"].SetConstant(math.radians(right_deg))
        else:
            cur_steer_rad = math.radians(cur_steer_deg)
            for fn in steer_functions.values():
                fn.SetConstant(cur_steer_rad)

        sv.apply_differential(motors, throttle_functions, sv.DRIVE_TORQUE)
        if tire_accumulators is not None:
            sv.apply_tire_forces(wheels, tire_accumulators)

        sys_.DoStepDynamics(sv.TIME_STEP)
        t = sys_.GetChTime()

        pos = chassis.GetPos()
        vel = chassis.GetPosDt()
        euler = chassis.GetRot().GetCardanAnglesXYZ()
        log["time"].append(t)
        log["x"].append(pos.x)
        log["y"].append(pos.y)
        log["yaw_deg"].append(math.degrees(euler.z))
        log["roll_deg"].append(math.degrees(euler.x))
        log["speed_mps"].append(math.hypot(vel.x, vel.y))

    return log


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--time", type=float, default=sv.SIM_TIME)
    parser.add_argument("--steer-deg", type=float, default=sv.STEER_DEG_DEFAULT)
    parser.add_argument("--steer-start", type=float, default=sv.STEER_START_DEFAULT)
    parser.add_argument("--steer-ramp", type=float, default=sv.STEER_RAMP_DEFAULT)
    parser.add_argument("--six-wheel", action="store_true")
    parser.add_argument("--ackermann", action="store_true")
    args = parser.parse_args()

    results = {}
    for tire_model in ("rigid", "empirical"):
        print(f"Running tire-model={tire_model} ...")
        results[tire_model] = run_simulation(tire_model, args)

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    ax = axes[0, 0]
    for mode, log in results.items():
        ax.plot(log["x"], log["y"], label=mode, color=COLORS[mode])
    ax.set_title("Top-down trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()

    ax = axes[0, 1]
    for mode, log in results.items():
        ax.plot(log["time"], log["speed_mps"], label=mode, color=COLORS[mode])
    ax.set_title("Forward speed")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed [m/s]")
    ax.grid(True)
    ax.legend()

    ax = axes[1, 0]
    for mode, log in results.items():
        yaw_unwrapped = np.degrees(np.unwrap(np.radians(log["yaw_deg"])))
        ax.plot(log["time"], yaw_unwrapped, label=mode, color=COLORS[mode])
    ax.set_title("Chassis yaw (unwrapped)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("deg")
    ax.grid(True)
    ax.legend()

    ax = axes[1, 1]
    for mode, log in results.items():
        ax.plot(log["time"], log["roll_deg"], label=mode, color=COLORS[mode])
    ax.set_title("Chassis roll (stick-slip chatter check)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("deg")
    ax.grid(True)
    ax.legend()

    fig.tight_layout()
    out_path = "tire_model_compare.png"
    fig.savefig(out_path, dpi=130)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
