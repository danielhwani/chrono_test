"""
Visual demo: one driven wheel loses traction, then the simple differential
(simple_vehicle.apply_differential) kicks in and recovers.

Uses the 6-wheel truck layout. The mid-right (MR) wheel gets a low-friction
contact material so it slips under drive torque. For the first SWITCH_TIME
seconds the differential is OFF (every driven wheel just gets the same fixed
torque, like the very first version of this model) so MR visibly spins much
faster than the other wheels. After that, the differential turns ON and
redistributes torque away from MR -- watch its spin (the yellow stripe marker
on each wheel) slow back down toward the others' speed.

The window title also prints live wheel spin rates and whether the
differential is currently active, in case the visual difference is subtle.

Run:
    python slip_demo.py
    python slip_demo.py --switch-time 4.0
"""
import argparse
import os

import pychrono as chrono
import pychrono.irrlicht as chronoirr

import simple_vehicle as sv

SLIP_WHEEL = "MR"
SLIP_FRICTION = 0.05
RENDER_DT = 1.0 / 60.0
CAMERA_OFFSET = chrono.ChVector3d(-3.2, -4.2, 1.8)  # close-in, so wheel spin is easy to see


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--switch-time", type=float, default=4.0,
                         help="sim time [s] at which the differential turns on")
    parser.add_argument("--time", type=float, default=10.0, help="total sim time [s]")
    args = parser.parse_args()

    chrono.SetChronoDataPath(
        os.environ.get("CHRONO_DATA_DIR", chrono.GetChronoDataPath())
    )

    sys_ = chrono.ChSystemNSC()
    sys_.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys_.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, sv.G))
    sys_.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys_.GetSolver().AsIterative().SetMaxIterations(150)

    chassis, wheels, springs, motors, steer_functions, throttle_functions = sv.make_vehicle(
        sys_, six_wheel=True
    )

    low_mat = chrono.ChContactMaterialNSC()
    low_mat.SetFriction(SLIP_FRICTION)
    wheels[SLIP_WHEEL].GetCollisionModel().SetAllShapesMaterial(low_mat)
    wheels[SLIP_WHEEL].GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.1, 0.1))

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.AttachSystem(sys_)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Slip demo")
    vis.Initialize()
    vis.ShowExplorer(False)
    vis.AddSkyBox()
    vis.AddCamera(CAMERA_OFFSET, chrono.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()

    print(f"Wheel '{SLIP_WHEEL}' (shown in red) has low friction (mu={SLIP_FRICTION}).")
    print(f"Differential is OFF for the first {args.switch_time:.1f}s, then turns ON.")
    print("Watch its yellow spin marker: fast/blurred while diff is off, "
          "settling back down once the differential engages.")

    next_render_t = 0.0
    next_title_t = 0.0
    realtime_timer = chrono.ChRealtimeStepTimer()

    t = 0.0
    while t < args.time:
        if not vis.Run():
            break

        diff_on = t >= args.switch_time
        if diff_on:
            sv.apply_differential(motors, throttle_functions, sv.DRIVE_TORQUE)
        else:
            for fn in throttle_functions.values():
                fn.SetConstant(sv.DRIVE_TORQUE)

        if t >= next_render_t:
            vis.ShowExplorer(False)
            chassis_pos = chassis.GetPos()
            vis.SetCameraTarget(chassis_pos)
            vis.SetCameraPosition(chassis_pos + CAMERA_OFFSET)
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            next_render_t += RENDER_DT

        if t >= next_title_t:
            # NOTE: SetWindowTitle() only takes effect once, before the render
            # loop starts -- calling it again here does not update the actual
            # window title, so live status is printed to the terminal instead.
            speeds = " ".join(f"{k}={m.GetMotorAngleDt():+6.0f}" for k, m in sorted(motors.items()))
            print(f"\rdiff={'ON ' if diff_on else 'OFF'} | {speeds} rad/s   ", end="", flush=True)
            next_title_t += 0.2

        sys_.DoStepDynamics(sv.TIME_STEP)
        t = sys_.GetChTime()
        realtime_timer.Spin(sv.TIME_STEP)

    print("\nDone.")


if __name__ == "__main__":
    main()
