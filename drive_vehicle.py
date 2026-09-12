"""
Interactive keyboard-controlled version of the simple Chrono vehicle.

Controls (work regardless of which window has focus -- the 3D view or
the terminal -- since keys are captured globally from the X server):
    Up / W      - accelerate forward
    Down / S    - accelerate backward (reverse)
    Left / A    - steer left
    Right / D   - steer right
    Space       - stop (cancels throttle and instantly zeroes velocity)
    q / Esc     - quit

Uses pynput (a global X11 keyboard listener) instead of reading the
terminal's stdin, because stdin only receives keystrokes while the
terminal window itself has input focus -- and the newly opened Irrlicht
3D window normally steals that focus, so a terminal-based reader never
sees anything you type. pynput listens at the X server level, so it
keeps working no matter which window is focused.

Reuses the vehicle/ground construction from simple_vehicle.make_vehicle();
only the drive-torque and steer-angle targets are updated every frame
from live keyboard state instead of a scripted profile.

Requires: pip install pynput   (needs a real X11 session; DISPLAY must be set)

Run:
    python drive_vehicle.py
    python drive_vehicle.py --six-wheel   # 3-axle truck instead of the 4-wheel car
    python drive_vehicle.py --ackermann   # proper L/R Ackermann steer angles
"""
import argparse
import math
import os

import pychrono as chrono
import pychrono.irrlicht as chronoirr
from pynput import keyboard

import simple_vehicle as sv

STEER_MAX_DEG = 25.0
STEER_RATE_DEG_S = 90.0        # how fast the steer angle ramps to its target
TORQUE_RATE_NM_S = 900.0       # how fast drive torque ramps to its target

RENDER_DT = 1.0 / 60.0


def ramp_toward(current, target, rate, dt):
    if current < target:
        return min(current + rate * dt, target)
    if current > target:
        return max(current - rate * dt, target)
    return target


class GlobalKeyState:
    """Tracks which control keys are currently held, via a global X11 listener."""

    CHAR_MAP = {"w": "up", "s": "down", "a": "left", "d": "right"}

    def __init__(self):
        self.held = {"up": False, "down": False, "left": False, "right": False}
        self.stop_requested = False
        self.quit_requested = False
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)

    def _key_name(self, key):
        if key in (keyboard.Key.up,):
            return "up"
        if key in (keyboard.Key.down,):
            return "down"
        if key in (keyboard.Key.left,):
            return "left"
        if key in (keyboard.Key.right,):
            return "right"
        if hasattr(key, "char") and key.char is not None:
            return self.CHAR_MAP.get(key.char.lower())
        return None

    def _on_press(self, key):
        name = self._key_name(key)
        if name is not None:
            self.held[name] = True
            return
        if key == keyboard.Key.space:
            self.stop_requested = True
        elif key == keyboard.Key.esc or (hasattr(key, "char") and key.char in ("q", "Q")):
            self.quit_requested = True

    def _on_release(self, key):
        name = self._key_name(key)
        if name is not None:
            self.held[name] = False

    def start(self):
        self.listener.start()

    def stop(self):
        self.listener.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--six-wheel", action="store_true",
                         help="use the 3-axle (6-wheel) truck layout instead of the 4-wheel car")
    parser.add_argument("--ackermann", action="store_true",
                         help="split the steer command into separate L/R wheel angles via "
                              "Ackermann geometry, instead of applying it to both equally")
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
        sys_, six_wheel=args.six_wheel
    )

    cam_offset = (
        chrono.ChVector3d(-9, -12, 5.0) if args.six_wheel else chrono.ChVector3d(-6, -8, 3.5)
    )
    ackermann_wheelbase = sv.WHEELBASE_6W if args.six_wheel else sv.WHEELBASE

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.AttachSystem(sys_)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Drive the Chrono Vehicle")
    vis.Initialize()
    vis.ShowExplorer(False)
    vis.AddSkyBox()
    vis.AddCamera(cam_offset, chrono.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()

    print(f"Steering mode: {'ACKERMANN' if args.ackermann else 'PARALLEL'}"
          f"{' (--ackermann)' if args.ackermann else ' (no --ackermann flag)'}")
    print("Controls (work no matter which window is focused):")
    print("  Up/W = forward   Down/S = reverse   Left/A = steer left   Right/D = steer right")
    print("  Space = stop   q/Esc = quit")
    print("Live status is printed below (throttle / FL / FR / speed):")

    keys = GlobalKeyState()
    keys.start()

    throttle_cmd = 0.0
    steer_cmd_deg = 0.0
    next_render_t = 0.0
    next_title_t = 0.0
    realtime_timer = chrono.ChRealtimeStepTimer()

    try:
        while vis.Run():
            if keys.stop_requested:
                keys.stop_requested = False
                throttle_cmd = 0.0
                for wheel in wheels.values():
                    wheel.SetPosDt(chrono.VNULL)
                    wheel.SetAngVelParent(chrono.VNULL)
                chassis.SetPosDt(chrono.VNULL)
                chassis.SetAngVelParent(chrono.VNULL)
            if keys.quit_requested:
                break

            want_throttle = 1.0 if keys.held["up"] else (-1.0 if keys.held["down"] else 0.0)
            want_steer = 1.0 if keys.held["right"] else (-1.0 if keys.held["left"] else 0.0)

            throttle_cmd = ramp_toward(
                throttle_cmd, want_throttle * sv.DRIVE_TORQUE, TORQUE_RATE_NM_S, sv.TIME_STEP
            )
            steer_cmd_deg = ramp_toward(
                steer_cmd_deg, want_steer * STEER_MAX_DEG, STEER_RATE_DEG_S, sv.TIME_STEP
            )

            sv.apply_differential(motors, throttle_functions, throttle_cmd)
            if args.ackermann:
                left_deg, right_deg = sv.ackermann_wheel_angles_deg(
                    steer_cmd_deg, ackermann_wheelbase, sv.TRACK
                )
                if "FL" in steer_functions:
                    steer_functions["FL"].SetConstant(math.radians(left_deg))
                if "FR" in steer_functions:
                    steer_functions["FR"].SetConstant(math.radians(right_deg))
            else:
                for fn in steer_functions.values():
                    fn.SetConstant(math.radians(steer_cmd_deg))

            t = sys_.GetChTime()
            if t >= next_render_t:
                chassis_pos = chassis.GetPos()
                vis.SetCameraTarget(chassis_pos)
                vis.SetCameraPosition(chassis_pos + cam_offset)
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                next_render_t += RENDER_DT

            if t >= next_title_t:
                # NOTE: vis.SetWindowTitle() only takes effect once, before the
                # render loop starts -- calling it again here does NOT update
                # the actual window title (verified: the OS-level title stays
                # frozen at whatever it was after Initialize()). So live status
                # is printed to the terminal instead, which reliably updates.
                speed = chassis.GetPosDt().Length()
                fl_deg = math.degrees(steer_functions["FL"].GetVal(0)) if "FL" in steer_functions else 0.0
                fr_deg = math.degrees(steer_functions["FR"].GetVal(0)) if "FR" in steer_functions else 0.0
                mode = "ACKERMANN" if args.ackermann else "PARALLEL"
                print(
                    f"\r[{mode}] throttle={throttle_cmd:+7.1f} Nm  "
                    f"FL={fl_deg:+6.2f} deg  FR={fr_deg:+6.2f} deg  speed={speed:5.2f} m/s   ",
                    end="", flush=True,
                )
                next_title_t += 0.2

            sys_.DoStepDynamics(sv.TIME_STEP)
            realtime_timer.Spin(sv.TIME_STEP)
    finally:
        keys.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()
