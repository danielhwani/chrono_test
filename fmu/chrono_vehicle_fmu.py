"""
The real vehicle (`make_vehicle()` in simple_vehicle.py), wrapped as an FMI2
Co-Simulation slave via pythonfmu -- the first FMU in this repo backed by
something other than a toy bouncing-ball model. Everything upstream of this
(pythonfmu itself, the native C/C++ FMI2 paths, the Modelica interop, the
real-time pacing/jitter tooling) was validated against bouncing balls
specifically so this step would just be "point the same machinery at the
real model" instead of debugging the toolchain and the vehicle at once.

Structural configuration (axle layout, steering geometry, tire model,
terrain) has to be fixed at build/instantiation time, before the Chrono
system is constructed -- these become FMI2 `parameter` variables (settable
only during initialization, mirroring simple_vehicle.py's CLI flags):
    six_wheel        bool   4-wheel car (false, default) vs 6-wheel truck
    ackermann        bool   parallel (false) vs Ackermann-geometry steering
    empirical_tire   bool   rigid Coulomb contact (false) vs slip-based tire
    bumps_terrain    bool   flat ground (false) vs a row of speed bumps
    four_wheel_drive bool   rear-only drive (false, default) vs also driving
                            the front (steered) axle -- adding a drive motor
                            to an already-steered corner is exactly how a
                            real CV-jointed front driveshaft works, so this
                            doesn't conflict with steering at all

Runtime control is `input` variables the master drives every step (replacing
simple_vehicle.py main()'s own steer_angle_deg() ramp and fixed DRIVE_TORQUE
constant, which were conveniences for the standalone demo -- a real FMI
slave should let the master command these directly):
    steer_deg          commanded front-wheel steer angle, degrees (applies to
                        both front wheels alike, or split via ackermann)
    drive_torque_rear  nominal rear-axle drive torque, N*m (named
                        "..._rear", not bare "drive_torque", once
                        drive_torque_front/_mid existed alongside it --
                        three siblings named the same way beats one
                        being the odd one out)
    drive_torque_front  nominal front-axle drive torque, N*m (only takes
                        effect when four_wheel_drive is true; default 0.0
                        is physically equivalent to no front motor at all,
                        so leaving it unset is backward compatible)
    drive_torque_mid    nominal mid-axle drive torque, N*m -- only exists
                        (six_wheel's mid axle is always driven, unlike the
                        front) when six_wheel is true; default equals
                        DRIVE_TORQUE, matching the value the mid axle
                        already got before this input existed, so leaving
                        it unset is backward compatible. This is the same
                        "each axle controlled independently" pattern as
                        drive_torque_front, extended from 2 axles to 3 for
                        six_wheel -- the ros2_control controller still only
                        ever sends one traction reference; independent
                        per-axle control is a model/FMU-level capability,
                        not something ros2_control's standard steering
                        controllers expose on their own.

Outputs mirror vehicle_log.csv's columns (position/orientation/speed/actual
steer angles), so this can be validated against the exact same signals the
standalone script already logs.

Build (needs simple_vehicle.py bundled in as a "project file" so it's
importable from inside the FMU's own resources/ directory at runtime):
    cd fmu
    pythonfmu build -f chrono_vehicle_fmu.py -d build ../simple_vehicle.py
"""
import math
import sys
import os

from pythonfmu import Fmi2Slave, Fmi2Causality, Fmi2Variability, Real, Boolean

# pythonfmu bundles ../simple_vehicle.py (passed as a project file at build
# time) into the FMU's resources/ next to this script, so it's importable
# here as a plain top-level module once do_step actually runs inside the FMU
# (resources/ == dirname(__file__) at that point). But pythonfmu's builder
# also imports this module directly from its original location to introspect
# the Fmi2Slave subclass *before* packaging, when simple_vehicle.py is still
# one directory up (repo root) instead of alongside -- so both dirs need to
# be on sys.path for the import to work in both phases.
_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _here)
sys.path.insert(0, os.path.dirname(_here))

import pychrono as chrono
from simple_vehicle import (
    G,
    TRACK,
    WHEELBASE,
    WHEELBASE_6W,
    DRIVE_TORQUE,
    make_vehicle,
    apply_differential,
    setup_empirical_tire_wheels,
    apply_tire_forces,
    ackermann_wheel_angles_deg,
)


class ChronoVehicle(Fmi2Slave):
    author = "chrono_test"
    description = "Simple Chrono multibody vehicle (simple_vehicle.make_vehicle) as an FMI2 CS slave"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # ---- structural parameters: fixed for the whole simulation, read
        # in exit_initialization_mode() when the Chrono system is built ----
        self.six_wheel = False
        self.ackermann = False
        self.empirical_tire = False
        self.bumps_terrain = False
        self.four_wheel_drive = False

        # ---- runtime inputs ----
        self.steer_deg = 0.0
        self.drive_torque_rear = DRIVE_TORQUE
        self.drive_torque_front = 0.0
        self.drive_torque_mid = DRIVE_TORQUE

        # ---- outputs ----
        self.chassis_x = 0.0
        self.chassis_y = 0.0
        self.chassis_z = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.speed_mps = 0.0
        self.steer_FL_deg = 0.0
        self.steer_FR_deg = 0.0

        for name in ("six_wheel", "ackermann", "empirical_tire", "bumps_terrain", "four_wheel_drive"):
            self.register_variable(
                Boolean(name, causality=Fmi2Causality.parameter, variability=Fmi2Variability.fixed)
            )
        self.register_variable(
            Real("steer_deg", causality=Fmi2Causality.input, variability=Fmi2Variability.continuous)
        )
        self.register_variable(
            Real("drive_torque_rear", causality=Fmi2Causality.input, variability=Fmi2Variability.continuous)
        )
        self.register_variable(
            Real("drive_torque_front", causality=Fmi2Causality.input, variability=Fmi2Variability.continuous)
        )
        self.register_variable(
            Real("drive_torque_mid", causality=Fmi2Causality.input, variability=Fmi2Variability.continuous)
        )
        for name in ("chassis_x", "chassis_y", "chassis_z", "roll_deg", "pitch_deg", "yaw_deg",
                     "speed_mps", "steer_FL_deg", "steer_FR_deg"):
            self.register_variable(
                Real(name, causality=Fmi2Causality.output, variability=Fmi2Variability.continuous)
            )

        self._sys = None
        self._chassis = None
        self._wheels = None
        self._motors = None
        self._steer_functions = None
        self._throttle_functions = None
        self._front_motors = None
        self._front_throttle_functions = None
        self._mid_motors = None
        self._mid_throttle_functions = None
        self._rear_motors = None
        self._rear_throttle_functions = None
        self._tire_accumulators = None
        self._wheelbase = WHEELBASE

    def exit_initialization_mode(self):
        self._sys = chrono.ChSystemNSC()
        self._sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        self._sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, G))
        self._sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
        self._sys.GetSolver().AsIterative().SetMaxIterations(150)

        terrain = "bumps" if self.bumps_terrain else "flat"
        (self._chassis, self._wheels, _springs, self._motors,
         self._steer_functions, self._throttle_functions) = make_vehicle(
            self._sys, six_wheel=self.six_wheel, terrain=terrain,
            four_wheel_drive=self.four_wheel_drive,
        )
        self._wheelbase = WHEELBASE_6W if self.six_wheel else WHEELBASE
        self._tire_accumulators = (
            setup_empirical_tire_wheels(self._wheels) if self.empirical_tire else None
        )

        # Split the driven corners by axle prefix (F/M/R) so do_step() can
        # apply drive_torque_front/drive_torque_mid/drive_torque_rear
        # independently per axle instead of one shared value for everything
        # -- F only exists as a driven group when four_wheel_drive is set, M
        # only exists at all when six_wheel is set (and is always driven
        # when present). Each dict is simply empty when that axle doesn't
        # apply, and apply_differential() on an empty dict is a harmless
        # no-op, so this works unchanged for every axle-layout combination.
        def _group(prefix):
            motors = {k: v for k, v in self._motors.items() if k.startswith(prefix)}
            throttle = {k: v for k, v in self._throttle_functions.items() if k.startswith(prefix)}
            return motors, throttle

        self._front_motors, self._front_throttle_functions = _group("F")
        self._mid_motors, self._mid_throttle_functions = _group("M")
        self._rear_motors, self._rear_throttle_functions = _group("R")

    def do_step(self, current_time: float, step_size: float) -> bool:
        if self.ackermann:
            left_deg, right_deg = ackermann_wheel_angles_deg(self.steer_deg, self._wheelbase, TRACK)
            if "FL" in self._steer_functions:
                self._steer_functions["FL"].SetConstant(math.radians(left_deg))
            if "FR" in self._steer_functions:
                self._steer_functions["FR"].SetConstant(math.radians(right_deg))
        else:
            cur_steer_rad = math.radians(self.steer_deg)
            for fn in self._steer_functions.values():
                fn.SetConstant(cur_steer_rad)

        apply_differential(self._rear_motors, self._rear_throttle_functions, self.drive_torque_rear)
        apply_differential(self._mid_motors, self._mid_throttle_functions, self.drive_torque_mid)
        apply_differential(self._front_motors, self._front_throttle_functions, self.drive_torque_front)
        if self._tire_accumulators is not None:
            apply_tire_forces(self._wheels, self._tire_accumulators)

        self._sys.DoStepDynamics(step_size)

        rot = self._chassis.GetRot()
        euler = rot.GetCardanAnglesXYZ()
        self.roll_deg = math.degrees(euler.x)
        self.pitch_deg = math.degrees(euler.y)
        self.yaw_deg = math.degrees(euler.z)

        pos = self._chassis.GetPos()
        self.chassis_x, self.chassis_y, self.chassis_z = pos.x, pos.y, pos.z

        vel = self._chassis.GetPosDt()
        self.speed_mps = math.sqrt(vel.x ** 2 + vel.y ** 2)

        self.steer_FL_deg = (
            math.degrees(self._steer_functions["FL"].GetVal(0)) if "FL" in self._steer_functions else 0.0
        )
        self.steer_FR_deg = (
            math.degrees(self._steer_functions["FR"].GetVal(0)) if "FR" in self._steer_functions else 0.0
        )
        return True
