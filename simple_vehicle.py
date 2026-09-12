"""
Simple vehicle multibody dynamics demo using PyChrono core API
(no Chrono::Vehicle module) -- built directly from ChBody + ChLink primitives.

Two axle layouts share the same construction code (make_vehicle):
  - 4-wheel car (default): front axle steers, rear axle drives.
  - 6-wheel truck (--six-wheel): front axle steers, mid + rear axles drive.

Model (per wheel corner):
  - An upright connects to the chassis via a vertical ChLinkLockPrismatic
    (suspension travel) + a ChLinkTSDA (spring-damper) acting in parallel.
  - Steered axles add a steering knuckle: upright -> knuckle via a
    ChLinkMotorRotationAngle rotating about the vertical (Z) axis, driven by
    a step-steer profile (straight, then ramp to a target angle and hold).
  - The wheel (a cylinder) attaches to the upright/knuckle via a
    ChLinkLockRevolute (spin axis), so it free-rotates.
  - Driven axles get an applied spin torque (via ChLinkMotorRotationTorque).
  - Ground: a large box with a friction/contact material.

By default both front wheels get the exact same steer angle ("parallel"
steering). --ackermann instead splits it into separate L/R angles via
ackermann_wheel_angles_deg() -- a pure steering-input transform layered on
top of the same steer_functions the model already exposes, so the vehicle
assembly itself (make_vehicle) is unchanged either way.

Run:
    python simple_vehicle.py              # headless, logs CSV + shows plots at end
    python simple_vehicle.py --irrlicht    # also opens a live 3D view (needs display)
    python simple_vehicle.py --six-wheel --irrlicht
    python simple_vehicle.py --ackermann --irrlicht
    python simple_vehicle.py --steer-deg 25 --steer-start 1.5 --steer-ramp 0.8
"""
import argparse
import csv
import math
import os

import pychrono as chrono


# ----------------------------- parameters -----------------------------
G = -9.81

CHASSIS_MASS = 1200.0
CHASSIS_DIMS = (2.6, 1.6, 0.4)   # length(x), width(y), height(z)
CHASSIS_CG_HEIGHT = 0.55         # chassis CG height above ground at rest

# six-wheel (3-axle truck) variant: front axle steers, mid+rear axles drive
CHASSIS_MASS_6W = 2600.0
CHASSIS_DIMS_6W = (4.4, 1.8, 0.5)
WHEELBASE_6W = 3.4               # front-to-rear axle distance; mid axle is centered

WHEEL_MASS = 18.0
WHEEL_RADIUS = 0.32
WHEEL_WIDTH = 0.22

UPRIGHT_MASS = 15.0
KNUCKLE_MASS = 3.0

WHEELBASE = 2.6
TRACK = 1.5

SPRING_K = 35000.0      # N/m
DAMPER_C = 3500.0       # N*(m/s)
SPRING_FREE_LENGTH = 0.42
SUSPENSION_TRAVEL_REST = 0.32  # upright resting offset below chassis mount

DRIVE_TORQUE = 260.0    # N*m applied to each rear wheel
SIM_TIME = 6.0
TIME_STEP = 2e-3

# "empirical" tire model (--tire-model empirical): a simplified, hand-rolled
# slip-based tire force law (conceptually similar to TMEASY -- linear near
# zero slip, saturating at a friction-circle limit) applied as an external
# force on each wheel, in place of Chrono's rigid Coulomb contact friction.
TIRE_MU = 0.9                     # peak friction coefficient (peak Fx or Fy = mu * Fz)
TIRE_CORNERING_STIFFNESS = 12.0   # N of lateral force per N of load per radian of slip angle
TIRE_LONGITUDINAL_STIFFNESS = 15.0  # N of longitudinal force per N of load per unit slip ratio
TIRE_SLIP_SPEED_EPS = 0.5         # m/s -- floor on the speed used to normalize slip ratio

# simple open/limited-slip differential: senses the L/R wheel speed
# difference on each driven axle and shifts torque toward the slower
# (higher-traction) side.
DIFF_GAIN = 40.0             # N*m of torque shift per (rad/s) of speed difference
DIFF_MAX_BIAS_FRACTION = 0.9  # cap the shift at this fraction of the nominal torque

GROUND_SIZE = 500.0      # flat square terrain, [m] per side, centered at origin

STEER_DEG_DEFAULT = 20.0    # target steer angle (both front wheels, parallel)
STEER_START_DEFAULT = 2.0   # sim time [s] when steering begins
STEER_RAMP_DEFAULT = 1.0    # ramp duration [s] to reach the target angle


def steer_angle_deg(t, target_deg, start, ramp):
    if t <= start:
        return 0.0
    if t >= start + ramp:
        return target_deg
    return target_deg * (t - start) / ramp


def ackermann_wheel_angles_deg(steer_deg, wheelbase, track):
    """Convert a single nominal ("bicycle model") front-steer angle into the
    separate left/right wheel angles that satisfy Ackermann geometry: both
    front wheels' steer axes then point at the same turn center (on the
    extended rear-axle line), so neither tire has to scrub sideways.

    steer_deg follows the existing sign convention (positive = turn right).
    On a right turn the right wheel is on the inside of the turn and gets
    the larger magnitude angle; on a left turn it's the opposite. Returns
    (left_deg, right_deg). Purely a steering-input transform -- it does not
    change the vehicle model, only how a commanded angle is split L/R.
    """
    if abs(steer_deg) < 1e-6:
        return 0.0, 0.0
    turn_radius = wheelbase / math.tan(math.radians(abs(steer_deg)))
    inner_deg = math.degrees(math.atan(wheelbase / (turn_radius - track / 2)))
    outer_deg = math.degrees(math.atan(wheelbase / (turn_radius + track / 2)))
    sign = 1.0 if steer_deg > 0 else -1.0
    if steer_deg > 0:  # turning right: right wheel is on the inside
        return sign * outer_deg, sign * inner_deg
    return sign * inner_deg, sign * outer_deg  # turning left: left wheel is on the inside


def make_vehicle(sys: chrono.ChSystem, six_wheel: bool = False):
    contact_method = sys.GetContactMethod()
    if contact_method == chrono.ChContactMethod_NSC:
        mat = chrono.ChContactMaterialNSC()
        mat.SetFriction(0.9)
        mat.SetRestitution(0.0)
    else:
        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(0.9)
        mat.SetRestitution(0.05)
        mat.SetYoungModulus(2e7)

    # ---- ground ----
    ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, 0.5, 1000, True, True, mat)
    ground.SetPos(chrono.ChVector3d(0, 0, -0.25))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.55, 0.4))
    ground.GetVisualShape(0).SetTexture(
        os.path.join(chrono.GetChronoDataPath(), "textures", "concrete.jpg"),
        GROUND_SIZE / 5, GROUND_SIZE / 5,
    )
    sys.Add(ground)

    # ---- chassis ----
    chassis_dims = CHASSIS_DIMS_6W if six_wheel else CHASSIS_DIMS
    chassis_mass = CHASSIS_MASS_6W if six_wheel else CHASSIS_MASS
    chassis = chrono.ChBodyEasyBox(*chassis_dims, 500, True, False)
    chassis.SetMass(chassis_mass)
    chassis.SetInertiaXX(chrono.ChVector3d(
        chassis_mass * (chassis_dims[1] ** 2 + chassis_dims[2] ** 2) / 12,
        chassis_mass * (chassis_dims[0] ** 2 + chassis_dims[2] ** 2) / 12,
        chassis_mass * (chassis_dims[0] ** 2 + chassis_dims[1] ** 2) / 12,
    ))
    chassis_z = WHEEL_RADIUS + SUSPENSION_TRAVEL_REST + CHASSIS_CG_HEIGHT
    chassis.SetPos(chrono.ChVector3d(0, 0, chassis_z))
    chassis.GetVisualShape(0).SetColor(
        chrono.ChColor(0.1, 0.35, 0.75) if six_wheel else chrono.ChColor(0.75, 0.1, 0.1)
    )
    sys.Add(chassis)

    # axles: (name prefix, x position, is_steered, is_driven)
    if six_wheel:
        axles = [
            ("F", WHEELBASE_6W / 2, True, False),
            ("M", 0.0, False, True),
            ("R", -WHEELBASE_6W / 2, False, True),
        ]
    else:
        axles = [
            ("F", WHEELBASE / 2, True, False),
            ("R", -WHEELBASE / 2, False, True),
        ]

    corners = {}
    for prefix, x, is_steered, is_driven in axles:
        corners[f"{prefix}L"] = (x, TRACK / 2, is_steered, is_driven)
        corners[f"{prefix}R"] = (x, -TRACK / 2, is_steered, is_driven)

    wheels = {}
    motors = {}
    springs = {}
    steer_functions = {}
    throttle_functions = {}

    for name, (x, y, is_steered, is_driven) in corners.items():
        mount_pos = chrono.ChVector3d(x, y, chassis_z)
        upright_z = WHEEL_RADIUS
        upright_pos = chrono.ChVector3d(x, y, upright_z)

        # ---- upright (unsprung mass, slides vertically vs chassis) ----
        upright = chrono.ChBody()
        upright.SetMass(UPRIGHT_MASS)
        upright.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
        upright.SetPos(upright_pos)
        sys.Add(upright)

        # vertical prismatic joint: upright translates along chassis Z axis
        prismatic = chrono.ChLinkLockPrismatic()
        frame = chrono.ChFramed(mount_pos, chrono.QuatFromAngleY(chrono.CH_PI_2))
        prismatic.Initialize(chassis, upright, frame)
        sys.Add(prismatic)

        # spring-damper (TSDA) between chassis mount and upright
        tsda = chrono.ChLinkTSDA()
        tsda.Initialize(chassis, upright, False, mount_pos, upright_pos)
        tsda.SetSpringCoefficient(SPRING_K)
        tsda.SetDampingCoefficient(DAMPER_C)
        tsda.SetRestLength((mount_pos - upright_pos).Length())
        tsda.AddVisualShape(chrono.ChVisualShapeSpring(0.06, 80))
        sys.Add(tsda)
        springs[name] = tsda

        # ---- steering knuckle (steered axles only): rotates about vertical
        # (Z) axis relative to the upright; the wheel's spin joint is
        # attached to the knuckle, so turning the knuckle turns the wheel.
        spin_parent = upright
        if is_steered:
            knuckle = chrono.ChBody()
            knuckle.SetMass(KNUCKLE_MASS)
            knuckle.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
            knuckle.SetPos(upright_pos)
            sys.Add(knuckle)

            steer_joint = chrono.ChLinkMotorRotationAngle()
            steer_frame = chrono.ChFramed(upright_pos, chrono.QUNIT)  # local Z = world Z (steer axis)
            steer_joint.Initialize(upright, knuckle, steer_frame)
            steer_fn = chrono.ChFunctionConst(0.0)
            steer_joint.SetAngleFunction(steer_fn)
            sys.Add(steer_joint)
            steer_functions[name] = steer_fn

            spin_parent = knuckle

        # ---- wheel ----
        wheel = chrono.ChBodyEasyCylinder(
            chrono.ChAxis_Y, WHEEL_RADIUS, WHEEL_WIDTH, 250, True, True, mat
        )
        wheel.SetMass(WHEEL_MASS)
        wheel.SetPos(upright_pos)
        wheel.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.1, 0.1))
        # rotation marker: a bright stripe offset from the spin axis, so the
        # wheel's actual spin rate/direction is visible on screen (a plain
        # cylinder looks identical whether it's still or spinning fast)
        marker = chrono.ChVisualShapeBox(0.10, WHEEL_WIDTH * 1.05, 0.03)
        marker.SetColor(chrono.ChColor(0.95, 0.85, 0.1))
        wheel.AddVisualShape(
            marker, chrono.ChFramed(chrono.ChVector3d(WHEEL_RADIUS - 0.02, 0, 0), chrono.QUNIT)
        )
        sys.Add(wheel)
        wheels[name] = wheel

        # revolute joint: wheel spins about Y axis relative to its parent
        # (the knuckle for front wheels, so the spin axis steers with it)
        revolute = chrono.ChLinkLockRevolute()
        rev_frame = chrono.ChFramed(upright_pos, chrono.QuatFromAngleX(chrono.CH_PI_2))
        revolute.Initialize(spin_parent, wheel, rev_frame)
        sys.Add(revolute)

        # drive torque motor on driven axles only
        if is_driven:
            motor = chrono.ChLinkMotorRotationTorque()
            motor.Initialize(spin_parent, wheel, rev_frame)
            throttle_fn = chrono.ChFunctionConst(DRIVE_TORQUE)
            motor.SetTorqueFunction(throttle_fn)
            sys.Add(motor)
            motors[name] = motor
            throttle_functions[name] = throttle_fn

    return chassis, wheels, springs, motors, steer_functions, throttle_functions


def apply_differential(motors, throttle_functions, nominal_torque,
                        gain=DIFF_GAIN, max_bias_fraction=DIFF_MAX_BIAS_FRACTION):
    """For each driven axle (a "<prefix>L"/"<prefix>R" pair in motors), read
    the actual L/R wheel spin-rate difference off the drive motors themselves
    (ChLinkMotorRotationTorque.GetMotorAngleDt() -- the relative angular speed
    the motor is actually producing) and shift torque away from whichever
    side is spinning faster (i.e. slipping) toward the slower, gripping side.
    A real open differential always splits torque exactly 50/50 regardless of
    speed -- this adds the limited-slip-style bias on top of that baseline.
    """
    prefixes = sorted({name[:-1] for name in motors})
    for prefix in prefixes:
        left_name, right_name = prefix + "L", prefix + "R"
        if left_name not in motors or right_name not in motors:
            continue
        w_left = motors[left_name].GetMotorAngleDt()
        w_right = motors[right_name].GetMotorAngleDt()
        max_bias = abs(nominal_torque) * max_bias_fraction
        bias = max(-max_bias, min(max_bias, gain * (w_left - w_right)))
        throttle_functions[left_name].SetConstant(nominal_torque - bias)
        throttle_functions[right_name].SetConstant(nominal_torque + bias)


def setup_empirical_tire_wheels(wheels):
    """Switch the given wheels from rigid Coulomb-friction ground contact to
    the "empirical" slip-based tire model: zero out their contact friction
    (so Bullet only supplies the vertical/normal reaction, no tangential
    friction force) and give each wheel a force accumulator that
    apply_tire_forces() will fill in every step instead. Returns a
    {wheel_name: accumulator_index} dict to pass to apply_tire_forces().
    Purely additive -- make_vehicle() and the default (rigid) path are
    untouched; this only runs when --tire-model empirical is selected.
    """
    zero_mu_mat = chrono.ChContactMaterialNSC()
    zero_mu_mat.SetFriction(0.0)
    zero_mu_mat.SetRestitution(0.0)
    accumulators = {}
    for name, wheel in wheels.items():
        wheel.GetCollisionModel().SetAllShapesMaterial(zero_mu_mat)
        accumulators[name] = wheel.AddAccumulator()
    return accumulators


def apply_tire_forces(wheels, accumulators,
                       mu=TIRE_MU,
                       c_alpha=TIRE_CORNERING_STIFFNESS,
                       c_kappa=TIRE_LONGITUDINAL_STIFFNESS,
                       speed_eps=TIRE_SLIP_SPEED_EPS,
                       wheel_radius=WHEEL_RADIUS):
    """Compute and apply a simplified, hand-rolled slip-based tire force for
    each wheel (must have been set up via setup_empirical_tire_wheels()).

    For each wheel: derive its current rolling (heading) and spin-axis
    (lateral) directions from its own orientation (so it works whether the
    wheel is steered or not), compute the longitudinal slip ratio and
    lateral slip angle from the wheel's linear/angular velocity, get the
    actual normal load from the physics engine's own contact resolution
    (ChBody.GetContactForce()), and apply a linear-then-saturating
    (friction-circle) force law -- conceptually similar to TMEASY/Pacejka,
    though far simpler and not a literal port of either.
    """
    up = chrono.ChVector3d(0, 0, 1)
    for name, wheel in wheels.items():
        idx = accumulators[name]
        wheel.EmptyAccumulator(idx)

        spin_axis = wheel.GetRot().Rotate(chrono.ChVector3d(0, 1, 0))
        spin_axis = spin_axis - up * spin_axis.Dot(up)  # project out any camber/tilt
        spin_len = spin_axis.Length()
        if spin_len < 1e-6:
            continue
        spin_axis = spin_axis * (1.0 / spin_len)
        heading = spin_axis.Cross(up)  # rolling direction, in the ground plane
        heading = heading * (1.0 / heading.Length())

        vel = wheel.GetPosDt()
        v_forward = vel.Dot(heading)
        v_lateral = vel.Dot(spin_axis)
        omega_spin = wheel.GetAngVelParent().Dot(spin_axis)

        kappa = (omega_spin * wheel_radius - v_forward) / max(abs(v_forward), speed_eps)
        alpha = math.atan2(v_lateral, max(abs(v_forward), speed_eps) * (1 if v_forward >= 0 else -1))

        fz = max(wheel.GetContactForce().z, 0.0)
        fx_lin = c_kappa * fz * kappa
        fy_lin = -c_alpha * fz * alpha

        f_max = mu * fz
        mag = math.hypot(fx_lin, fy_lin)
        if mag > f_max and mag > 1e-9:
            scale = f_max / mag
            fx_lin *= scale
            fy_lin *= scale

        force = heading * fx_lin + spin_axis * fy_lin
        contact_point = wheel.GetPos() - up * wheel_radius
        wheel.AccumulateForce(idx, force, contact_point, False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--irrlicht", action="store_true", help="open live 3D view")
    parser.add_argument("--time", type=float, default=SIM_TIME)
    parser.add_argument("--steer-deg", type=float, default=STEER_DEG_DEFAULT,
                         help="target front-wheel steer angle in degrees (both wheels, parallel)")
    parser.add_argument("--steer-start", type=float, default=STEER_START_DEFAULT,
                         help="sim time [s] when steering begins")
    parser.add_argument("--steer-ramp", type=float, default=STEER_RAMP_DEFAULT,
                         help="ramp duration [s] to reach the target steer angle")
    parser.add_argument("--six-wheel", action="store_true",
                         help="use the 3-axle (6-wheel) truck layout instead of the 4-wheel car")
    parser.add_argument("--ackermann", action="store_true",
                         help="split the commanded steer angle into separate L/R wheel angles "
                              "via Ackermann geometry, instead of applying it to both equally")
    parser.add_argument("--tire-model", choices=["rigid", "empirical"], default="rigid",
                         help="'rigid' (default): Bullet Coulomb-friction wheel/ground contact. "
                              "'empirical': slip-based tire force law (see apply_tire_forces)")
    args = parser.parse_args()

    chrono.SetChronoDataPath(
        os.environ.get("CHRONO_DATA_DIR", chrono.GetChronoDataPath())
    )

    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, G))
    sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)

    chassis, wheels, springs, motors, steer_functions, throttle_functions = make_vehicle(
        sys, six_wheel=args.six_wheel
    )
    susp_keys = sorted(springs.keys())
    ackermann_wheelbase = WHEELBASE_6W if args.six_wheel else WHEELBASE
    tire_accumulators = (
        setup_empirical_tire_wheels(wheels) if args.tire_model == "empirical" else None
    )

    log_path = os.path.join(os.path.dirname(__file__), "vehicle_log.csv")
    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(
        ["time", "chassis_x", "chassis_y", "chassis_z", "roll_deg", "pitch_deg", "yaw_deg"]
        + [f"susp_{k}" for k in susp_keys]
        + ["steer_deg", "steer_FL_deg", "steer_FR_deg", "speed_mps"]
    )

    vis = None
    if args.irrlicht:
        import pychrono.irrlicht as chronoirr
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
        vis.AttachSystem(sys)
        vis.SetWindowSize(1024, 768)
        vis.SetWindowTitle("Simple Chrono Vehicle")
        vis.Initialize()
        vis.ShowExplorer(False)
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(-4, -6, 2.5), chrono.ChVector3d(0, 0, 0.5))
        vis.AddTypicalLights()

    CAMERA_OFFSET = (
        chrono.ChVector3d(-9, -12, 5.0) if args.six_wheel else chrono.ChVector3d(-6, -8, 3.5)
    )  # chase-camera offset in world frame

    RENDER_DT = 1.0 / 60.0
    next_render_t = 0.0
    realtime_timer = chrono.ChRealtimeStepTimer()

    t = 0.0
    while t < args.time:
        if vis is not None:
            if not vis.Run():
                break
            if t >= next_render_t:
                chassis_pos = chassis.GetPos()
                vis.SetCameraTarget(chassis_pos)
                vis.SetCameraPosition(chassis_pos + CAMERA_OFFSET)
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                next_render_t += RENDER_DT

        cur_steer_deg = steer_angle_deg(t, args.steer_deg, args.steer_start, args.steer_ramp)
        if args.ackermann:
            left_deg, right_deg = ackermann_wheel_angles_deg(
                cur_steer_deg, ackermann_wheelbase, TRACK
            )
            if "FL" in steer_functions:
                steer_functions["FL"].SetConstant(math.radians(left_deg))
            if "FR" in steer_functions:
                steer_functions["FR"].SetConstant(math.radians(right_deg))
        else:
            cur_steer_rad = math.radians(cur_steer_deg)
            for steer_fn in steer_functions.values():
                steer_fn.SetConstant(cur_steer_rad)

        apply_differential(motors, throttle_functions, DRIVE_TORQUE)
        if tire_accumulators is not None:
            apply_tire_forces(wheels, tire_accumulators)

        sys.DoStepDynamics(TIME_STEP)
        t = sys.GetChTime()

        if vis is not None:
            realtime_timer.Spin(TIME_STEP)

        rot = chassis.GetRot()
        euler = rot.GetCardanAnglesXYZ()
        roll_deg = math.degrees(euler.x)
        pitch_deg = math.degrees(euler.y)
        yaw_deg = math.degrees(euler.z)
        pos = chassis.GetPos()
        vel = chassis.GetPosDt()
        speed = math.sqrt(vel.x ** 2 + vel.y ** 2)

        steer_FL_deg = math.degrees(steer_functions["FL"].GetVal(0)) if "FL" in steer_functions else 0.0
        steer_FR_deg = math.degrees(steer_functions["FR"].GetVal(0)) if "FR" in steer_functions else 0.0
        writer.writerow(
            [f"{t:.4f}", f"{pos.x:.4f}", f"{pos.y:.4f}", f"{pos.z:.4f}",
             f"{roll_deg:.3f}", f"{pitch_deg:.3f}", f"{yaw_deg:.3f}"]
            + [f"{springs[k].GetLength():.4f}" for k in susp_keys]
            + [f"{cur_steer_deg:.3f}", f"{steer_FL_deg:.3f}", f"{steer_FR_deg:.3f}", f"{speed:.4f}"]
        )

    log_file.close()
    print(f"Simulation done. Log written to {log_path}")


if __name__ == "__main__":
    main()
