/*
 * Native FMI2 Co-Simulation FMU for the vehicle, calling Chrono's C++ API
 * directly (same pattern as bouncing_ball_native_chrono.cpp) -- no Python,
 * no pythonfmu. This is the native-C++ counterpart of fmu/chrono_vehicle_fmu.py,
 * and (per the same proof as the bouncing-ball case) is expected to be the
 * only version loadable by a non-Python FMI master like OMSimulator/Modelica.
 *
 * MVP SCOPE (first pass -- additive follow-ups noted in README, not here):
 *   - 4-wheel car only (no six_wheel option)
 *   - rigid tire model only (Bullet Coulomb contact, no empirical_tire)
 *   - flat terrain only (no bumps_terrain)
 *   - parallel steering only (no ackermann)
 * These are exactly simple_vehicle.py's defaults, so this reproduces the
 * default-config vehicle from make_vehicle(), just constructed directly in
 * C++ instead of via PyChrono. Model parameters (masses, dims, spring/
 * damper rates, etc.) are copied verbatim from simple_vehicle.py's module
 * constants.
 *
 * Variables (value references):
 *   0: steer_deg      input   [deg]   commanded front-wheel steer angle
 *   1: drive_torque   input   [N*m]   nominal per-driven-wheel torque
 *   2: chassis_x      output  [m]
 *   3: chassis_y      output  [m]
 *   4: chassis_z      output  [m]
 *   5: roll_deg       output  [deg]
 *   6: pitch_deg      output  [deg]
 *   7: yaw_deg        output  [deg]
 *   8: speed_mps      output  [m/s]   sqrt(vx^2+vy^2)
 *   9: steer_FL_deg   output  [deg]
 *  10: steer_FR_deg   output  [deg]
 *
 * Build: see ../build.sh
 */
#include <cmath>
#include <cstring>
#include <memory>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/core/ChRotation.h"
#include "chrono/utils/ChConstants.h"

using namespace chrono;

// ---- model constants, copied from simple_vehicle.py ----
static const double G = -9.81;
static const double CHASSIS_MASS = 1200.0;
static const double CHASSIS_DIMS[3] = {2.6, 1.6, 0.4};
static const double CHASSIS_CG_HEIGHT = 0.55;
static const double WHEEL_MASS = 18.0;
static const double WHEEL_RADIUS = 0.32;
static const double WHEEL_WIDTH = 0.22;
static const double UPRIGHT_MASS = 15.0;
static const double KNUCKLE_MASS = 3.0;
static const double WHEELBASE = 2.6;
static const double TRACK = 1.5;
static const double SPRING_K = 35000.0;
static const double DAMPER_C = 3500.0;
static const double SUSPENSION_TRAVEL_REST = 0.32;
static const double DRIVE_TORQUE_DEFAULT = 260.0;
static const double GROUND_SIZE = 500.0;

// ---- minimal FMI2 type definitions (see bouncing_ball_native.c for why) ----
typedef void* fmi2Component;
typedef void* fmi2ComponentEnvironment;
typedef void* fmi2FMUstate;
typedef const char* fmi2String;
typedef double fmi2Real;
typedef int fmi2Integer;
typedef int fmi2Boolean;
typedef unsigned int fmi2ValueReference;
typedef int fmi2Status;
typedef int fmi2Type;
typedef int fmi2StatusKind;

#define fmi2OK 0
#define fmi2Error 3
#define fmi2CoSimulation 1

typedef void (*fmi2CallbackLogger)(fmi2ComponentEnvironment, fmi2String, fmi2Status, fmi2String, fmi2String, ...);
typedef void* (*fmi2CallbackAllocateMemory)(size_t, size_t);
typedef void (*fmi2CallbackFreeMemory)(void*);
typedef void (*fmi2StepFinished)(fmi2ComponentEnvironment, fmi2Status);

typedef struct {
    fmi2CallbackLogger logger;
    fmi2CallbackAllocateMemory allocateMemory;
    fmi2CallbackFreeMemory freeMemory;
    fmi2StepFinished stepFinished;
    fmi2ComponentEnvironment componentEnvironment;
} fmi2CallbackFunctions;

// ---- vehicle model ----

struct Corner {
    std::shared_ptr<ChBody> upright;
    std::shared_ptr<ChBody> wheel;
    std::shared_ptr<ChLinkMotorRotationAngle> steer_motor;  // null if not steered
    std::shared_ptr<ChFunctionConst> steer_fn;              // null if not steered
    std::shared_ptr<ChLinkMotorRotationTorque> drive_motor; // null if not driven
    std::shared_ptr<ChFunctionConst> throttle_fn;           // null if not driven
    double steer_deg_actual = 0.0;
};

// corner order: 0=FL 1=FR 2=RL 3=RR
struct ModelInstance {
    fmi2Real steer_deg_in = 0.0;
    fmi2Real drive_torque_in = DRIVE_TORQUE_DEFAULT;

    fmi2Real chassis_x = 0.0, chassis_y = 0.0, chassis_z = 0.0;
    fmi2Real roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;
    fmi2Real speed_mps = 0.0;
    fmi2Real steer_FL_deg = 0.0, steer_FR_deg = 0.0;

    std::shared_ptr<ChSystemNSC> sys;
    std::shared_ptr<ChBody> chassis;
    Corner corners[4];
    bool built = false;

    void build();
    void step(double dt);
};

static void make_corner(ChSystemNSC& sys, std::shared_ptr<ChBody> chassis, Corner& c,
                         double x, double y, double chassis_z, bool is_steered, bool is_driven,
                         std::shared_ptr<ChContactMaterial> mat) {
    ChVector3d mount_pos(x, y, chassis_z);
    double upright_z = WHEEL_RADIUS;
    ChVector3d upright_pos(x, y, upright_z);

    // upright (unsprung mass, vertical prismatic + spring-damper vs chassis)
    c.upright = chrono_types::make_shared<ChBody>();
    c.upright->SetMass(UPRIGHT_MASS);
    c.upright->SetInertiaXX(ChVector3d(0.05, 0.05, 0.05));
    c.upright->SetPos(upright_pos);
    sys.Add(c.upright);

    auto prismatic = chrono_types::make_shared<ChLinkLockPrismatic>();
    ChFramed prismatic_frame(mount_pos, QuatFromAngleY(CH_PI_2));
    prismatic->Initialize(chassis, c.upright, prismatic_frame);
    sys.Add(prismatic);

    auto tsda = chrono_types::make_shared<ChLinkTSDA>();
    tsda->Initialize(chassis, c.upright, false, mount_pos, upright_pos);
    tsda->SetSpringCoefficient(SPRING_K);
    tsda->SetDampingCoefficient(DAMPER_C);
    tsda->SetRestLength((mount_pos - upright_pos).Length());
    sys.Add(tsda);

    // steering knuckle (steered corners only)
    std::shared_ptr<ChBody> spin_parent = c.upright;
    if (is_steered) {
        auto knuckle = chrono_types::make_shared<ChBody>();
        knuckle->SetMass(KNUCKLE_MASS);
        knuckle->SetInertiaXX(ChVector3d(0.01, 0.01, 0.01));
        knuckle->SetPos(upright_pos);
        sys.Add(knuckle);

        c.steer_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        ChFramed steer_frame(upright_pos, QUNIT);  // local Z = world Z (steer axis)
        c.steer_motor->Initialize(c.upright, knuckle, steer_frame);
        c.steer_fn = chrono_types::make_shared<ChFunctionConst>(0.0);
        c.steer_motor->SetAngleFunction(c.steer_fn);
        sys.Add(c.steer_motor);

        spin_parent = knuckle;
    }

    // wheel
    c.wheel = chrono_types::make_shared<ChBodyEasyCylinder>(
        ChAxis::Y, WHEEL_RADIUS, WHEEL_WIDTH, 250.0, true, true, mat);
    c.wheel->SetMass(WHEEL_MASS);
    c.wheel->SetPos(upright_pos);
    sys.Add(c.wheel);

    auto revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    ChFramed rev_frame(upright_pos, QuatFromAngleX(CH_PI_2));
    revolute->Initialize(spin_parent, c.wheel, rev_frame);
    sys.Add(revolute);

    if (is_driven) {
        c.drive_motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
        c.drive_motor->Initialize(spin_parent, c.wheel, rev_frame);
        c.throttle_fn = chrono_types::make_shared<ChFunctionConst>(DRIVE_TORQUE_DEFAULT);
        c.drive_motor->SetTorqueFunction(c.throttle_fn);
        sys.Add(c.drive_motor);
    }
}

void ModelInstance::build() {
    sys = chrono_types::make_shared<ChSystemNSC>();
    sys->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys->SetGravitationalAcceleration(ChVector3d(0, 0, G));
    sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys->GetSolver()->AsIterative()->SetMaxIterations(150);

    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.0f);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(GROUND_SIZE, GROUND_SIZE, 0.5, 1000.0, true, true, mat);
    ground->SetPos(ChVector3d(0, 0, -0.25));
    ground->SetFixed(true);
    sys->Add(ground);

    chassis = chrono_types::make_shared<ChBodyEasyBox>(CHASSIS_DIMS[0], CHASSIS_DIMS[1], CHASSIS_DIMS[2],
                                                         500.0, true, false);
    chassis->SetMass(CHASSIS_MASS);
    chassis->SetInertiaXX(ChVector3d(
        CHASSIS_MASS * (CHASSIS_DIMS[1] * CHASSIS_DIMS[1] + CHASSIS_DIMS[2] * CHASSIS_DIMS[2]) / 12,
        CHASSIS_MASS * (CHASSIS_DIMS[0] * CHASSIS_DIMS[0] + CHASSIS_DIMS[2] * CHASSIS_DIMS[2]) / 12,
        CHASSIS_MASS * (CHASSIS_DIMS[0] * CHASSIS_DIMS[0] + CHASSIS_DIMS[1] * CHASSIS_DIMS[1]) / 12));
    double chassis_z = WHEEL_RADIUS + SUSPENSION_TRAVEL_REST + CHASSIS_CG_HEIGHT;
    chassis->SetPos(ChVector3d(0, 0, chassis_z));
    sys->Add(chassis);

    // corner order: FL, FR, RL, RR -- front (x=+WHEELBASE/2) steered, rear driven
    make_corner(*sys, chassis, corners[0], WHEELBASE / 2, TRACK / 2, chassis_z, true, false, mat);
    make_corner(*sys, chassis, corners[1], WHEELBASE / 2, -TRACK / 2, chassis_z, true, false, mat);
    make_corner(*sys, chassis, corners[2], -WHEELBASE / 2, TRACK / 2, chassis_z, false, true, mat);
    make_corner(*sys, chassis, corners[3], -WHEELBASE / 2, -TRACK / 2, chassis_z, false, true, mat);

    built = true;
}

static void apply_differential(Corner& left, Corner& right, double nominal_torque,
                                double gain = 40.0, double max_bias_fraction = 0.9) {
    double w_left = left.drive_motor->GetMotorAngleDt();
    double w_right = right.drive_motor->GetMotorAngleDt();
    double max_bias = std::fabs(nominal_torque) * max_bias_fraction;
    double bias = gain * (w_left - w_right);
    if (bias > max_bias) bias = max_bias;
    if (bias < -max_bias) bias = -max_bias;
    left.throttle_fn->SetConstant(nominal_torque - bias);
    right.throttle_fn->SetConstant(nominal_torque + bias);
}

void ModelInstance::step(double dt) {
    double steer_rad = steer_deg_in * (CH_PI / 180.0);
    for (auto& c : corners) {
        if (c.steer_fn) {
            c.steer_fn->SetConstant(steer_rad);
            c.steer_deg_actual = steer_deg_in;
        }
    }

    // rear axle (corners 2=RL, 3=RR) is the only driven axle in this MVP
    apply_differential(corners[2], corners[3], drive_torque_in);

    sys->DoStepDynamics(dt);

    ChQuaterniond rot = chassis->GetRot();
    ChVector3d euler = rot.GetCardanAnglesXYZ();
    roll_deg = euler.x() * (180.0 / CH_PI);
    pitch_deg = euler.y() * (180.0 / CH_PI);
    yaw_deg = euler.z() * (180.0 / CH_PI);

    ChVector3d pos = chassis->GetPos();
    chassis_x = pos.x();
    chassis_y = pos.y();
    chassis_z = pos.z();

    ChVector3d vel = chassis->GetPosDt();
    speed_mps = std::sqrt(vel.x() * vel.x() + vel.y() * vel.y());

    steer_FL_deg = corners[0].steer_deg_actual;
    steer_FR_deg = corners[1].steer_deg_actual;
}

#define VR_STEER_DEG 0
#define VR_DRIVE_TORQUE 1
#define VR_CHASSIS_X 2
#define VR_CHASSIS_Y 3
#define VR_CHASSIS_Z 4
#define VR_ROLL_DEG 5
#define VR_PITCH_DEG 6
#define VR_YAW_DEG 7
#define VR_SPEED_MPS 8
#define VR_STEER_FL_DEG 9
#define VR_STEER_FR_DEG 10

static fmi2Real* var_ptr(ModelInstance* m, fmi2ValueReference vr) {
    switch (vr) {
        case VR_STEER_DEG: return &m->steer_deg_in;
        case VR_DRIVE_TORQUE: return &m->drive_torque_in;
        case VR_CHASSIS_X: return &m->chassis_x;
        case VR_CHASSIS_Y: return &m->chassis_y;
        case VR_CHASSIS_Z: return &m->chassis_z;
        case VR_ROLL_DEG: return &m->roll_deg;
        case VR_PITCH_DEG: return &m->pitch_deg;
        case VR_YAW_DEG: return &m->yaw_deg;
        case VR_SPEED_MPS: return &m->speed_mps;
        case VR_STEER_FL_DEG: return &m->steer_FL_deg;
        case VR_STEER_FR_DEG: return &m->steer_FR_deg;
        default: return nullptr;
    }
}

extern "C" {

const char* fmi2GetTypesPlatform(void) { return "default"; }
const char* fmi2GetVersion(void) { return "2.0"; }

fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories,
                                const fmi2String categories[]) {
    (void)c; (void)loggingOn; (void)nCategories; (void)categories;
    return fmi2OK;
}

fmi2Component fmi2Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
                               fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions,
                               fmi2Boolean visible, fmi2Boolean loggingOn) {
    (void)instanceName; (void)fmuGUID; (void)fmuResourceLocation; (void)functions; (void)visible; (void)loggingOn;
    if (fmuType != fmi2CoSimulation) return nullptr;
    return (fmi2Component) new ModelInstance();
}

void fmi2FreeInstance(fmi2Component c) {
    delete (ModelInstance*)c;
}

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean toleranceDefined, fmi2Real tolerance,
                                fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
    (void)c; (void)toleranceDefined; (void)tolerance; (void)startTime; (void)stopTimeDefined; (void)stopTime;
    return fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) { (void)c; return fmi2OK; }

fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    ModelInstance* m = (ModelInstance*)c;
    if (!m) return fmi2Error;
    m->build();
    return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component c) { (void)c; return fmi2OK; }
fmi2Status fmi2Reset(fmi2Component c) { (void)c; return fmi2Error; }  // not supported (would need a full rebuild)

fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {
    ModelInstance* m = (ModelInstance*)c;
    if (!m) return fmi2Error;
    for (size_t i = 0; i < nvr; ++i) {
        fmi2Real* p = var_ptr(m, vr[i]);
        if (!p) return fmi2Error;
        *p = value[i];
    }
    return fmi2OK;
}

fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {
    ModelInstance* m = (ModelInstance*)c;
    if (!m) return fmi2Error;
    for (size_t i = 0; i < nvr; ++i) {
        fmi2Real* p = var_ptr(m, vr[i]);
        if (!p) return fmi2Error;
        value[i] = *p;
    }
    return fmi2OK;
}

fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[]) {
    (void)c; (void)vr; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}

fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* state) {
    (void)c; if (state) *state = nullptr; return fmi2Error;
}
fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate state) {
    (void)c; (void)state; return fmi2Error;
}
fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* state) {
    (void)c; if (state) *state = nullptr; return fmi2OK;
}
fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate state, size_t* size) {
    (void)c; (void)state; if (size) *size = 0; return fmi2Error;
}
fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate state, fmi2Boolean* data, size_t size) {
    (void)c; (void)state; (void)data; (void)size; return fmi2Error;
}
fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Boolean* data, size_t size, fmi2FMUstate* state) {
    (void)c; (void)data; (void)size; if (state) *state = nullptr; return fmi2Error;
}
fmi2Status fmi2GetDirectionalDerivative(fmi2Component c, const fmi2ValueReference vrUnknown[], size_t nUnknown,
                                        const fmi2ValueReference vrKnown[], size_t nKnown,
                                        const fmi2Real dvKnown[], fmi2Real dvUnknown[]) {
    (void)c; (void)vrUnknown; (void)nUnknown; (void)vrKnown; (void)nKnown; (void)dvKnown; (void)dvUnknown;
    return fmi2Error;
}

fmi2Status fmi2SetRealInputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                       const fmi2Integer order[], const fmi2Real value[]) {
    (void)c; (void)vr; (void)order; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}
fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr,
                                        const fmi2Integer order[], fmi2Real value[]) {
    (void)c; (void)vr; (void)order; (void)value; return nvr == 0 ? fmi2OK : fmi2Error;
}

fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentCommunicationPoint,
                       fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint) {
    (void)currentCommunicationPoint; (void)noSetFMUStatePriorToCurrentPoint;
    ModelInstance* m = (ModelInstance*)c;
    if (!m || !m->built) return fmi2Error;
    m->step(communicationStepSize);
    return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component c) { (void)c; return fmi2OK; }

fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value) {
    (void)c; (void)s; if (value) *value = fmi2OK; return fmi2OK;
}
fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value) {
    (void)c; (void)s; if (value) *value = 0.0; return fmi2OK;
}
fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value) {
    (void)c; (void)s; if (value) *value = 0; return fmi2OK;
}
fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value) {
    (void)c; (void)s; if (value) *value = 0; return fmi2OK;
}
fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value) {
    (void)c; (void)s; if (value) *value = ""; return fmi2OK;
}

}  // extern "C"
