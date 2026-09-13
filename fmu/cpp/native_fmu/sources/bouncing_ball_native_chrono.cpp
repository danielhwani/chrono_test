/*
 * Same FMI2 Co-Simulation surface as bouncing_ball_native.c (g/e/floor
 * inputs, h/v outputs, value references 0-4, same causality/defaults) --
 * but fmi2DoStep() doesn't hand-integrate the physics. It builds a real
 * chrono::ChSystemNSC (Bullet collision, an NSC contact material carrying
 * the restitution coefficient e) with a sphere and a fixed ground, and each
 * step just calls sys->DoStepDynamics(). This is the C++-native counterpart
 * of fmu/chrono_bouncing_ball_fmu.py (which does the same thing but through
 * pythonfmu) -- and unlike that one, this .so has zero Python dependency,
 * so it's expected to work as a slave for a non-Python master (verified:
 * OMSimulator loads it -- see the "네이티브 Chrono FMU" README section).
 *
 * This is a NEW, separate model/artifact alongside bouncing_ball_native.c --
 * that file and its .fmu are untouched. Both live under native_fmu/ and are
 * built by separate scripts (./build.sh vs ./build_chrono.sh) into separate
 * binaries/modelIdentifiers, so either can be used without disturbing the
 * other. See chrono_variant/ for this model's own modelDescription.xml.
 *
 * h is the sphere's bottom-surface height (center z - radius), matching the
 * floor-contact convention of every other bouncing-ball model in this repo.
 *
 * Build: see ./build_chrono.sh (needs the Chrono C++ headers/libs that ship
 * inside the `chrono` conda env, e.g. .../envs/chrono/include/chrono and
 * .../envs/chrono/lib/libChrono_core.so -- not just the pychrono Python
 * bindings).
 */
#include <cstdlib>
#include <cstring>
#include <memory>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/collision/ChCollisionSystem.h"

static const double RADIUS = 0.05;

/* ---- minimal FMI2 type definitions (see bouncing_ball_native.c for why
 * these are hand-written instead of vendoring the official headers) ---- */
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

#define fmi2ModelExchange 0
#define fmi2CoSimulation 1

#define fmi2True 1
#define fmi2False 0

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

/* ---- model state ---- */
struct ModelInstance {
    fmi2Real g = -9.81, e = 0.7, floor_ = 0.0;
    fmi2Real h = 10.0, v = 0.0;
    char instance_name[128] = {0};

    std::shared_ptr<chrono::ChSystemNSC> sys;
    std::shared_ptr<chrono::ChBody> ball;
    bool built = false;

    void build() {
        sys = chrono_types::make_shared<chrono::ChSystemNSC>();
        sys->SetGravitationalAcceleration(chrono::ChVector3d(0, 0, g));
        sys->SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET);

        auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
        mat->SetRestitution((float)e);
        mat->SetFriction(0.0f);

        auto sphere = chrono_types::make_shared<chrono::ChBodyEasySphere>(RADIUS, 1000.0, true, true, mat);
        sphere->SetPos(chrono::ChVector3d(0, 0, h + RADIUS));
        sys->Add(sphere);
        ball = sphere;

        auto ground = chrono_types::make_shared<chrono::ChBodyEasyBox>(50.0, 50.0, 0.1, 1000.0, true, true, mat);
        ground->SetPos(chrono::ChVector3d(0, 0, floor_ - 0.05));
        ground->SetFixed(true);
        sys->Add(ground);

        built = true;
    }

    void step(double dt) {
        sys->DoStepDynamics(dt);
        h = ball->GetPos().z() - RADIUS;
        v = ball->GetPosDt().z();
    }
};

#define VR_G 0
#define VR_E 1
#define VR_FLOOR 2
#define VR_H 3
#define VR_V 4

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
    (void)fmuGUID; (void)fmuResourceLocation; (void)functions; (void)visible; (void)loggingOn;
    if (fmuType != fmi2CoSimulation) return nullptr;

    ModelInstance* m = new ModelInstance();
    std::strncpy(m->instance_name, instanceName ? instanceName : "bouncing_ball_chrono",
                 sizeof(m->instance_name) - 1);
    return (fmi2Component)m;
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
    m->build();  // build the Chrono system now, with whatever g/e/floor were set as inputs
    return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component c) { (void)c; return fmi2OK; }

fmi2Status fmi2Reset(fmi2Component c) {
    ModelInstance* m = (ModelInstance*)c;
    if (!m) return fmi2Error;
    m->g = -9.81; m->e = 0.7; m->floor_ = 0.0; m->h = 10.0; m->v = 0.0;
    m->built = false;
    m->sys.reset();
    m->ball.reset();
    return fmi2OK;
}

static fmi2Real* var_ptr(ModelInstance* m, fmi2ValueReference vr) {
    switch (vr) {
        case VR_G: return &m->g;
        case VR_E: return &m->e;
        case VR_FLOOR: return &m->floor_;
        case VR_H: return &m->h;
        case VR_V: return &m->v;
        default: return nullptr;
    }
}

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
    (void)c; (void)s; if (value) *value = fmi2False; return fmi2OK;
}
fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value) {
    (void)c; (void)s; if (value) *value = ""; return fmi2OK;
}

}  // extern "C"
