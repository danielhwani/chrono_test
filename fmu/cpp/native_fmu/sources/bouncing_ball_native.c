/*
 * Native FMI2 Co-Simulation FMU for the bouncing ball model -- same physics
 * as bouncing_ball_fmu.py / bouncing_ball.cpp, but this time implementing
 * the FMI2 C API directly instead of going through pythonfmu. No embedded
 * Python interpreter anywhere in the resulting .so -- a driver (fmpy, or a
 * C/C++ program) talks to this shared library through the plain C ABI below.
 *
 * Deliberately does NOT include the official fmi2Functions.h / fmi2TypesPlatform.h
 * headers (not vendored into this repo) -- the handful of types and function
 * signatures used here are hand-written to match the FMI 2.0 spec exactly
 * (function names, parameter order/types, and enum values are part of the
 * spec and are stable), which is enough for a Co-Simulation-only FMU.
 *
 * Model:
 *   v += g*dt
 *   h += v*dt
 *   if h < floor: h = floor; v = -e*v
 *
 * Variables (value references, must match modelDescription.xml):
 *   0: g      input   [m/s^2]  default -9.81
 *   1: e      input   [-]      default 0.7
 *   2: floor  input   [m]      default 0.0
 *   3: h      output  [m]      start 10.0
 *   4: v      output  [m/s]    start 0.0
 *
 * Build: see ../build.sh
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ---- minimal FMI2 type definitions (see file header comment) ---- */
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
#define fmi2Warning 1
#define fmi2Discard 2
#define fmi2Error 3
#define fmi2Fatal 4
#define fmi2Pending 5

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
typedef struct {
    fmi2Real g, e, floor_;
    fmi2Real h, v;
    char instance_name[128];
} ModelInstance;

#define VR_G 0
#define VR_E 1
#define VR_FLOOR 2
#define VR_H 3
#define VR_V 4

/* ---- required exports ---- */

#ifdef __cplusplus
extern "C" {
#endif

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
    (void)fmuGUID; (void)fmuResourceLocation; (void)visible; (void)loggingOn;
    if (fmuType != fmi2CoSimulation) return NULL;

    ModelInstance* m;
    if (functions && functions->allocateMemory) {
        m = (ModelInstance*)functions->allocateMemory(1, sizeof(ModelInstance));
    } else {
        m = (ModelInstance*)malloc(sizeof(ModelInstance));
    }
    if (!m) return NULL;

    m->g = -9.81;
    m->e = 0.7;
    m->floor_ = 0.0;
    m->h = 10.0;
    m->v = 0.0;
    strncpy(m->instance_name, instanceName ? instanceName : "bouncing_ball", sizeof(m->instance_name) - 1);
    m->instance_name[sizeof(m->instance_name) - 1] = '\0';
    return (fmi2Component)m;
}

void fmi2FreeInstance(fmi2Component c) {
    if (c) free(c);
}

fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean toleranceDefined, fmi2Real tolerance,
                                fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
    (void)c; (void)toleranceDefined; (void)tolerance; (void)startTime; (void)stopTimeDefined; (void)stopTime;
    return fmi2OK;
}

fmi2Status fmi2EnterInitializationMode(fmi2Component c) { (void)c; return fmi2OK; }
fmi2Status fmi2ExitInitializationMode(fmi2Component c) { (void)c; return fmi2OK; }
fmi2Status fmi2Terminate(fmi2Component c) { (void)c; return fmi2OK; }

fmi2Status fmi2Reset(fmi2Component c) {
    ModelInstance* m = (ModelInstance*)c;
    if (!m) return fmi2Error;
    m->g = -9.81;
    m->e = 0.7;
    m->floor_ = 0.0;
    m->h = 10.0;
    m->v = 0.0;
    return fmi2OK;
}

static fmi2Real* var_ptr(ModelInstance* m, fmi2ValueReference vr) {
    switch (vr) {
        case VR_G: return &m->g;
        case VR_E: return &m->e;
        case VR_FLOOR: return &m->floor_;
        case VR_H: return &m->h;
        case VR_V: return &m->v;
        default: return NULL;
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

/* Integer/Boolean/String variables: none in this model, so these are just
 * no-ops that succeed on an empty request and fail otherwise. */
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

/* FMU state save/restore and directional-derivative functions: not
 * supported (modelDescription.xml declares canGetAndSetFMUstate="false"
 * etc), but fmpy resolves every standard FMI2 symbol from the shared
 * library up front regardless of the declared capabilities, so these
 * still need to exist -- they just always fail/no-op. */
fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* state) {
    (void)c; if (state) *state = NULL; return fmi2Error;
}
fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate state) {
    (void)c; (void)state; return fmi2Error;
}
fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* state) {
    (void)c; if (state) *state = NULL; return fmi2OK;
}
fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate state, size_t* size) {
    (void)c; (void)state; if (size) *size = 0; return fmi2Error;
}
fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate state, fmi2Boolean* data, size_t size) {
    (void)c; (void)state; (void)data; (void)size; return fmi2Error;
}
fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Boolean* data, size_t size, fmi2FMUstate* state) {
    (void)c; (void)data; (void)size; if (state) *state = NULL; return fmi2Error;
}
fmi2Status fmi2GetDirectionalDerivative(fmi2Component c, const fmi2ValueReference vrUnknown[], size_t nUnknown,
                                        const fmi2ValueReference vrKnown[], size_t nKnown,
                                        const fmi2Real dvKnown[], fmi2Real dvUnknown[]) {
    (void)c; (void)vrUnknown; (void)nUnknown; (void)vrKnown; (void)nKnown; (void)dvKnown; (void)dvUnknown;
    return fmi2Error;
}

/* ---- Co-Simulation ---- */

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
    if (!m) return fmi2Error;

    m->v += m->g * communicationStepSize;
    m->h += m->v * communicationStepSize;
    if (m->h < m->floor_) {
        m->h = m->floor_;
        m->v = -m->e * m->v;
    }
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

#ifdef __cplusplus
}
#endif
