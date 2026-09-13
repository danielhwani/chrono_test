/*
 * fmu_driver -- a minimal C "master"/host for an FMI2 Co-Simulation FMU,
 * loaded via dlopen()/dlsym() instead of any FMI library (fmpy, etc).
 *
 * This is deliberately NOT the same thing as either of these other files in
 * the repo, which it sits between:
 *   - fmu/cpp/bouncing_ball.cpp   : pure C++ physics, no FMI at all. Used as
 *                                    the "ceiling" perf number (raw compute,
 *                                    no FMU call overhead).
 *   - .../sources/bouncing_ball_native.c : the FMU *model* itself -- the
 *                                    fmi2DoStep/fmi2GetReal/... implementation
 *                                    that gets compiled into the .so this
 *                                    driver loads. It doesn't run on its own.
 * fmu_driver.c is the *caller*: it dlopen()s that .so and calls its FMI2
 * entry points directly, the same role fmpy plays in validate_native_fmu.py
 * and benchmark_realtime.py, but with zero Python anywhere in the process.
 * That completes the fully-native path: native FMU + native driver.
 *
 * Usage:
 *   ./fmu_driver <path-to-.so> bench <sim_time_s> <dt_s>
 *   ./fmu_driver <path-to-.so> csv   <sim_time_s> <dt_s>
 *
 *   bench -- flat-out fmi2DoStep loop (no pacing), reports ns/step and RTF.
 *            Compare directly against `bouncing_ball --bench` to see the
 *            overhead the FMI function-pointer layer adds over raw C++.
 *   csv   -- prints time,h,v rows to stdout (redirect to a file to compare
 *            against bouncing_ball.cpp --csv / the Python FMU's output).
 *
 * Build: see ./build.sh (needs -ldl).
 */
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ---- minimal FMI2 type definitions, matching bouncing_ball_native.c ---- */
typedef void* fmi2Component;
typedef void* fmi2ComponentEnvironment;
typedef const char* fmi2String;
typedef double fmi2Real;
typedef int fmi2Integer;
typedef int fmi2Boolean;
typedef unsigned int fmi2ValueReference;
typedef int fmi2Status;
typedef int fmi2Type;

#define fmi2OK 0
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

typedef fmi2Component (*fmi2Instantiate_t)(fmi2String, fmi2Type, fmi2String, fmi2String,
                                            const fmi2CallbackFunctions*, fmi2Boolean, fmi2Boolean);
typedef void (*fmi2FreeInstance_t)(fmi2Component);
typedef fmi2Status (*fmi2SetupExperiment_t)(fmi2Component, fmi2Boolean, fmi2Real, fmi2Real, fmi2Boolean, fmi2Real);
typedef fmi2Status (*fmi2EnterInitializationMode_t)(fmi2Component);
typedef fmi2Status (*fmi2ExitInitializationMode_t)(fmi2Component);
typedef fmi2Status (*fmi2Terminate_t)(fmi2Component);
typedef fmi2Status (*fmi2SetReal_t)(fmi2Component, const fmi2ValueReference[], size_t, const fmi2Real[]);
typedef fmi2Status (*fmi2GetReal_t)(fmi2Component, const fmi2ValueReference[], size_t, fmi2Real[]);
typedef fmi2Status (*fmi2DoStep_t)(fmi2Component, fmi2Real, fmi2Real, fmi2Boolean);

/* value references, must match modelDescription.xml / bouncing_ball_native.c */
#define VR_G 0
#define VR_E 1
#define VR_FLOOR 2
#define VR_H 3
#define VR_V 4

#define GUID "{a1b2c3d4-e5f6-4789-a012-3456789abcde}"

static void* xdlsym(void* handle, const char* name) {
    void* sym = dlsym(handle, name);
    if (!sym) {
        fprintf(stderr, "dlsym failed for %s: %s\n", name, dlerror());
        exit(1);
    }
    return sym;
}

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char** argv) {
    if (argc != 5) {
        fprintf(stderr, "usage: %s <path-to-.so> {bench|csv} <sim_time_s> <dt_s>\n", argv[0]);
        return 1;
    }
    const char* so_path = argv[1];
    const char* mode = argv[2];
    double sim_time = atof(argv[3]);
    double dt = atof(argv[4]);
    int bench = strcmp(mode, "bench") == 0;
    if (!bench && strcmp(mode, "csv") != 0) {
        fprintf(stderr, "mode must be 'bench' or 'csv', got '%s'\n", mode);
        return 1;
    }

    void* handle = dlopen(so_path, RTLD_NOW);
    if (!handle) {
        fprintf(stderr, "dlopen(%s) failed: %s\n", so_path, dlerror());
        return 1;
    }

    fmi2Instantiate_t fmi2Instantiate = (fmi2Instantiate_t)xdlsym(handle, "fmi2Instantiate");
    fmi2FreeInstance_t fmi2FreeInstance = (fmi2FreeInstance_t)xdlsym(handle, "fmi2FreeInstance");
    fmi2SetupExperiment_t fmi2SetupExperiment = (fmi2SetupExperiment_t)xdlsym(handle, "fmi2SetupExperiment");
    fmi2EnterInitializationMode_t fmi2EnterInitializationMode =
        (fmi2EnterInitializationMode_t)xdlsym(handle, "fmi2EnterInitializationMode");
    fmi2ExitInitializationMode_t fmi2ExitInitializationMode =
        (fmi2ExitInitializationMode_t)xdlsym(handle, "fmi2ExitInitializationMode");
    fmi2Terminate_t fmi2Terminate = (fmi2Terminate_t)xdlsym(handle, "fmi2Terminate");
    fmi2SetReal_t fmi2SetReal = (fmi2SetReal_t)xdlsym(handle, "fmi2SetReal");
    fmi2GetReal_t fmi2GetReal = (fmi2GetReal_t)xdlsym(handle, "fmi2GetReal");
    fmi2DoStep_t fmi2DoStep = (fmi2DoStep_t)xdlsym(handle, "fmi2DoStep");

    fmi2Component c = fmi2Instantiate("fmu_driver_instance", fmi2CoSimulation, GUID, "", NULL, 0, 0);
    if (!c) {
        fprintf(stderr, "fmi2Instantiate returned NULL\n");
        return 1;
    }
    fmi2SetupExperiment(c, 0, 0.0, 0.0, 0, 0.0);
    fmi2EnterInitializationMode(c);
    fmi2ExitInitializationMode(c);

    long n_steps = (long)(sim_time / dt);
    fmi2ValueReference vr_hv[2] = {VR_H, VR_V};
    fmi2Real hv[2];

    if (bench) {
        double t0 = now_s();
        double t = 0.0;
        for (long i = 0; i < n_steps; ++i) {
            fmi2DoStep(c, t, dt, 1);
            t += dt;
        }
        double elapsed = now_s() - t0;
        fmi2GetReal(c, vr_hv, 2, hv);
        double ns_per_step = elapsed * 1e9 / n_steps;
        double rtf = sim_time / elapsed;
        printf("fmu_driver bench: %ld steps, dt=%g, sim_time=%g s\n", n_steps, dt, sim_time);
        printf("  wall elapsed: %.6f s\n", elapsed);
        printf("  ns/step:      %.2f\n", ns_per_step);
        printf("  RTF:          %.1fx realtime\n", rtf);
        printf("  final state:  h=%.6f v=%.6f (printed so -O2 can't dead-code-eliminate the loop)\n", hv[0], hv[1]);
    } else {
        double t = 0.0;
        printf("time,h,v\n");
        fmi2GetReal(c, vr_hv, 2, hv);
        printf("%.6f,%.6f,%.6f\n", t, hv[0], hv[1]);
        for (long i = 0; i < n_steps; ++i) {
            fmi2DoStep(c, t, dt, 1);
            t += dt;
            fmi2GetReal(c, vr_hv, 2, hv);
            printf("%.6f,%.6f,%.6f\n", t, hv[0], hv[1]);
        }
    }

    fmi2Terminate(c);
    fmi2FreeInstance(c);
    dlclose(handle);
    return 0;
}
