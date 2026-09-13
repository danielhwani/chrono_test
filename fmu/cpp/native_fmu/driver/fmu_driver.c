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
 *   ./fmu_driver <path-to-.so> bench  <sim_time_s> <dt_s>
 *   ./fmu_driver <path-to-.so> csv    <sim_time_s> <dt_s>
 *   ./fmu_driver <path-to-.so> paced  <sim_time_s> <dt_s> [fifo <priority>] [--json-out PATH]
 *
 *   bench -- flat-out fmi2DoStep loop (no pacing), reports ns/step and RTF.
 *            Compare directly against `bouncing_ball --bench` to see the
 *            overhead the FMI function-pointer layer adds over raw C++.
 *   csv   -- prints time,h,v rows to stdout (redirect to a file to compare
 *            against bouncing_ball.cpp --csv / the Python FMU's output).
 *   paced -- actually paces fmi2DoStep to wall-clock time, same yield()-based
 *            wait as bouncing_ball.cpp --paced (see that file's comment for
 *            why: sleep_until() alone had multi-ms tail latency here, a pure
 *            busy-yield loop doesn't). Reports per-step period percentiles
 *            and can write the same JSON schema as rt_results/*.json via
 *            --json-out, so this overlays directly with the existing Python
 *            (fmpy) and no-FMU C++ paced results in plot_rt_comparison.py.
 *
 * Build: see ./build.sh (needs -ldl).
 */
#include <dlfcn.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/utsname.h>
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

/* ---- runtime context (kernel + this process's scheduling policy), same
 * fields/format as bouncing_ball.cpp's get_runtime_context/print_runtime_context ---- */
typedef struct {
    char kernel_release[65];
    int is_rt_kernel;
    const char* policy_name;
    int priority;
} RuntimeContext;

static RuntimeContext get_runtime_context(void) {
    RuntimeContext ctx;
    struct utsname u;
    uname(&u);
    strncpy(ctx.kernel_release, u.release, sizeof(ctx.kernel_release) - 1);
    ctx.kernel_release[sizeof(ctx.kernel_release) - 1] = '\0';

    ctx.is_rt_kernel = 0;
    FILE* f = fopen("/sys/kernel/realtime", "r");
    if (f) {
        int val = 0;
        if (fscanf(f, "%d", &val) == 1) ctx.is_rt_kernel = (val == 1);
        fclose(f);
    }

    int policy = sched_getscheduler(0);
    ctx.policy_name = policy == SCHED_FIFO ? "SCHED_FIFO" :
                       policy == SCHED_RR ? "SCHED_RR" :
                       policy == SCHED_OTHER ? "SCHED_OTHER" : "unknown";
    struct sched_param sp;
    sched_getparam(0, &sp);
    ctx.priority = sp.sched_priority;
    return ctx;
}

static void print_runtime_context(const RuntimeContext* ctx) {
    printf("kernel: Linux %s  (PREEMPT_RT kernel: %s)  process sched: %s prio=%d\n",
           ctx->kernel_release, ctx->is_rt_kernel ? "yes" : "no", ctx->policy_name, ctx->priority);
}

static int cmp_double(const void* a, const void* b) {
    double da = *(const double*)a, db = *(const double*)b;
    return (da > db) - (da < db);
}

static double percentile(double* sorted_vals, long n, double p) {
    long idx = (long)(n * p);
    if (idx >= n) idx = n - 1;
    return sorted_vals[idx];
}

/* Same JSON schema as rt_results/*.json (benchmark_rt_jitter.py / bouncing_ball.cpp
 * --paced --json-out), so plot_rt_comparison.py overlays this with everything else. */
static void write_json(const char* path, const char* kernel_label, int is_rt_kernel,
                        double step_size, long n, double mean, double* sorted_periods,
                        double* periods) {
    FILE* f = fopen(path, "w");
    if (!f) {
        perror(path);
        return;
    }
    fprintf(f, "{\"kernel\": \"%s\", \"is_realtime_kernel\": %s, \"step_size\": %.10f, "
               "\"n_samples\": %ld, \"mean\": %.10f, \"min\": %.10f, \"max\": %.10f, "
               "\"p50\": %.10f, \"p95\": %.10f, \"p99\": %.10f, \"p999\": %.10f, \"periods\": [",
            kernel_label, is_rt_kernel ? "true" : "false", step_size, n, mean,
            sorted_periods[0], sorted_periods[n - 1],
            percentile(sorted_periods, n, 0.50), percentile(sorted_periods, n, 0.95),
            percentile(sorted_periods, n, 0.99), percentile(sorted_periods, n, 0.999));
    for (long i = 0; i < n; ++i) {
        fprintf(f, "%s%.10f", i == 0 ? "" : ", ", periods[i]);
    }
    fprintf(f, "]}\n");
    fclose(f);
    printf("  wrote %s\n", path);
}

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
    /* pull "--json-out PATH" out wherever it appears, then treat the rest positionally */
    char* json_out = NULL;
    int argn = argc - 1;
    char** args = argv + 1;
    for (int i = 0; i < argn; ++i) {
        if (strcmp(args[i], "--json-out") == 0 && i + 1 < argn) {
            json_out = args[i + 1];
            for (int j = i; j + 2 < argn; ++j) args[j] = args[j + 2];
            argn -= 2;
            break;
        }
    }

    if (argn < 4) {
        fprintf(stderr, "usage: %s <path-to-.so> {bench|csv|paced} <sim_time_s> <dt_s> "
                        "[fifo priority] [--json-out PATH]\n", argv[0]);
        return 1;
    }
    const char* so_path = args[0];
    const char* mode = args[1];
    double sim_time = atof(args[2]);
    double dt = atof(args[3]);
    int is_bench = strcmp(mode, "bench") == 0;
    int is_paced = strcmp(mode, "paced") == 0;
    if (!is_bench && !is_paced && strcmp(mode, "csv") != 0) {
        fprintf(stderr, "mode must be 'bench', 'csv', or 'paced', got '%s'\n", mode);
        return 1;
    }

    /* optional trailing "fifo <priority>" (only meaningful with paced) */
    if (argn > 5 && strcmp(args[4], "fifo") == 0) {
        int prio = atoi(args[5]);
        struct sched_param sp;
        sp.sched_priority = prio;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
            perror("sched_setscheduler(SCHED_FIFO) failed "
                   "(needs root, or an rtprio ulimit -- check: ulimit -r)");
            return 1;
        }
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

    if (is_bench) {
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
    } else if (is_paced) {
        /* Actually pace fmi2DoStep to wall-clock time -- same yield()-based
         * wait as bouncing_ball.cpp --paced (sleep_until had multi-ms tail
         * latency here; a pure busy-yield loop doesn't). */
        RuntimeContext ctx = get_runtime_context();
        print_runtime_context(&ctx);

        struct timespec next, prev;
        clock_gettime(CLOCK_MONOTONIC, &next);
        prev = next;
        long dt_ns = (long)(dt * 1e9);

        double* periods = malloc(sizeof(double) * n_steps);
        double t = 0.0;
        for (long i = 0; i < n_steps; ++i) {
            fmi2DoStep(c, t, dt, 1);
            t += dt;

            next.tv_nsec += dt_ns;
            while (next.tv_nsec >= 1000000000L) {
                next.tv_nsec -= 1000000000L;
                next.tv_sec += 1;
            }
            struct timespec now;
            do {
                sched_yield();
                clock_gettime(CLOCK_MONOTONIC, &now);
            } while (now.tv_sec < next.tv_sec ||
                     (now.tv_sec == next.tv_sec && now.tv_nsec < next.tv_nsec));

            double actual = now.tv_sec + now.tv_nsec * 1e-9;
            double prev_s = prev.tv_sec + prev.tv_nsec * 1e-9;
            periods[i] = actual - prev_s;
            prev = now;
        }

        double* sorted = malloc(sizeof(double) * n_steps);
        memcpy(sorted, periods, sizeof(double) * n_steps);
        qsort(sorted, n_steps, sizeof(double), cmp_double);

        double sum = 0.0;
        for (long i = 0; i < n_steps; ++i) sum += periods[i];
        double mean = sum / n_steps;

        printf("target step period: %.0f us   n=%ld\n", dt * 1e6, n_steps);
        const char* names[] = {"mean", "min", "p50", "p95", "p99", "p999", "max"};
        double vals[] = {mean, sorted[0], percentile(sorted, n_steps, 0.50),
                          percentile(sorted, n_steps, 0.95), percentile(sorted, n_steps, 0.99),
                          percentile(sorted, n_steps, 0.999), sorted[n_steps - 1]};
        for (int i = 0; i < 7; ++i) {
            printf("  %5s: %8.1f us   (target %+.1f us)\n", names[i], vals[i] * 1e6, (vals[i] - dt) * 1e6);
        }
        long n_over_2x = 0;
        for (long i = 0; i < n_steps; ++i) if (periods[i] > 2 * dt) ++n_over_2x;
        printf("  iterations > 2x target: %ld (%.3f%%)\n", n_over_2x, 100.0 * n_over_2x / n_steps);
        fmi2GetReal(c, vr_hv, 2, hv);
        printf("  final state: h=%.6f v=%.6f (printed so -O2 can't dead-code-eliminate the loop)\n", hv[0], hv[1]);

        if (json_out) {
            char label[96];
            if (strcmp(ctx.policy_name, "SCHED_FIFO") == 0) {
                snprintf(label, sizeof(label), "cpp-native-fmu-driver-%s+FIFO%d", ctx.kernel_release, ctx.priority);
            } else {
                snprintf(label, sizeof(label), "cpp-native-fmu-driver-%s", ctx.kernel_release);
            }
            write_json(json_out, label, ctx.is_rt_kernel, dt, n_steps, mean, sorted, periods);
        }
        free(periods);
        free(sorted);
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
