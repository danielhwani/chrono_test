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
 *   ./fmu_driver <extracted-fmu-dir> bench  <sim_time_s> <dt_s>
 *   ./fmu_driver <extracted-fmu-dir> csv    <sim_time_s> <dt_s>
 *   ./fmu_driver <extracted-fmu-dir> paced  <sim_time_s> <dt_s> [fifo <priority>] [--json-out PATH]
 *
 * <extracted-fmu-dir> is a directory containing modelDescription.xml and
 * binaries/linux64/<modelIdentifier>.so -- i.e. an FMU already unzipped (a
 * .fmu file IS just that layout zipped up). For our own native_fmu, that's
 * the native_fmu/ directory itself (build.sh leaves the unzipped layout in
 * place alongside the .fmu it also produces). For a third-party .fmu, unzip
 * it first: `mkdir foo_extracted && unzip foo.fmu -d foo_extracted`.
 *
 * This driver is deliberately NOT specific to our own model: it reads the
 * FMU's own modelDescription.xml at startup to find the GUID, the
 * modelIdentifier (so it knows which .so to dlopen), and the value
 * references of the two variables it knows how to drive/print: "h" and
 * "v". Any FMI2 Co-Simulation FMU exposing Real variables literally named
 * "h" and "v" works here unmodified -- e.g. fmu/cpp/native_fmu/ (our
 * hand-written one) or fmu/modelica/BouncingBallModelica.fmu (built by
 * OpenModelica's omc), which don't share a GUID or variable numbering with
 * each other at all.
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
#include <limits.h>
#include <sched.h>
#include <stdarg.h>
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

/* ---- tiny modelDescription.xml scanner ----
 * Not a real XML parser -- just enough to pull `attr="value"` out of the
 * well-formed, single-line-per-attribute XML that both our own hand-written
 * modelDescription.xml and OpenModelica's omc-generated one use. Searches
 * forward from `search_from` for `attr_name="`, copies out everything up to
 * the closing quote. Returns 0 (and leaves out untouched) if not found. */
static int find_attr_value(const char* search_from, const char* attr_name, char* out, size_t out_size) {
    char needle[64];
    snprintf(needle, sizeof(needle), "%s=\"", attr_name);
    const char* p = strstr(search_from, needle);
    if (!p) return 0;
    p += strlen(needle);
    const char* q = strchr(p, '"');
    if (!q) return 0;
    size_t len = (size_t)(q - p);
    if (len >= out_size) len = out_size - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    return 1;
}

/* Real callback functions instead of NULL: our own hand-written FMU tolerates
 * a NULL fmi2CallbackFunctions* (falls back to malloc), but the FMI2 spec
 * doesn't actually make these optional, and OpenModelica-generated FMUs
 * dereference them unconditionally (e.g. to log during instantiation) --
 * passing NULL segfaults inside their fmi2Instantiate. fmpy always supplies
 * real callbacks for the same reason. */
static void fmu_logger(fmi2ComponentEnvironment env, fmi2String instanceName, fmi2Status status,
                        fmi2String category, fmi2String message, ...) {
    (void)env; (void)status;
    va_list ap;
    va_start(ap, message);
    fprintf(stderr, "[%s|%s] ", instanceName ? instanceName : "?", category ? category : "?");
    vfprintf(stderr, message, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static void* fmu_alloc(size_t nobj, size_t size) { return calloc(nobj, size); }
static void fmu_free(void* p) { free(p); }

static char* read_whole_file(const char* path) {
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* buf = malloc((size_t)size + 1);
    if (fread(buf, 1, (size_t)size, f) != (size_t)size) {
        fclose(f);
        free(buf);
        return NULL;
    }
    buf[size] = '\0';
    fclose(f);
    return buf;
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
        fprintf(stderr, "usage: %s <extracted-fmu-dir> {bench|csv|paced} <sim_time_s> <dt_s> "
                        "[fifo priority] [--json-out PATH]\n", argv[0]);
        return 1;
    }
    const char* fmu_dir = args[0];
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

    char xml_path[1024];
    snprintf(xml_path, sizeof(xml_path), "%s/modelDescription.xml", fmu_dir);
    char* xml = read_whole_file(xml_path);
    if (!xml) {
        fprintf(stderr, "could not read %s (pass a directory with modelDescription.xml and "
                        "binaries/linux64/ in it -- unzip a .fmu first if needed)\n", xml_path);
        return 1;
    }

    char guid[128], model_id[128], vr_h_str[32], vr_v_str[32];
    const char* cosim_tag = strstr(xml, "<CoSimulation");
    const char* h_tag = strstr(xml, "name=\"h\"");
    const char* v_tag = strstr(xml, "name=\"v\"");
    if (!find_attr_value(xml, "guid", guid, sizeof(guid)) ||
        !cosim_tag || !find_attr_value(cosim_tag, "modelIdentifier", model_id, sizeof(model_id)) ||
        !h_tag || !find_attr_value(h_tag, "valueReference", vr_h_str, sizeof(vr_h_str)) ||
        !v_tag || !find_attr_value(v_tag, "valueReference", vr_v_str, sizeof(vr_v_str))) {
        fprintf(stderr, "could not parse guid/modelIdentifier/h/v out of %s -- this driver "
                        "expects Real variables literally named 'h' and 'v'\n", xml_path);
        free(xml);
        return 1;
    }
    fmi2ValueReference vr_hv[2] = {(fmi2ValueReference)atoi(vr_h_str), (fmi2ValueReference)atoi(vr_v_str)};
    free(xml);

    char so_path[1024];
    snprintf(so_path, sizeof(so_path), "%s/binaries/linux64/%s.so", fmu_dir, model_id);

    void* handle = dlopen(so_path, RTLD_NOW);
    if (!handle) {
        fprintf(stderr, "dlopen(%s) failed: %s\n", so_path, dlerror());
        return 1;
    }
    fprintf(stderr, "model: %s  guid=%s  h=vr%s v=vr%s\n", model_id, guid, vr_h_str, vr_v_str);

    char abs_dir[PATH_MAX];
    char resource_uri[PATH_MAX + 32] = "";
    if (realpath(fmu_dir, abs_dir)) {
        snprintf(resource_uri, sizeof(resource_uri), "file://%s/resources", abs_dir);
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

    fmi2CallbackFunctions callbacks = {fmu_logger, fmu_alloc, fmu_free, NULL, NULL};
    fmi2Component c = fmi2Instantiate("fmu_driver_instance", fmi2CoSimulation, guid, resource_uri,
                                       &callbacks, 0, 0);
    if (!c) {
        fprintf(stderr, "fmi2Instantiate returned NULL\n");
        return 1;
    }
    fmi2SetupExperiment(c, 0, 0.0, 0.0, 0, 0.0);
    fmi2EnterInitializationMode(c);
    fmi2ExitInitializationMode(c);

    long n_steps = (long)(sim_time / dt);
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
