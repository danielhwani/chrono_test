/*
 * fmu_driver -- a minimal C "master"/host for an FMI2 Co-Simulation FMU,
 * built on top of fmu_client (dlopen()/dlsym()-based, no fmpy/Python
 * anywhere). fmu_driver.c itself is now just CLI argv parsing and the
 * bench/csv/paced mode loops -- all the FMI-loading plumbing (dlopen the
 * .so, parse modelDescription.xml, instantiate/setupExperiment/
 * enter+exitInitializationMode, setReal/getReal/doStep/close) lives in
 * fmu_client.c/.h, shared with the planned ros2_control
 * ChronoFmuSystemInterface plugin so that logic isn't duplicated.
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
 * fmu_driver.c is the *caller*: it loads that .so (via fmu_client) and
 * calls its FMI2 entry points, the same role fmpy plays in
 * validate_native_fmu.py and benchmark_realtime.py, but with zero Python
 * anywhere in the process. That completes the fully-native path: native
 * FMU + native driver.
 *
 * Usage:
 *   ./fmu_driver <extracted-fmu-dir> bench  <sim_time_s> <dt_s> [options]
 *   ./fmu_driver <extracted-fmu-dir> csv    <sim_time_s> <dt_s> [options]
 *   ./fmu_driver <extracted-fmu-dir> paced  <sim_time_s> <dt_s> [fifo <priority>] [options]
 *
 *   options:
 *     --outputs name1,name2,...   Real variables to read/print each step
 *                                  (default: h,v -- the bouncing-ball models'
 *                                  outputs, kept as the default so existing
 *                                  invocations are unaffected)
 *     --set name=value            set a Real input once, right after
 *                                  exitInitializationMode, before stepping
 *                                  starts (repeatable, e.g. two --set flags
 *                                  for two different inputs). Held constant
 *                                  for the whole run -- there's no per-step
 *                                  time-varying input support here; a real
 *                                  driving scenario (a steer ramp, a control
 *                                  loop) belongs in a purpose-built driver or
 *                                  control layer, not this generic one.
 *     --json-out PATH             (paced only) write rt_results/*.json-
 *                                  schema percentiles, see below
 *
 * <extracted-fmu-dir> is a directory containing modelDescription.xml and
 * binaries/linux64/<modelIdentifier>.so -- i.e. an FMU already unzipped (a
 * .fmu file IS just that layout zipped up). For our own native_fmu, that's
 * the native_fmu/ directory itself (build.sh leaves the unzipped layout in
 * place alongside the .fmu it also produces). For a third-party .fmu, unzip
 * it first: `mkdir foo_extracted && unzip foo.fmu -d foo_extracted`.
 *
 * This driver is deliberately NOT specific to our own model: fmu_client
 * reads the FMU's own modelDescription.xml at startup to find the GUID, the
 * modelIdentifier (so it knows which .so to dlopen), and the value
 * references of whichever Real variables --outputs/--set name. Any FMI2
 * Co-Simulation FMU works here unmodified as long as the variables named
 * exist -- e.g. fmu/cpp/native_fmu/ or fmu/modelica/BouncingBallModelica.fmu
 * (h, v -- the default), or fmu/cpp/native_vehicle_fmu/ (steer_deg,
 * drive_torque_rear as --set inputs; chassis_x, yaw_deg, etc. as --outputs),
 * which don't share a GUID or variable numbering with any of the others.
 *
 *   bench -- flat-out fmi2DoStep loop (no pacing), reports ns/step and RTF.
 *            Compare directly against `bouncing_ball --bench` to see the
 *            overhead the FMI function-pointer layer adds over raw C++.
 *   csv   -- prints time,<outputs...> rows to stdout (redirect to a file to
 *            compare against bouncing_ball.cpp --csv / the Python FMU's
 *            output, for the default h,v case).
 *   paced -- actually paces fmi2DoStep to wall-clock time, same yield()-based
 *            wait as bouncing_ball.cpp --paced (see that file's comment for
 *            why: sleep_until() alone had multi-ms tail latency here, a pure
 *            busy-yield loop doesn't). Reports per-step period percentiles
 *            and can write the same JSON schema as rt_results/*.json via
 *            --json-out, so this overlays directly with the existing Python
 *            (fmpy) and no-FMU C++ paced results in plot_rt_comparison.py.
 *
 * Examples:
 *   ./fmu_driver ../../native_vehicle_fmu csv 3 0.002 \
 *       --set steer_deg=15 --set drive_torque_rear=260 \
 *       --outputs chassis_x,chassis_y,yaw_deg,speed_mps
 *
 * Build: see ./build.sh (compiles fmu_client.c alongside this file, -ldl).
 */
#include "fmu_client.h"

#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/utsname.h>
#include <time.h>

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

#define MAX_VARS 16

int main(int argc, char** argv) {
    /* pull "--json-out PATH", "--outputs a,b,c" and (repeatable) "--set
     * name=value" out wherever they appear, then treat the rest positionally */
    char* json_out = NULL;
    char* outputs_arg = NULL;
    char* set_names[MAX_VARS];
    double set_values[MAX_VARS];
    int n_sets = 0;

    int argn = argc - 1;
    char** args = argv + 1;
    for (int i = 0; i < argn; ) {
        if (strcmp(args[i], "--json-out") == 0 && i + 1 < argn) {
            json_out = args[i + 1];
            for (int j = i; j + 2 < argn; ++j) args[j] = args[j + 2];
            argn -= 2;
        } else if (strcmp(args[i], "--outputs") == 0 && i + 1 < argn) {
            outputs_arg = args[i + 1];
            for (int j = i; j + 2 < argn; ++j) args[j] = args[j + 2];
            argn -= 2;
        } else if (strcmp(args[i], "--set") == 0 && i + 1 < argn) {
            if (n_sets >= MAX_VARS) {
                fprintf(stderr, "too many --set flags (max %d)\n", MAX_VARS);
                return 1;
            }
            char* eq = strchr(args[i + 1], '=');
            if (!eq) {
                fprintf(stderr, "--set expects name=value, got '%s'\n", args[i + 1]);
                return 1;
            }
            size_t name_len = (size_t)(eq - args[i + 1]);
            char* name = malloc(name_len + 1);
            memcpy(name, args[i + 1], name_len);
            name[name_len] = '\0';
            set_names[n_sets] = name;
            set_values[n_sets] = atof(eq + 1);
            n_sets++;
            for (int j = i; j + 2 < argn; ++j) args[j] = args[j + 2];
            argn -= 2;
        } else {
            ++i;
        }
    }

    if (argn < 4) {
        fprintf(stderr, "usage: %s <extracted-fmu-dir> {bench|csv|paced} <sim_time_s> <dt_s> "
                        "[fifo priority] [--outputs a,b,c] [--set name=value ...] [--json-out PATH]\n",
                argv[0]);
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

    FmuClient* client = fmu_client_open(fmu_dir, "fmu_driver_instance");
    if (!client) {
        return 1;  /* fmu_client_open already printed a diagnostic */
    }

    /* --outputs a,b,c (default: h,v, for backward compatibility with the
     * bouncing-ball models this driver was originally written for) */
    const char* default_outputs[2] = {"h", "v"};
    char* output_names[MAX_VARS];
    int n_outputs = 0;
    if (outputs_arg) {
        char* tok = strtok(outputs_arg, ",");
        while (tok && n_outputs < MAX_VARS) {
            output_names[n_outputs++] = tok;
            tok = strtok(NULL, ",");
        }
    } else {
        for (int i = 0; i < 2; ++i) output_names[n_outputs++] = (char*)default_outputs[i];
    }

    FmuValueReference vr_out[MAX_VARS];
    for (int i = 0; i < n_outputs; ++i) {
        if (!fmu_client_find_vr(client, output_names[i], &vr_out[i])) {
            fprintf(stderr, "could not find output variable '%s' in %s/modelDescription.xml\n",
                    output_names[i], fmu_dir);
            fmu_client_close(client);
            return 1;
        }
    }

    FmuValueReference vr_set[MAX_VARS];
    for (int i = 0; i < n_sets; ++i) {
        if (!fmu_client_find_vr(client, set_names[i], &vr_set[i])) {
            fprintf(stderr, "could not find --set variable '%s' in %s/modelDescription.xml\n",
                    set_names[i], fmu_dir);
            fmu_client_close(client);
            return 1;
        }
    }

    fprintf(stderr, "model: %s  guid=%s  outputs:", fmu_client_model_id(client), fmu_client_guid(client));
    for (int i = 0; i < n_outputs; ++i) fprintf(stderr, " %s=vr%u", output_names[i], vr_out[i]);
    if (n_sets > 0) {
        fprintf(stderr, "  set:");
        for (int i = 0; i < n_sets; ++i) fprintf(stderr, " %s=%g", set_names[i], set_values[i]);
    }
    fprintf(stderr, "\n");

    if (n_sets > 0) {
        fmu_client_set_real(client, vr_set, n_sets, set_values);
    }

    long n_steps = (long)(sim_time / dt);
    double out_vals[MAX_VARS];

    if (is_bench) {
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        double t = 0.0;
        for (long i = 0; i < n_steps; ++i) {
            fmu_client_do_step(client, t, dt);
            t += dt;
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) * 1e-9;
        fmu_client_get_real(client, vr_out, n_outputs, out_vals);
        double ns_per_step = elapsed * 1e9 / n_steps;
        double rtf = sim_time / elapsed;
        printf("fmu_driver bench: %ld steps, dt=%g, sim_time=%g s\n", n_steps, dt, sim_time);
        printf("  wall elapsed: %.6f s\n", elapsed);
        printf("  ns/step:      %.2f\n", ns_per_step);
        printf("  RTF:          %.1fx realtime\n", rtf);
        printf("  final state:  ");
        for (int i = 0; i < n_outputs; ++i) printf("%s=%.6f ", output_names[i], out_vals[i]);
        printf("(printed so -O2 can't dead-code-eliminate the loop)\n");
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
            fmu_client_do_step(client, t, dt);
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
        fmu_client_get_real(client, vr_out, n_outputs, out_vals);
        printf("  final state: ");
        for (int i = 0; i < n_outputs; ++i) printf("%s=%.6f ", output_names[i], out_vals[i]);
        printf("(printed so -O2 can't dead-code-eliminate the loop)\n");

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
        printf("time");
        for (int i = 0; i < n_outputs; ++i) printf(",%s", output_names[i]);
        printf("\n");

        fmu_client_get_real(client, vr_out, n_outputs, out_vals);
        printf("%.6f", t);
        for (int i = 0; i < n_outputs; ++i) printf(",%.6f", out_vals[i]);
        printf("\n");
        for (long i = 0; i < n_steps; ++i) {
            fmu_client_do_step(client, t, dt);
            t += dt;
            fmu_client_get_real(client, vr_out, n_outputs, out_vals);
            printf("%.6f", t);
            for (int j = 0; j < n_outputs; ++j) printf(",%.6f", out_vals[j]);
            printf("\n");
        }
    }

    fmu_client_close(client);
    return 0;
}
