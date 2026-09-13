// Standalone C++ port of bouncing_ball_fmu.py -- same model, no FMU/FMI
// wrapper, no Chrono, no external dependencies at all. Two purposes:
//   1. A ground-truth reference to check the Python FMU's numbers against.
//   2. A raw compute-speed baseline: how fast can this same physics run
//      in native compiled C++, for comparison against benchmark_realtime.py's
//      "394x real-time, ~2us/do_step" headless (Python+FMI) result.
//
// Model (identical to bouncing_ball_fmu.py):
//   v += g*dt
//   h += v*dt
//   if h < floor: h = floor; v = -e*v
//
// Usage:
//   bouncing_ball --csv [sim_time] [dt]      print time,h,v to stdout
//   bouncing_ball --bench [sim_time] [dt]    flat-out speed, report ns/step and RTF
//   bouncing_ball --paced [sim_time] [dt]    actually pace to wall-clock time
//                                            (like benchmark_rt_jitter.py's Spin()
//                                            loop, not benchmark_realtime.py's RTF)
//                                            -- reports per-step period percentiles
//                                            -- prints the actual running kernel +
//                                            PREEMPT_RT status + this process's own
//                                            scheduling policy before it starts
//   bouncing_ball --paced [sim_time] [dt] fifo [priority]
//                                            same, but elevate to SCHED_FIFO first
//                                            (needs root or an rtprio ulimit)
//
// --bench (and plain --csv) run flat-out with no synchronization to wall-clock
// time at all -- useful for "how much compute headroom is there" but NOT a
// real-time-paced loop. --paced is the C++ equivalent of the Python
// ChRealtimeStepTimer.Spin() loop: sleep for most of each step's budget, then
// busy-spin the last ~150us for precision (plain sleep_until alone has Linux
// scheduler wake-up jitter well over 10us, even under PREEMPT_RT).
//
// Build:
//   g++ -O2 -o bouncing_ball bouncing_ball.cpp

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sched.h>
#include <sys/utsname.h>

// Prints the kernel this exact process is running under (not just what a
// separate `uname` in the shell reports) plus this process's own current
// scheduling policy, so a run's console output is self-verifying instead
// of having to trust a claim made outside the program.
static void print_runtime_context() {
    struct utsname u;
    uname(&u);
    FILE* f = fopen("/sys/kernel/realtime", "r");
    bool is_rt_kernel = false;
    if (f) {
        int val = 0;
        if (fscanf(f, "%d", &val) == 1) is_rt_kernel = (val == 1);
        fclose(f);
    }
    int policy = sched_getscheduler(0);
    const char* policy_name =
        policy == SCHED_FIFO ? "SCHED_FIFO" :
        policy == SCHED_RR ? "SCHED_RR" :
        policy == SCHED_OTHER ? "SCHED_OTHER" : "unknown";
    struct sched_param sp;
    sched_getparam(0, &sp);
    std::printf("kernel: %s %s  (PREEMPT_RT kernel: %s)  process sched: %s prio=%d\n",
                u.sysname, u.release, is_rt_kernel ? "yes" : "no", policy_name, sp.sched_priority);
}

struct State {
    double h;
    double v;
};

inline void step(State& s, double g, double e, double floor, double dt) {
    s.v += g * dt;
    s.h += s.v * dt;
    if (s.h < floor) {
        s.h = floor;
        s.v = -e * s.v;
    }
}

static void run_csv(double sim_time, double dt) {
    State s{10.0, 0.0};
    const double g = -9.81, e = 0.7, floor = 0.0;
    std::printf("time,h,v\n");
    for (double t = 0.0; t < sim_time; t += dt) {
        step(s, g, e, floor, dt);
        std::printf("%.6f,%.6f,%.6f\n", t + dt, s.h, s.v);
    }
}

static void run_bench(double sim_time, double dt) {
    State s{10.0, 0.0};
    const double g = -9.81, e = 0.7, floor = 0.0;
    long n_steps = static_cast<long>(sim_time / dt);

    auto wall_start = std::chrono::steady_clock::now();
    for (long i = 0; i < n_steps; ++i) {
        step(s, g, e, floor, dt);
    }
    auto wall_end = std::chrono::steady_clock::now();

    double wall_elapsed = std::chrono::duration<double>(wall_end - wall_start).count();
    double ns_per_step = wall_elapsed * 1e9 / n_steps;
    double rtf = wall_elapsed > 0 ? sim_time / wall_elapsed : 0.0;

    std::printf("n_steps: %ld\n", n_steps);
    std::printf("wall time: %.6f s\n", wall_elapsed);
    std::printf("ns/step: %.2f ns\n", ns_per_step);
    std::printf("real-time factor: %.1fx\n", rtf);
    std::printf("(final state, to prevent the optimizer from discarding the loop: h=%.6f v=%.6f)\n",
                 s.h, s.v);
}

static double percentile(std::vector<double>& sorted_vals, double p) {
    size_t idx = std::min(static_cast<size_t>(sorted_vals.size() * p), sorted_vals.size() - 1);
    return sorted_vals[idx];
}

static void run_paced(double sim_time, double dt) {
    using clock = std::chrono::steady_clock;
    print_runtime_context();
    State s{10.0, 0.0};
    const double g = -9.81, e = 0.7, floor = 0.0;
    long n_steps = static_cast<long>(sim_time / dt);
    const auto dt_dur = std::chrono::duration<double>(dt);
    const auto spin_margin = std::chrono::microseconds(150);  // busy-spin the last bit for precision

    std::vector<double> periods;
    periods.reserve(n_steps);

    auto next = clock::now();
    auto prev = next;
    for (long i = 0; i < n_steps; ++i) {
        step(s, g, e, floor, dt);

        next += std::chrono::duration_cast<clock::duration>(dt_dur);
        auto now = clock::now();
        if (next > now) {
            if (next - now > spin_margin) {
                std::this_thread::sleep_until(next - spin_margin);
            }
            while (clock::now() < next) {
                // busy-spin for sub-microsecond precision near the deadline
            }
        }
        auto actual = clock::now();
        periods.push_back(std::chrono::duration<double>(actual - prev).count());
        prev = actual;
    }

    std::vector<double> sorted_periods = periods;
    std::sort(sorted_periods.begin(), sorted_periods.end());

    double sum = 0.0;
    for (double p : periods) sum += p;
    double mean = sum / periods.size();

    std::printf("target step period: %.0f us   n=%ld\n", dt * 1e6, n_steps);
    auto print_metric = [&](const char* name, double val) {
        std::printf("  %5s: %8.1f us   (target %+.1f us)\n", name, val * 1e6, (val - dt) * 1e6);
    };
    print_metric("mean", mean);
    print_metric("min", sorted_periods.front());
    print_metric("p50", percentile(sorted_periods, 0.50));
    print_metric("p95", percentile(sorted_periods, 0.95));
    print_metric("p99", percentile(sorted_periods, 0.99));
    print_metric("p999", percentile(sorted_periods, 0.999));
    print_metric("max", sorted_periods.back());

    long n_over_2x = 0;
    for (double p : periods) if (p > 2 * dt) ++n_over_2x;
    std::printf("  iterations > 2x target: %ld (%.3f%%)\n", n_over_2x,
                100.0 * n_over_2x / periods.size());
}

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "--csv";
    double sim_time = argc > 2 ? std::stod(argv[2]) : 8.0;
    double dt = argc > 3 ? std::stod(argv[3]) : 0.002;

    // optional trailing "fifo <priority>" args (only meaningful with --paced),
    // e.g.: bouncing_ball --paced 10 0.002 fifo 10
    if (argc > 4 && std::string(argv[4]) == "fifo") {
        int prio = argc > 5 ? std::atoi(argv[5]) : 10;
        struct sched_param sp;
        sp.sched_priority = prio;
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
            std::perror("sched_setscheduler(SCHED_FIFO) failed "
                        "(needs root, or an rtprio ulimit -- check: ulimit -r)");
            return 1;
        }
    }

    if (mode == "--csv") {
        run_csv(sim_time, dt);
    } else if (mode == "--bench") {
        run_bench(sim_time, dt);
    } else if (mode == "--paced") {
        run_paced(sim_time, dt);
    } else {
        std::fprintf(stderr, "usage: %s [--csv|--bench|--paced] [sim_time] [dt] [fifo priority]\n", argv[0]);
        return 1;
    }
    return 0;
}
