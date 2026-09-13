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
//   bouncing_ball --bench [sim_time] [dt]    time N steps, report ns/step and RTF
//
// Build:
//   g++ -O2 -o bouncing_ball bouncing_ball.cpp

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

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

int main(int argc, char** argv) {
    std::string mode = argc > 1 ? argv[1] : "--csv";
    double sim_time = argc > 2 ? std::stod(argv[2]) : 8.0;
    double dt = argc > 3 ? std::stod(argv[3]) : 0.002;

    if (mode == "--csv") {
        run_csv(sim_time, dt);
    } else if (mode == "--bench") {
        run_bench(sim_time, dt);
    } else {
        std::fprintf(stderr, "usage: %s [--csv|--bench] [sim_time] [dt]\n", argv[0]);
        return 1;
    }
    return 0;
}
