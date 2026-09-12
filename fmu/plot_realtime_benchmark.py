"""
Run benchmark_realtime.py --pacing on --render both --trace-out ... and plot
the resulting (sim_time, wall_time) traces: does wall-clock time actually
track simulated time (a y=x diagonal), and how does the drift build up over
the course of the run?

Run:
    python plot_realtime_benchmark.py
    python plot_realtime_benchmark.py --sim-time 8
"""
import argparse
import csv
import os
import subprocess
import sys
import tempfile

import matplotlib.pyplot as plt

DEFAULT_FMU = os.path.join(os.path.dirname(__file__), "build", "BouncingBall.fmu")


def load_trace(path):
    sim_t, wall_t = [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            sim_t.append(float(row["sim_time"]))
            wall_t.append(float(row["wall_time"]))
    return sim_t, wall_t


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fmu", default=DEFAULT_FMU)
    parser.add_argument("--sim-time", type=float, default=5.0)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory() as tmpdir:
        trace_base = os.path.join(tmpdir, "trace")
        subprocess.run(
            [sys.executable, os.path.join(os.path.dirname(__file__), "benchmark_realtime.py"),
             "--fmu", args.fmu, "--sim-time", str(args.sim_time),
             "--pacing", "on", "--render", "both", "--trace-out", trace_base],
            check=True,
        )
        sim_h, wall_h = load_trace(trace_base + ".headless.csv")
        sim_r, wall_r = load_trace(trace_base + ".render.csv")

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    max_t = max(sim_h[-1], sim_r[-1])
    ax.plot([0, max_t], [0, max_t], color="0.6", linestyle="--", linewidth=1, label="perfect sync (y=x)")
    ax.plot(sim_h, wall_h, color="tab:blue", linewidth=2, label="headless")
    ax.plot(sim_r, wall_r, color="tab:red", linewidth=2, label="with rendering")
    ax.set_title("Wall time vs simulated time (paced)")
    ax.set_xlabel("simulated time [s]")
    ax.set_ylabel("wall time [s]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_xlim(0, max_t)
    ax.set_ylim(0, max(wall_h[-1], wall_r[-1]) * 1.05)

    ax = axes[1]
    ax.axhline(0, color="0.6", linewidth=1, linestyle="--")
    drift_h = [w - s for s, w in zip(sim_h, wall_h)]
    drift_r = [w - s for s, w in zip(sim_r, wall_r)]
    ax.plot(sim_h, drift_h, color="tab:blue", linewidth=2, label="headless")
    ax.plot(sim_r, drift_r, color="tab:red", linewidth=2, label="with rendering")
    ax.set_title("Drift (wall - sim) over the run")
    ax.set_xlabel("simulated time [s]")
    ax.set_ylabel("drift [s]  (+ = running slow)")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.suptitle(f"Pacing accuracy ({os.path.basename(args.fmu)}, ChRealtimeStepTimer.Spin)", fontsize=13)
    fig.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "realtime_pacing.png")
    fig.savefig(out_path, dpi=130)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
