"""
Measure scheduling jitter of the paced (ChRealtimeStepTimer.Spin) headless
FMU loop -- what an RT (PREEMPT_RT) kernel is actually supposed to help
with, as opposed to the systematic rendering-induced drift found in
benchmark_realtime.py. Rendering is deliberately excluded (dynamics-only),
per the request that started this script.

Where benchmark_realtime.py's --pacing on reports one aggregate drift
number, this records every single loop period (time between consecutive
iterations) and reports percentiles/outliers -- the standard way
scheduling latency is characterized (cf. cyclictest), because a kernel
can have a fine *average* but a bad tail (rare multi-ms scheduling
stalls), which is exactly what PREEMPT_RT targets.

Results are tagged with `uname -r` and saved as JSON under rt_results/,
so you can run this once now (whichever kernel you're on), reboot into
the other kernel, run it again, and then compare both with
plot_rt_comparison.py -- across a reboot, so nothing here keeps a live
process running or does the reboot itself.

Run:
    python benchmark_rt_jitter.py                  # ~10s, saves rt_results/<kernel>.json
    python benchmark_rt_jitter.py --sim-time 20
"""
import argparse
import json
import os
import platform
import time

from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave

DEFAULT_FMU = os.path.join(os.path.dirname(__file__), "build", "BouncingBall.fmu")
RESULTS_DIR = os.path.join(os.path.dirname(__file__), "rt_results")


def make_fmu_instance(fmu_path):
    model_description = read_model_description(fmu_path)
    value_refs = {v.name: v.valueReference for v in model_description.modelVariables}
    unzip_dir = extract(fmu_path)
    fmu = FMU2Slave(
        guid=model_description.guid,
        unzipDirectory=unzip_dir,
        modelIdentifier=model_description.coSimulation.modelIdentifier,
        instanceName="rt_jitter_instance",
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()
    return fmu, value_refs


def percentile(sorted_vals, p):
    idx = min(int(len(sorted_vals) * p), len(sorted_vals) - 1)
    return sorted_vals[idx]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fmu", default=DEFAULT_FMU)
    parser.add_argument("--sim-time", type=float, default=10.0)
    parser.add_argument("--step-size", type=float, default=2e-3)
    parser.add_argument("--label", default=None,
                         help="override the auto-detected kernel label used for the output filename")
    args = parser.parse_args()

    if not os.path.exists(args.fmu):
        raise SystemExit(f"{args.fmu} not found -- build it first with pythonfmu build")

    import pychrono as chrono
    fmu, vr = make_fmu_instance(args.fmu)
    realtime_timer = chrono.ChRealtimeStepTimer()

    n_steps = int(args.sim_time / args.step_size)
    periods = []  # wall time between the start of consecutive iterations

    prev = time.perf_counter()
    t = 0.0
    for _ in range(n_steps):
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=args.step_size)
        t += args.step_size
        fmu.getReal([vr["h"]])
        realtime_timer.Spin(args.step_size)
        now = time.perf_counter()
        periods.append(now - prev)
        prev = now

    fmu.terminate()
    fmu.freeInstance()

    periods_sorted = sorted(periods)
    target = args.step_size
    stats = {
        "kernel": args.label or platform.uname().release,
        "is_realtime_kernel": os.path.exists("/sys/kernel/realtime"),
        "step_size": target,
        "n_samples": len(periods),
        "mean": sum(periods) / len(periods),
        "min": periods_sorted[0],
        "max": periods_sorted[-1],
        "p50": percentile(periods_sorted, 0.50),
        "p95": percentile(periods_sorted, 0.95),
        "p99": percentile(periods_sorted, 0.99),
        "p999": percentile(periods_sorted, 0.999),
        "periods": periods,  # full trace, for histogram plotting
    }

    print(f"kernel: {stats['kernel']}  (PREEMPT_RT: {stats['is_realtime_kernel']})")
    print(f"target loop period: {target * 1e6:.0f} us   n={stats['n_samples']}")
    for k in ("mean", "min", "p50", "p95", "p99", "p999", "max"):
        dev = (stats[k] - target) * 1e6
        print(f"  {k:>5}: {stats[k] * 1e6:8.1f} us   (target +{dev:+.1f} us)")

    n_over_2x = sum(1 for p in periods if p > 2 * target)
    n_over_5x = sum(1 for p in periods if p > 5 * target)
    print(f"  iterations > 2x target: {n_over_2x} ({100 * n_over_2x / len(periods):.3f}%)")
    print(f"  iterations > 5x target: {n_over_5x} ({100 * n_over_5x / len(periods):.3f}%)")

    os.makedirs(RESULTS_DIR, exist_ok=True)
    safe_label = stats["kernel"].replace("/", "_")
    out_path = os.path.join(RESULTS_DIR, f"{safe_label}.json")
    with open(out_path, "w") as f:
        json.dump(stats, f)
    print(f"\nSaved {out_path}")


if __name__ == "__main__":
    main()
