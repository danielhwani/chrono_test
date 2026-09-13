"""
Plot every rt_results/*.json produced by benchmark_rt_jitter.py together --
one histogram of loop-period jitter per kernel, so an RT (PREEMPT_RT) vs
generic kernel run can be compared at a glance once both exist.

Works with just one result file too (run it again after collecting a
second kernel's data to get the comparison).

Run:
    python plot_rt_comparison.py
"""
import glob
import json
import os

import matplotlib.pyplot as plt

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "rt_results")
COLORS = ["tab:blue", "tab:red", "tab:green", "tab:orange", "tab:purple", "tab:brown", "tab:cyan", "tab:pink"]


def main():
    paths = sorted(glob.glob(os.path.join(RESULTS_DIR, "*.json")))
    if not paths:
        raise SystemExit(f"No results in {RESULTS_DIR} -- run benchmark_rt_jitter.py first")

    results = []
    for p in paths:
        with open(p) as f:
            results.append(json.load(f))

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    ax = axes[0]
    for i, r in enumerate(results):
        periods_us = [p * 1e6 for p in r["periods"]]
        label = f"{r['kernel']}{' (RT)' if r['is_realtime_kernel'] else ' (generic)'}"
        ax.hist(periods_us, bins=80, alpha=0.5, color=COLORS[i % len(COLORS)], label=label,
                density=True)
    target_us = results[0]["step_size"] * 1e6
    ax.axvline(target_us, color="0.3", linestyle="--", linewidth=1, label=f"target ({target_us:.0f} us)")
    ax.set_title("Loop period distribution")
    ax.set_xlabel("period [us]")
    ax.set_ylabel("density")
    ax.set_yscale("log")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    metrics = ["mean", "p50", "p95", "p99", "p999", "max"]
    x = range(len(metrics))
    width = 0.8 / len(results)
    for i, r in enumerate(results):
        offsets = [r[m] * 1e6 - r["step_size"] * 1e6 for m in metrics]
        label = f"{r['kernel']}{' (RT)' if r['is_realtime_kernel'] else ' (generic)'}"
        ax.bar([xi + i * width for xi in x], offsets, width=width,
               color=COLORS[i % len(COLORS)], label=label)
    ax.set_xticks([xi + width * (len(results) - 1) / 2 for xi in x])
    ax.set_xticklabels(metrics)
    ax.set_title("Deviation from target step size, by percentile")
    ax.set_ylabel("deviation [us]  (log scale)")
    ax.set_yscale("log")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")

    fig.suptitle("Headless FMU loop jitter (ChRealtimeStepTimer.Spin, no rendering)", fontsize=13)
    fig.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "rt_comparison.png")
    fig.savefig(out_path, dpi=130)
    print(f"Saved {out_path}")
    if len(results) == 1:
        print("Only one kernel's results found so far -- reboot into the other kernel, "
              "run benchmark_rt_jitter.py again, then re-run this script for the comparison.")


if __name__ == "__main__":
    main()
