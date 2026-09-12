"""
Simulate the built FreeFall FMU with fmpy and plot h(t)/v(t) against the
analytical free-fall solution.

Run:
    pythonfmu build -f free_fall_fmu.py -d build   # if not already built
    python plot_free_fall.py
"""
import os

import fmpy
import matplotlib.pyplot as plt

FMU_PATH = os.path.join(os.path.dirname(__file__), "build", "FreeFall.fmu")
H0 = 10.0
G = -9.81


def main():
    if not os.path.exists(FMU_PATH):
        raise SystemExit(f"{FMU_PATH} not found -- run: pythonfmu build -f free_fall_fmu.py -d build")

    result = fmpy.simulate_fmu(
        FMU_PATH, start_time=0.0, stop_time=2.0, output_interval=1e-3,
        output=["h", "v"],
    )
    t = result["time"]
    h = result["h"]
    v = result["v"]
    h_analytic = H0 + 0.5 * G * t * t
    v_analytic = G * t

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

    ax = axes[0]
    ax.axhline(0, color="0.6", linewidth=1, linestyle="--", label="ground (h=0)")
    ax.plot(t, h_analytic, color="0.4", linewidth=5, alpha=0.3, label="analytic")
    ax.plot(t, h, color="tab:blue", linewidth=2, label="FMU (h)")
    ax.set_title("Height vs time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("h [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1]
    ax.axhline(0, color="0.6", linewidth=1, linestyle="--")
    ax.plot(t, v_analytic, color="0.4", linewidth=5, alpha=0.3, label="analytic")
    ax.plot(t, v, color="tab:red", linewidth=2, label="FMU (v)")
    ax.set_title("Velocity vs time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("v [m/s]")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.suptitle("FreeFall.fmu vs analytical free fall", fontsize=13)
    fig.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "free_fall.png")
    fig.savefig(out_path, dpi=130)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    main()
