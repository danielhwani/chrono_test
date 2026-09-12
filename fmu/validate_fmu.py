"""
Validate that free_fall_fmu.py's built FMU actually works: load it with
fmpy, simulate it, and check the result against the analytical free-fall
solution (h(t) = h0 + 0.5*g*t^2, v(t) = g*t).

This is a toolchain smoke test (pythonfmu build -> .fmu -> fmpy load and
simulate), not a physics check -- but comparing against the closed-form
solution also catches step-size mistakes, sign errors, etc. A co-simulation
FMU's do_step is called once per `output_interval`, so a small interval is
needed for the result to actually converge to the analytical curve (this
is expected discretization error, not a bug -- see free_fall_fmu.py).

Run:
    pythonfmu build -f free_fall_fmu.py -d build   # (re)build first
    python validate_fmu.py
"""
import os

import fmpy

FMU_PATH = os.path.join(os.path.dirname(__file__), "build", "FreeFall.fmu")
H0 = 10.0
G = -9.81
TOLERANCE = 0.01  # meters, at output_interval=1e-4


def main():
    if not os.path.exists(FMU_PATH):
        raise SystemExit(f"{FMU_PATH} not found -- run: pythonfmu build -f free_fall_fmu.py -d build")

    info = fmpy.read_model_description(FMU_PATH)
    print(f"Loaded model '{info.modelName}' -- variables: "
          f"{[v.name for v in info.modelVariables]}")

    result = fmpy.simulate_fmu(
        FMU_PATH, start_time=0.0, stop_time=2.0, output_interval=1e-4,
        output=["h", "v"],
    )

    worst = 0.0
    for t_target in (0.0, 0.5, 1.0, 1.5, 2.0):
        idx = abs(result["time"] - t_target).argmin()
        row = result[idx]
        t = row["time"]
        h_expected = H0 + 0.5 * G * t * t
        v_expected = G * t
        h_err = abs(row["h"] - h_expected)
        v_err = abs(row["v"] - v_expected)
        worst = max(worst, h_err, v_err)
        print(f"t={t:5.3f}  h={row['h']:9.5f} (expected {h_expected:9.5f}, err {h_err:.6f})  "
              f"v={row['v']:9.5f} (expected {v_expected:9.5f}, err {v_err:.6f})")

    if worst > TOLERANCE:
        raise SystemExit(f"FAIL: worst error {worst:.6f} exceeds tolerance {TOLERANCE}")
    print(f"\nOK -- worst error {worst:.6f} within tolerance {TOLERANCE}. "
          f"pythonfmu toolchain (build -> load -> simulate) verified.")


if __name__ == "__main__":
    main()
