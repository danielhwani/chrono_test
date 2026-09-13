"""
Validate bouncing_ball_native.fmu the same way validate_bouncing_ball.py
checks the pythonfmu-built one: load with fmpy, simulate, and confirm the
floor is never penetrated and consecutive bounce peak heights decay by
e^2 (there's no single closed-form h(t) once bouncing starts).

Run:
    ./build.sh                       # (re)build first
    python validate_native_fmu.py
"""
import os

import fmpy

FMU_PATH = os.path.join(os.path.dirname(__file__), "bouncing_ball_native.fmu")
E = 0.7
FLOOR = 0.0
RATIO_TOLERANCE = 0.03


def main():
    if not os.path.exists(FMU_PATH):
        raise SystemExit(f"{FMU_PATH} not found -- run ./build.sh first")

    info = fmpy.read_model_description(FMU_PATH)
    print(f"Loaded model '{info.modelName}' -- variables: "
          f"{[v.name for v in info.modelVariables]}")

    result = fmpy.simulate_fmu(
        FMU_PATH, start_time=0.0, stop_time=8.0, output_interval=1e-4,
        start_values={"e": E, "floor": FLOOR},
        output=["h", "v"],
    )
    h = result["h"]
    t = result["time"]

    worst_penetration = FLOOR - h.min()
    print(f"min h = {h.min():.6f} (floor={FLOOR}) -- worst penetration: {max(worst_penetration, 0):.6f}")
    if h.min() < FLOOR - 1e-6:
        raise SystemExit(f"FAIL: ball penetrated the floor by {worst_penetration:.6f}")

    peaks = []
    for i in range(1, len(h) - 1):
        if h[i] > h[i - 1] and h[i] > h[i + 1] and h[i] > FLOOR + 1e-3:
            peaks.append((t[i], h[i]))

    print(f"found {len(peaks)} bounce peaks")
    if len(peaks) < 3:
        raise SystemExit("FAIL: expected at least 3 bounce peaks to check decay ratio")

    expected_ratio = E ** 2
    worst_dev = 0.0
    for i in range(1, min(len(peaks), 5)):
        ratio = (peaks[i][1] - FLOOR) / (peaks[i - 1][1] - FLOOR)
        dev = abs(ratio - expected_ratio)
        worst_dev = max(worst_dev, dev)
        print(f"  peak {i}: height ratio = {ratio:.4f}  (expected e^2 = {expected_ratio:.4f}, dev {dev:.4f})")

    if worst_dev > RATIO_TOLERANCE:
        raise SystemExit(f"FAIL: peak-height ratio deviates from e^2 by {worst_dev:.4f}")

    print(f"\nOK -- floor respected, bounce peak heights decay by ~e^2={expected_ratio:.4f} as expected. "
          f"Native FMI2 C API implementation verified against the same physics as the Python FMU.")


if __name__ == "__main__":
    main()
