"""
Validate BouncingBall.fmu: load it with fmpy, simulate, and check that
consecutive bounce peak heights (above the floor) shrink by a factor of
e^2 each time -- that's what a coefficient-of-restitution model should do
(v_after = -e*v_before at impact, so kinetic energy, and therefore the
next peak height, scales by e^2). There's no single closed-form h(t) once
bouncing starts, so this checks that physical invariant instead (plus
that h never dips below the floor).

Run:
    pythonfmu build -f bouncing_ball_fmu.py -d build   # (re)build first
    python validate_bouncing_ball.py
"""
import os

import fmpy

FMU_PATH = os.path.join(os.path.dirname(__file__), "build", "BouncingBall.fmu")
E = 0.7
FLOOR = 0.0
RATIO_TOLERANCE = 0.03  # allow 3% deviation from the ideal e^2 peak-height ratio


def main():
    if not os.path.exists(FMU_PATH):
        raise SystemExit(f"{FMU_PATH} not found -- run: pythonfmu build -f bouncing_ball_fmu.py -d build")

    result = fmpy.simulate_fmu(
        FMU_PATH, start_time=0.0, stop_time=8.0, output_interval=1e-4,
        start_values={"e": E, "floor": FLOOR},
        output=["h", "v"],
    )
    h = result["h"]
    t = result["time"]

    # never penetrates the floor
    worst_penetration = FLOOR - h.min()
    print(f"min h = {h.min():.6f} (floor={FLOOR}) -- worst penetration: {max(worst_penetration, 0):.6f}")
    if h.min() < FLOOR - 1e-6:
        raise SystemExit(f"FAIL: ball penetrated the floor by {worst_penetration:.6f}")

    # find local maxima (peaks) in h: h[i-1] < h[i] > h[i+1]
    peaks = []
    for i in range(1, len(h) - 1):
        if h[i] > h[i - 1] and h[i] > h[i + 1] and h[i] > FLOOR + 1e-3:
            peaks.append((t[i], h[i]))

    print(f"found {len(peaks)} bounce peaks")
    for pt, ph in peaks[:6]:
        print(f"  t={pt:6.3f}  peak height above floor = {ph - FLOOR:.4f}")

    if len(peaks) < 3:
        raise SystemExit("FAIL: expected at least 3 bounce peaks to check decay ratio")

    expected_ratio = E ** 2
    worst_dev = 0.0
    for i in range(1, min(len(peaks), 5)):
        h_prev = peaks[i - 1][1] - FLOOR
        h_next = peaks[i][1] - FLOOR
        ratio = h_next / h_prev
        dev = abs(ratio - expected_ratio)
        worst_dev = max(worst_dev, dev)
        print(f"  peak {i}: height ratio = {ratio:.4f}  (expected e^2 = {expected_ratio:.4f}, "
              f"dev {dev:.4f})")

    if worst_dev > RATIO_TOLERANCE:
        raise SystemExit(f"FAIL: peak-height ratio deviates from e^2 by {worst_dev:.4f} "
                          f"(tolerance {RATIO_TOLERANCE})")

    print(f"\nOK -- floor respected, bounce peak heights decay by ~e^2={expected_ratio:.4f} "
          f"as expected (worst deviation {worst_dev:.4f}).")


if __name__ == "__main__":
    main()
