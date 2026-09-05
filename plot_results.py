"""Plot vehicle_log.csv produced by simple_vehicle.py."""
import csv
import os

import matplotlib.pyplot as plt


def load_log(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        rows = list(reader)

    susp_cols = [c for c in fieldnames if c.startswith("susp_")]
    cols = {name: [] for name in fieldnames}
    for row in rows:
        for k in fieldnames:
            cols[k].append(float(row[k]))
    return cols, susp_cols


def main():
    log_path = os.path.join(os.path.dirname(__file__), "vehicle_log.csv")
    d, susp_cols = load_log(log_path)

    fig, axes = plt.subplots(2, 3, figsize=(15, 7))

    ax = axes[0, 0]
    ax.plot(d["chassis_x"], d["chassis_y"])
    ax.set_title("Top-down trajectory (X-Y)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True)

    ax = axes[0, 1]
    ax.plot(d["time"], d["speed_mps"])
    ax.set_title("Forward speed")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed [m/s]")
    ax.grid(True)

    ax = axes[0, 2]
    ax.plot(d["time"], d["steer_deg"], label="steer cmd")
    ax.plot(d["time"], d["yaw_deg"], label="yaw")
    ax.set_title("Steer angle / chassis yaw")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("deg")
    ax.legend()
    ax.grid(True)

    ax = axes[1, 0]
    ax.plot(d["time"], d["chassis_z"])
    ax.set_title("Chassis height (Z)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("z [m]")
    ax.grid(True)

    ax = axes[1, 1]
    ax.plot(d["time"], d["roll_deg"], label="roll")
    ax.plot(d["time"], d["pitch_deg"], label="pitch")
    ax.set_title("Chassis roll / pitch")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("deg")
    ax.legend()
    ax.grid(True)

    ax = axes[1, 2]
    for k in susp_cols:
        ax.plot(d["time"], d[k], label=k.replace("susp_", ""))
    ax.set_title("Suspension spring length")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("length [m]")
    ax.legend(ncol=2, fontsize=8)
    ax.grid(True)

    fig.tight_layout()
    out_path = os.path.join(os.path.dirname(__file__), "vehicle_results.png")
    fig.savefig(out_path, dpi=130)
    print(f"Saved plot to {out_path}")


if __name__ == "__main__":
    main()
