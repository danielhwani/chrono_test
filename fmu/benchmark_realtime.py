"""
Two different questions, both answered here:

1. --pacing off (default): how much computational headroom is there?
   Steps the FMU as fast as possible (no ChRealtimeStepTimer.Spin), with
   and without rendering, and reports the real-time factor (RTF =
   simulated_time / wall_time). This is what render_*.py's pacing would
   otherwise hide, since Spin() just sleeps off any surplus speed.

2. --pacing on: does the *paced* loop (the one render_*.py actually
   runs, Spin() included) actually track real time? Reports the wall-clock
   time actually taken for a requested simulated duration -- it should
   land within a few ms of the target if pacing is working correctly.
   A paced run finishing noticeably *short* of the target means Spin()
   isn't sleeping enough (a bug); finishing *long* means each step's own
   work already exceeds its real-time budget and Spin() can't make up the
   difference (the pacing has nothing left to hide -- this is the same
   "can't keep up" case --pacing off's RTF<1 would also catch).

Both modes have --render off/on/both to separate the dynamics-only cost
from the rendering cost.

Run:
    python benchmark_realtime.py                        # RTF, unpaced
    python benchmark_realtime.py --pacing on             # sync accuracy, paced
    python benchmark_realtime.py --pacing on --render on
    python benchmark_realtime.py --fmu build/FreeFall.fmu --sim-time 5

--render both runs the headless and with-rendering measurements as two
separate subprocesses (not sequentially in this one process) -- creating
a second FMU2Slave/Irrlicht session after a first one has already run
segfaulted ("corrupted double-linked list") during cleanup in testing.
Isolating them in fresh processes also avoids either measurement being
skewed by state (or GPU/driver context) left over from the other.
"""
import argparse
import os
import subprocess
import sys
import time

from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave

DEFAULT_FMU = os.path.join(os.path.dirname(__file__), "build", "BouncingBall.fmu")


def make_fmu_instance(fmu_path):
    model_description = read_model_description(fmu_path)
    value_refs = {v.name: v.valueReference for v in model_description.modelVariables}
    unzip_dir = extract(fmu_path)

    fmu = FMU2Slave(
        guid=model_description.guid,
        unzipDirectory=unzip_dir,
        modelIdentifier=model_description.coSimulation.modelIdentifier,
        instanceName="benchmark_instance",
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()
    return fmu, value_refs


def run_headless(fmu_path, sim_time, step_size, pacing=False, trace_interval=0):
    fmu, vr = make_fmu_instance(fmu_path)
    has_h = "h" in vr

    realtime_timer = None
    if pacing:
        import pychrono as chrono
        realtime_timer = chrono.ChRealtimeStepTimer()

    n_steps = int(sim_time / step_size)
    step_wall_times = []
    trace = []  # (sim_time, wall_time_elapsed_so_far)

    wall_start = time.perf_counter()
    t = 0.0
    for i in range(n_steps):
        step_t0 = time.perf_counter()
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=step_size)
        step_wall_times.append(time.perf_counter() - step_t0)
        t += step_size
        if has_h:
            fmu.getReal([vr["h"]])  # touch an output, like a real caller would
        if realtime_timer is not None:
            realtime_timer.Spin(step_size)
        if trace_interval and i % trace_interval == 0:
            trace.append((t, time.perf_counter() - wall_start))
    wall_elapsed = time.perf_counter() - wall_start
    if trace_interval:
        trace.append((t, wall_elapsed))

    fmu.terminate()
    fmu.freeInstance()
    return wall_elapsed, step_wall_times, trace


def run_with_rendering(fmu_path, sim_time, step_size, pacing=False, trace_interval=0):
    import pychrono as chrono
    import pychrono.irrlicht as chronoirr

    fmu, vr = make_fmu_instance(fmu_path)
    has_h = "h" in vr
    realtime_timer = chrono.ChRealtimeStepTimer() if pacing else None

    sys_ = chrono.ChSystemNSC()
    sys_.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    ground = chrono.ChBodyEasyBox(20, 20, 0.05, 1000, True, False)
    ground.SetFixed(True)
    sys_.Add(ground)
    ball = chrono.ChBodyEasySphere(0.3, 500, True, False)
    ball.SetFixed(True)
    sys_.Add(ball)

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.AttachSystem(sys_)
    vis.SetWindowSize(900, 700)
    vis.SetWindowTitle(f"Realtime benchmark (rendering ON, {'paced' if pacing else 'unpaced'})")
    vis.Initialize()
    vis.ShowExplorer(False)
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(10, -10, 4), chrono.ChVector3d(0, 0, 3))
    vis.AddTypicalLights()

    RENDER_DT = 1.0 / 60.0
    next_render_t = 0.0
    n_steps = int(sim_time / step_size)
    step_wall_times = []
    trace = []  # (sim_time, wall_time_elapsed_so_far)

    wall_start = time.perf_counter()
    t = 0.0
    for i in range(n_steps):
        if not vis.Run():
            break
        step_t0 = time.perf_counter()
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=step_size)
        step_wall_times.append(time.perf_counter() - step_t0)
        t += step_size
        if has_h:
            h = fmu.getReal([vr["h"]])[0]
            ball.SetPos(chrono.ChVector3d(0, 0, h + 0.3))

        if t >= next_render_t:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            next_render_t += RENDER_DT
        if realtime_timer is not None:
            realtime_timer.Spin(step_size)
        if trace_interval and i % trace_interval == 0:
            trace.append((t, time.perf_counter() - wall_start))
    wall_elapsed = time.perf_counter() - wall_start
    if trace_interval:
        trace.append((t, wall_elapsed))

    fmu.terminate()
    fmu.freeInstance()
    return wall_elapsed, step_wall_times, trace


def write_trace(path, trace):
    import csv
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sim_time", "wall_time"])
        writer.writerows(trace)
    print(f"  trace written to {path} ({len(trace)} samples)")


def report(label, sim_time, wall_elapsed, step_wall_times, pacing=False):
    rtf = sim_time / wall_elapsed if wall_elapsed > 0 else float("inf")
    mean_step = sum(step_wall_times) / len(step_wall_times)
    max_step = max(step_wall_times)
    print(f"\n--- {label} ---")
    print(f"  simulated time : {sim_time:.3f} s")
    print(f"  wall time      : {wall_elapsed:.3f} s")
    if pacing:
        drift = wall_elapsed - sim_time
        pct = 100 * drift / sim_time
        verdict = "in sync" if abs(pct) < 1 else ("running SLOW -- Spin() can't keep up" if drift > 0 else "running FAST -- Spin() under-sleeping")
        print(f"  drift (wall - sim): {drift:+.4f} s ({pct:+.2f}%)  -- {verdict}")
    else:
        print(f"  real-time factor (sim/wall): {rtf:.2f}x  "
              f"{'(faster than real time)' if rtf >= 1 else '(CANNOT keep up with real time)'}")
    print(f"  mean do_step wall time: {mean_step * 1e6:.1f} us")
    print(f"  max  do_step wall time: {max_step * 1e6:.1f} us")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fmu", default=DEFAULT_FMU)
    parser.add_argument("--sim-time", type=float, default=5.0)
    parser.add_argument("--step-size", type=float, default=2e-3)
    parser.add_argument("--render", choices=["off", "on", "both"], default="both")
    parser.add_argument("--pacing", choices=["off", "on"], default="off",
                         help="off (default): run flat-out, report the real-time factor. "
                              "on: pace with ChRealtimeStepTimer.Spin() like render_*.py "
                              "actually does, and report how closely wall time tracks sim time.")
    parser.add_argument("--trace-out", default=None,
                         help="write a (sim_time,wall_time) CSV trace to this path, for plotting "
                              "(see plot_realtime_benchmark.py)")
    parser.add_argument("--trace-interval", type=int, default=10,
                         help="record a trace sample every N steps (default 10)")
    parser.add_argument("--_mode", choices=["headless", "render"], help=argparse.SUPPRESS)
    args = parser.parse_args()
    pacing = args.pacing == "on"

    if not os.path.exists(args.fmu):
        raise SystemExit(f"{args.fmu} not found -- build it first with pythonfmu build")

    trace_interval = args.trace_interval if args.trace_out else 0

    # single-mode: actually run the measurement (this is what the subprocess dispatch below invokes)
    if args._mode == "headless":
        wall, steps, trace = run_headless(args.fmu, args.sim_time, args.step_size,
                                           pacing=pacing, trace_interval=trace_interval)
        report("headless (dynamics only, no Chrono/Irrlicht)", args.sim_time, wall, steps, pacing)
        if args.trace_out:
            write_trace(args.trace_out, trace)
        return
    if args._mode == "render":
        wall, steps, trace = run_with_rendering(args.fmu, args.sim_time, args.step_size,
                                                 pacing=pacing, trace_interval=trace_interval)
        report(f"with rendering ({'paced' if pacing else 'unpaced'})", args.sim_time, wall, steps, pacing)
        if args.trace_out:
            write_trace(args.trace_out, trace)
        return

    # top-level: dispatch each requested mode as its own subprocess
    common = ["--fmu", args.fmu, "--sim-time", str(args.sim_time),
              "--step-size", str(args.step_size), "--pacing", args.pacing,
              "--trace-interval", str(args.trace_interval)]
    if args.render in ("off", "both"):
        extra = ["--trace-out", args.trace_out + ".headless.csv"] if args.trace_out else []
        subprocess.run([sys.executable, __file__, "--_mode", "headless"] + common + extra, check=True)
    if args.render in ("on", "both"):
        extra = ["--trace-out", args.trace_out + ".render.csv"] if args.trace_out else []
        subprocess.run([sys.executable, __file__, "--_mode", "render"] + common + extra, check=True)


if __name__ == "__main__":
    main()
