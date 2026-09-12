"""
Measure whether stepping the FMU can actually keep up with real time --
without the artificial pacing (ChRealtimeStepTimer.Spin) the render_*.py
scripts use, which would otherwise just sleep off any surplus speed and
hide the true computational cost.

Two modes:
  --render off (default): pure fmpy stepping loop, no Chrono/Irrlicht at
      all. This is the number that matters for "can the dynamics alone
      run in real time" (e.g. for a future ros_control-style loop with no
      visualization).
  --render on: same stepping loop, but also drives a Chrono/Irrlicht scene
      (BeginScene/Render/EndScene) every frame, still with no pacing, so
      the reported real-time factor shows the cost of visualization on
      top of the dynamics -- exactly what you'd want to know to decide
      whether to exclude rendering from a real-time budget.

Real-time factor (RTF) = simulated_time / wall_time. RTF >= 1 means it
ran at or faster than real time (there's slack); RTF < 1 means it can't
keep up.

Run:
    python benchmark_realtime.py
    python benchmark_realtime.py --render on
    python benchmark_realtime.py --fmu build/FreeFall.fmu --sim-time 5
"""
import argparse
import os
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


def run_headless(fmu_path, sim_time, step_size):
    fmu, vr = make_fmu_instance(fmu_path)
    has_h = "h" in vr

    n_steps = int(sim_time / step_size)
    step_wall_times = []

    wall_start = time.perf_counter()
    t = 0.0
    for _ in range(n_steps):
        step_t0 = time.perf_counter()
        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=step_size)
        step_wall_times.append(time.perf_counter() - step_t0)
        t += step_size
        if has_h:
            fmu.getReal([vr["h"]])  # touch an output, like a real caller would
    wall_elapsed = time.perf_counter() - wall_start

    fmu.terminate()
    fmu.freeInstance()
    return wall_elapsed, step_wall_times


def run_with_rendering(fmu_path, sim_time, step_size):
    import pychrono as chrono
    import pychrono.irrlicht as chronoirr

    fmu, vr = make_fmu_instance(fmu_path)
    has_h = "h" in vr

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
    vis.SetWindowTitle("Realtime benchmark (rendering ON, unpaced)")
    vis.Initialize()
    vis.ShowExplorer(False)
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(10, -10, 4), chrono.ChVector3d(0, 0, 3))
    vis.AddTypicalLights()

    RENDER_DT = 1.0 / 60.0
    next_render_t = 0.0
    n_steps = int(sim_time / step_size)
    step_wall_times = []

    wall_start = time.perf_counter()
    t = 0.0
    for _ in range(n_steps):
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
        # deliberately NOT calling realtime_timer.Spin() -- unpaced, to measure true cost
    wall_elapsed = time.perf_counter() - wall_start

    fmu.terminate()
    fmu.freeInstance()
    return wall_elapsed, step_wall_times


def report(label, sim_time, wall_elapsed, step_wall_times):
    rtf = sim_time / wall_elapsed if wall_elapsed > 0 else float("inf")
    mean_step = sum(step_wall_times) / len(step_wall_times)
    max_step = max(step_wall_times)
    print(f"\n--- {label} ---")
    print(f"  simulated time : {sim_time:.3f} s")
    print(f"  wall time      : {wall_elapsed:.3f} s")
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
    args = parser.parse_args()

    if not os.path.exists(args.fmu):
        raise SystemExit(f"{args.fmu} not found -- build it first with pythonfmu build")

    if args.render in ("off", "both"):
        wall, steps = run_headless(args.fmu, args.sim_time, args.step_size)
        report("headless (dynamics only, no Chrono/Irrlicht)", args.sim_time, wall, steps)

    if args.render in ("on", "both"):
        wall, steps = run_with_rendering(args.fmu, args.sim_time, args.step_size)
        report("with rendering (unpaced)", args.sim_time, wall, steps)


if __name__ == "__main__":
    main()
