"""
Real-time Irrlicht rendering of BouncingBall.fmu -- same approach as
render_free_fall.py (fmpy steps the FMU directly, Chrono only draws the
result), but now the ball actually bounces instead of falling through the
floor. Because it stays bounded between the floor and its drop height, a
single fixed camera frames the whole thing (no chase-cam dive issue like
the plain free-fall version had).

Build the FMU first if you haven't:
    pythonfmu build -f bouncing_ball_fmu.py -d build

Run:
    python render_bouncing_ball.py
    python render_bouncing_ball.py --restitution 0.9 --time 12
"""
import argparse
import os

import pychrono as chrono
import pychrono.irrlicht as chronoirr
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave

FMU_PATH = os.path.join(os.path.dirname(__file__), "build", "BouncingBall.fmu")
SIM_TIME = 8.0
STEP_SIZE = 2e-3
BALL_RADIUS = 0.3


def make_fmu_instance(restitution):
    if not os.path.exists(FMU_PATH):
        raise SystemExit(f"{FMU_PATH} not found -- run: pythonfmu build -f bouncing_ball_fmu.py -d build")

    model_description = read_model_description(FMU_PATH)
    value_refs = {v.name: v.valueReference for v in model_description.modelVariables}
    unzip_dir = extract(FMU_PATH)

    fmu = FMU2Slave(
        guid=model_description.guid,
        unzipDirectory=unzip_dir,
        modelIdentifier=model_description.coSimulation.modelIdentifier,
        instanceName="bouncing_ball_instance",
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.setReal([value_refs["e"]], [restitution])
    fmu.exitInitializationMode()
    return fmu, value_refs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--time", type=float, default=SIM_TIME)
    parser.add_argument("--restitution", type=float, default=0.7,
                         help="coefficient of restitution e (0..1); set via the FMU's own "
                              "input variable, not by editing the model")
    args = parser.parse_args()

    fmu, vr = make_fmu_instance(args.restitution)

    sys_ = chrono.ChSystemNSC()
    sys_.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    ground = chrono.ChBodyEasyBox(20, 20, 0.05, 1000, True, False)
    ground.SetPos(chrono.ChVector3d(0, 0, 0))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.55, 0.5))
    sys_.Add(ground)

    ball = chrono.ChBodyEasySphere(BALL_RADIUS, 500, True, False)
    ball.SetFixed(True)  # position is driven directly from the FMU, not by Chrono dynamics
    ball.GetVisualShape(0).SetColor(chrono.ChColor(0.9, 0.2, 0.2))
    sys_.Add(ball)

    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.AttachSystem(sys_)
    vis.SetWindowSize(900, 700)
    vis.SetWindowTitle("BouncingBall.fmu -- real-time render")
    vis.Initialize()
    vis.ShowExplorer(False)
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(10, -10, 4), chrono.ChVector3d(0, 0, 3))
    vis.AddTypicalLights()

    print(f"Rendering BouncingBall.fmu (e={args.restitution}) in real time. "
          f"Close the window or Ctrl+C to stop.")

    RENDER_DT = 1.0 / 60.0
    next_render_t = 0.0
    realtime_timer = chrono.ChRealtimeStepTimer()

    t = 0.0
    while t < args.time:
        if not vis.Run():
            break

        fmu.doStep(currentCommunicationPoint=t, communicationStepSize=STEP_SIZE)
        t += STEP_SIZE
        h = fmu.getReal([vr["h"]])[0]
        ball.SetPos(chrono.ChVector3d(0, 0, h + BALL_RADIUS))

        if t >= next_render_t:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            next_render_t += RENDER_DT

        realtime_timer.Spin(STEP_SIZE)

    fmu.terminate()
    fmu.freeInstance()
    print("Done.")


if __name__ == "__main__":
    main()
