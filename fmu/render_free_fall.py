"""
Real-time Irrlicht rendering of the FreeFall FMU -- same idea as the
vehicle demos (simple_vehicle.py --irrlicht), but the "dynamics" come
entirely from stepping the .fmu co-simulation slave via fmpy's low-level
FMI2 API instead of a PyChrono ChSystem. The falling ball is purely
kinematic: its position is set directly from the FMU's `h` output each
frame, nothing is computed by Chrono itself.

Build the FMU first if you haven't:
    pythonfmu build -f free_fall_fmu.py -d build

Run:
    python render_free_fall.py
"""
import argparse
import os

import pychrono as chrono
import pychrono.irrlicht as chronoirr
from fmpy import read_model_description, extract
from fmpy.fmi2 import FMU2Slave

FMU_PATH = os.path.join(os.path.dirname(__file__), "build", "FreeFall.fmu")
SIM_TIME = 1.8   # by t=1.8s, h has fallen from 10 to about -6 -- stays in the camera frame
STEP_SIZE = 2e-3
BALL_RADIUS = 0.3


def make_fmu_instance():
    if not os.path.exists(FMU_PATH):
        raise SystemExit(f"{FMU_PATH} not found -- run: pythonfmu build -f free_fall_fmu.py -d build")

    model_description = read_model_description(FMU_PATH)
    value_refs = {v.name: v.valueReference for v in model_description.modelVariables}
    unzip_dir = extract(FMU_PATH)

    fmu = FMU2Slave(
        guid=model_description.guid,
        unzipDirectory=unzip_dir,
        modelIdentifier=model_description.coSimulation.modelIdentifier,
        instanceName="free_fall_instance",
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()
    return fmu, value_refs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--time", type=float, default=SIM_TIME)
    args = parser.parse_args()

    fmu, vr = make_fmu_instance()

    sys_ = chrono.ChSystemNSC()
    sys_.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # reference plane at h=0 -- the ball is allowed to fall through it
    # (this FMU has no floor/bounce logic by design, see free_fall_fmu.py)
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
    vis.SetWindowTitle("FreeFall.fmu -- real-time render")
    vis.Initialize()
    vis.ShowExplorer(False)
    vis.AddSkyBox()
    # fixed camera framing roughly h in [-6, 12] -- the ball only moves
    # vertically, so (unlike the vehicle demos) a chase camera would just
    # dive below the ground plane and look disorienting once h goes negative
    vis.AddCamera(chrono.ChVector3d(10, -10, 4), chrono.ChVector3d(0, 0, 2))
    vis.AddTypicalLights()

    print("Rendering FreeFall.fmu in real time. Close the window or Ctrl+C to stop.")

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
        ball.SetPos(chrono.ChVector3d(0, 0, h))

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
