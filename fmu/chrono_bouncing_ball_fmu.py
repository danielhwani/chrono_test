"""
Same bouncing-ball FMU interface as bouncing_ball_fmu.py (g/e/floor inputs,
h/v outputs, same defaults) -- but this time do_step() doesn't hand-integrate
the physics itself. It builds a real pychrono.ChSystemNSC with a collidable
sphere and a fixed ground plane (Bullet collision, an NSC contact material
carrying the restitution coefficient `e`), and each do_step() just calls
sys.DoStepDynamics(step_size) and reads the ball's actual simulated position/
velocity back out. This is a genuine "Chrono slave" -- the point of building
it is to test whether a master other than our own drivers (specifically,
OpenModelica/OMEdit) can import and co-simulate it, i.e. whether Chrono
physics can sit on the *slave* side of someone else's FMI master for once,
instead of Chrono always being the master (as in simple_vehicle.py) or not
involved at all (as in every other FMU in this repo).

h is defined as the sphere's bottom surface height (center z - radius) so it
lines up with the other bouncing-ball models' floor-contact convention, not
the sphere center.

Build:
    pythonfmu build -f chrono_bouncing_ball_fmu.py -d build
"""
from pythonfmu import Fmi2Slave, Fmi2Causality, Fmi2Variability, Real

RADIUS = 0.05


class ChronoBouncingBall(Fmi2Slave):
    author = "chrono_test"
    description = "Ball falling under gravity, bouncing off a floor -- actually simulated by pychrono (Bullet collision + NSC contact restitution), not hand-integrated"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.g = -9.81
        self.e = 0.7
        self.floor = 0.0
        self.h = 10.0
        self.v = 0.0
        self._sys = None
        self._ball = None

        self.register_variable(
            Real("g", causality=Fmi2Causality.input, variability=Fmi2Variability.continuous)
        )
        self.register_variable(
            Real("e", causality=Fmi2Causality.input, variability=Fmi2Variability.continuous)
        )
        self.register_variable(
            Real("floor", causality=Fmi2Causality.input, variability=Fmi2Variability.continuous)
        )
        self.register_variable(
            Real("h", causality=Fmi2Causality.output, variability=Fmi2Variability.continuous)
        )
        self.register_variable(
            Real("v", causality=Fmi2Causality.output, variability=Fmi2Variability.continuous)
        )

    def exit_initialization_mode(self):
        # Build the actual Chrono system here, not in __init__, so it picks
        # up whatever g/e/floor the master set as start values/inputs during
        # initialization instead of always using our own defaults.
        import pychrono as chrono

        self._sys = chrono.ChSystemNSC()
        self._sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, self.g))
        self._sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

        mat = chrono.ChContactMaterialNSC()
        mat.SetRestitution(self.e)
        mat.SetFriction(0.0)

        self._ball = chrono.ChBodyEasySphere(RADIUS, 1000, True, True, mat)
        self._ball.SetPos(chrono.ChVector3d(0, 0, self.h + RADIUS))
        self._sys.Add(self._ball)

        ground = chrono.ChBodyEasyBox(50, 50, 0.1, 1000, True, True, mat)
        ground.SetPos(chrono.ChVector3d(0, 0, self.floor - 0.05))
        ground.SetFixed(True)
        self._sys.Add(ground)

    def do_step(self, current_time: float, step_size: float) -> bool:
        self._sys.DoStepDynamics(step_size)
        self.h = self._ball.GetPos().z - RADIUS
        self.v = self._ball.GetLinVel().z
        return True
