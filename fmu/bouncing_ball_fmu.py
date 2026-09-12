"""
BouncingBall FMU -- free_fall_fmu.py plus a floor and a coefficient of
restitution. This is essentially the classic FMI reference "BouncingBall"
model: same semi-implicit Euler integration, but when h would go below the
floor it's clamped there and the velocity is reflected and scaled by `e`.

    v(t+dt) = v(t) + g*dt
    h(t+dt) = h(t) + v(t+dt)*dt
    if h(t+dt) < floor:
        h(t+dt) = floor
        v(t+dt) = -e * v(t+dt)

Exposes:
    input  g      [m/s^2]  gravitational acceleration (default -9.81)
    input  e      [-]      coefficient of restitution, 0..1 (default 0.7)
    input  floor  [m]      floor height (default 0.0)
    output h      [m]      height
    output v      [m/s]    vertical velocity

Each bounce should lose a factor e^2 of the kinetic energy at impact, so
consecutive peak heights above the floor should shrink by that same ratio
-- see validate_bouncing_ball.py, which checks exactly that instead of
comparing to a single closed-form curve (there isn't one once bouncing
starts).

Build:
    pythonfmu build -f bouncing_ball_fmu.py -d build
"""
from pythonfmu import Fmi2Slave, Fmi2Causality, Fmi2Variability, Real


class BouncingBall(Fmi2Slave):
    author = "chrono_test"
    description = "Ball falling under gravity, bouncing off a floor with restitution e"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.g = -9.81
        self.e = 0.7
        self.floor = 0.0
        self.h = 10.0
        self.v = 0.0

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

    def do_step(self, current_time: float, step_size: float) -> bool:
        self.v += self.g * step_size
        self.h += self.v * step_size
        if self.h < self.floor:
            self.h = self.floor
            self.v = -self.e * self.v
        return True
