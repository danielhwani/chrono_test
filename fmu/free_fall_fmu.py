"""
Toy FMU to validate the pythonfmu toolchain (build + load + simulate) before
wrapping the actual vehicle dynamics. Deliberately the simplest possible
dynamic system: a point mass in free fall, no floor, no bounce.

    v(t+dt) = v(t) + g*dt
    h(t+dt) = h(t) + v(t+dt)*dt   (semi-implicit Euler)

Exposes:
    input  g  [m/s^2]  gravitational acceleration (constant, default -9.81)
    output h  [m]      height
    output v  [m/s]     vertical velocity

Analytical reference (constant g, h(0)=h0, v(0)=0): h(t) = h0 + 0.5*g*t^2,
v(t) = g*t -- used by validate_fmu.py to check the built FMU numerically,
not just that a .fmu file got produced.

Build:
    pythonfmu build -f free_fall_fmu.py -d build

The instance name is fixed by the class name (FreeFall) unless overridden
by the importing tool.
"""
from pythonfmu import Fmi2Slave, Fmi2Causality, Fmi2Variability, Real


class FreeFall(Fmi2Slave):
    author = "chrono_test"
    description = "Point mass in free fall (pythonfmu toolchain smoke test)"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.g = -9.81
        self.h = 10.0
        self.v = 0.0

        self.register_variable(
            Real("g", causality=Fmi2Causality.input, variability=Fmi2Variability.continuous)
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
        return True
