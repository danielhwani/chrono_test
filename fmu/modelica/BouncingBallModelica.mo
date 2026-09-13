// Same physical model/parameters as bouncing_ball_fmu.py and
// bouncing_ball_native.c (g=-9.81, e=0.7, floor=0.0, h0=10.0, v0=0.0),
// but written as an actual continuous-time hybrid Modelica model instead
// of a hand-rolled explicit-Euler integrator. OpenModelica's default
// solver handles the bounce as a zero-crossing event (reinit), so its
// bounce timing/height is the analytic solution, not a discretization
// approximation -- the point of trying this FMU is to drive a real
// tool-generated FMU (not one of our own hand-written ones) through
// fmu_driver.c and see whether the C driver, written generically against
// the FMI2 spec, works against it unmodified.
model BouncingBallModelica
  parameter Real g = -9.81 "gravity";
  parameter Real e = 0.7 "restitution coefficient";
  parameter Real floor = 0.0 "floor height";
  Real h(start = 10.0, fixed = true) "height";
  Real v(start = 0.0, fixed = true) "velocity";
equation
  der(h) = v;
  der(v) = g;
  when h <= floor then
    reinit(v, -e * pre(v));
    reinit(h, floor);
  end when;
end BouncingBallModelica;
