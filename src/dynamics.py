import numpy as np 

class VehicleDynamics:

    def __init__(
            self,
            m: float = 1650.0,
            f0: float = 0.1,
            f1: float = 5.0,
            f2: float = 0.25,
            g: float = 9.81,
    ):
        self.m = m
        self.f0 = f0
        self.f1 = f1
        self.f2 = f2
        self.gravity = g

    def drag_force(self, v: float) -> float:
        return self.f0 + self.f1 * v + self.f2 * (v**2)

    def f(self, x: np.ndarray, v_lead: float) -> np.ndarray:
        D, v = x[0], x[1]
        return np.array([v_lead - v, -self.drag_force(v) / self.m])

    def g(self) -> np.ndarray:
        return np.array([0.0, 1.0/ self.m])

    def state_derivatives(
            self, t: float, x: np.ndarray, u: float, v_lead: float
    ) -> np.ndarray:
        return self.f(x, v_lead) + self.g() * u

