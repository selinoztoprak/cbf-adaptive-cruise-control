from dynamics import VehicleDynamics
import numpy as np
import cvxpy as cp
from nominal_controller import NominalController

class CBFFilter():

    def __init__(
            self,
            dynamics : VehicleDynamics,
            nominal_controller : NominalController,
            D_min : float = 2.0,
            tau: float = 1.4,
            gamma: float = 1.0,
            p: float = 1e6, 
            u_min_ratio: float = -0.4, 
            u_max_ratio: float = 0.25,
    ):
        self.dynamics = dynamics
        self.nominal_controller = nominal_controller
        self.D_min = D_min
        self.tau = tau
        self.gamma = gamma
        self.p = p
        self.u_min = u_min_ratio * self.dynamics.m * self.dynamics.gravity
        self.u_max = u_max_ratio * self.dynamics.m * self.dynamics.gravity
    

    def h(self, x: np.ndarray) -> float:
        return x[0] - self.tau*x[1] - self.D_min

    def lie_derivatives(self, v_lead: float, x: np.ndarray) -> tuple[float, float]:
        lf_h = v_lead - x[1] + (self.tau * self.dynamics.drag_force(x[1])) / self.dynamics.m
        lf_g = -self.tau / self.dynamics.m
        return float(lf_h), float(lf_g)

    def filter_control(
            self, x: np.ndarray, v_lead : float) -> tuple[float, float]:
        h_val = self.h(x)
        lf_h, lg_h = self.lie_derivatives(v_lead, x)
        u_nom = self.nominal_controller.u_nom(x[1])

        u = cp.Variable()
        delta = cp.Variable() # Slack variable for feasability 

        objective = cp.Minimize(0.5 * cp.square(u - u_nom) + self.p * cp.square(delta))

        constraints = [
            lf_h + lg_h * u >= -self.gamma * h_val - delta,
            u >= self.u_min,
            u <= self.u_max,
            delta >= 0.0,
        ]

        problem = cp.Problem(objective, constraints)
        problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)


        if problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            return float(self.u_min), 0.0

        return float(u.value), float(delta.value)


    

