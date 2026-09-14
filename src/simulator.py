import numpy as np 
from scipy.integrate import solve_ivp

class ACCSimulator:

    def __init__(self, dynamics, dt: float = 0.01):
        self.dynamics = dynamics 
        self.dt = dt

    def step(
            self, x : np.ndarray, u : float, v_lead: float, t_start: float
    ) -> np.ndarray:

        def ode_func(t, state):
            return self.dynamics.state_derivatives(t, state, u, v_lead)

        sol = solve_ivp(ode_func, 
                        t_span = [t_start, t_start + self.dt],
                        y0=x,
                        method='RK45',
                        t_eval=[t_start + self.dt],
                        )

        return sol.y[:, -1]