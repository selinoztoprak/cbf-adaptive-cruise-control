from dynamics import VehicleDynamics

class NominalController:

    def __init__(
            self,
            dynamics: VehicleDynamics,
            v_cruise : float = 30.0,
            k_v : float = 100.0,
    ):
        self.dynamics = dynamics
        self.v_cruise = v_cruise
        self.k_v = k_v

    def u_nom(self, v: float) -> float:
        f_feedforward = self.dynamics.drag_force(self.v_cruise)
        f_feedback = -self.k_v * (v - self.v_cruise)
        return float(f_feedback + f_feedforward)

