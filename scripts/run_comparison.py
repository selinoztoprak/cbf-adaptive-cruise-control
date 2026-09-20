import sys
from pathlib import Path

src_path = Path(__file__).resolve().parent.parent / "src"
sys.path.append(str(src_path))

import matplotlib.pyplot as plt
import numpy as np
from clf_cbf_filter import CBFFilter
from dynamics import VehicleDynamics
from nominal_controller import NominalController
from simulator import ACCSimulator


def run_simulation(use_cbf: bool, cut_in: bool, t_end: float = 12.0, dt: float = 0.01):
    # Benchmark vehicle and scenario parameters
    v_des = 30.0  # m/s
    v_lead_init = 15.0  # m/s
    t_cut = 3.0  # cut-in occurrence time (s)
    clf_rate = 1.0

    # Initial state: [Distance D = 50 m, Ego velocity v = 30 m/s]
    x0 = np.array([50.0, 30.0])

    dynamics = VehicleDynamics()
    nominal_ctrl = NominalController(dynamics=dynamics, v_cruise=v_des)
    sim = ACCSimulator(dynamics=dynamics, dt=dt)

    cbf = CBFFilter(
        dynamics=dynamics,
        nominal_controller=nominal_ctrl,
        D_min=2.0,
        tau=1.4,
        gamma=1.0,
        p=1e6,
        u_min_ratio=-0.4,
        u_max_ratio=0.25,
        v_des = v_des,
        clf_rate = clf_rate,
    )

    t_steps = int(t_end / dt)
    time = np.linspace(0, t_end, t_steps + 1)

    states = np.zeros((t_steps + 1, 2))
    u_nom_hist = np.zeros(t_steps)
    u_actual_hist = np.zeros(t_steps)
    h_hist = np.zeros(t_steps + 1)
    slacks = np.zeros(t_steps)

    states[0] = x0
    h_hist[0] = cbf.h(x0)

    for k in range(t_steps):
        t = time[k]
        x_curr = states[k].copy()

        # Dynamic disturbance: abrupt highway cut-in collapses distance gap
        if cut_in and abs(t - t_cut) < (dt / 2.0):
            x_curr[0] = 8.0  # Headway instantly collapses down to 8 m

        v_lead = v_lead_init

        # Nominal controller tracks speed: u_nom = nominal_ctrl.u_nom(v)
        u_nom = nominal_ctrl.u_nom(x_curr[1])
        u_nom_hist[k] = u_nom

        if use_cbf:
            # Uses your exact filter_control method signature
            u_act, delta = cbf.filter_control(x=x_curr, v_lead=v_lead)
            slacks[k] = delta
        else:
            # Unshielded cruise control clamped only by actuator limits
            u_act = float(np.clip(u_nom, cbf.u_min, cbf.u_max))
            slacks[k] = 0.0

        u_actual_hist[k] = u_act

        # Step dynamics forward: x_{k+1}
        # If your dynamics step expects step(x, u, v_lead, dt)
        x_next = sim.step(x=x_curr, u=u_act, v_lead=v_lead, t_start=t)
        states[k + 1] = x_next
        h_hist[k + 1] = cbf.h(x_next)

    return {
        "time": time,
        "states": states,
        "u_nom": u_nom_hist,
        "u_actual": u_actual_hist,
        "h": h_hist,
        "slacks": slacks,
    }


def main():
    print("Simulating Scenario 1: Unshielded Nominal Controller (Cut-In)...")
    res_nominal = run_simulation(use_cbf=False, cut_in=True)

    print("Simulating Scenario 2: CBF-QP Shielded Controller (Cut-In)...")
    res_cbf = run_simulation(use_cbf=True, cut_in=True)

    t = res_cbf["time"]
    t_ctrl = t[:-1]

    fig, axs = plt.subplots(5, 1, figsize=(9, 11), sharex=True)

    # 1. Bumper-to-Bumper Distance
    axs[0].plot(
        t, res_nominal["states"][:, 0], "r--", label="Nominal Controller (Collision)"
    )
    axs[0].plot(t, res_cbf["states"][:, 0], "b-", label="CBF-QP Filtered")
    axs[0].axhline(y=0.0, color="k", linestyle=":", label="Physical Crash ($D=0$)")
    axs[0].set_ylabel("Distance $D$ [m]")
    axs[0].set_title("Cut-In Scenario: Nominal vs. CBF Filtered ACC")
    axs[0].grid(True)
    axs[0].legend(loc="upper right")

    # 2. Ego Speed
    axs[1].plot(t, res_nominal["states"][:, 1], "r--", label="Nominal Speed")
    axs[1].plot(t, res_cbf["states"][:, 1], "b-", label="CBF-Filtered Speed")
    axs[1].axhline(y=15.0, color="g", linestyle="--", label="Lead Vehicle (15 m/s)")
    axs[1].set_ylabel("Speed $v$ [m/s]")
    axs[1].grid(True)
    axs[1].legend(loc="upper right")

    # 3. Control Force Input
    axs[2].plot(t_ctrl, res_nominal["u_actual"], "r--", label="Nominal Input")
    axs[2].plot(t_ctrl, res_cbf["u_actual"], "b-", label="CBF-Filtered Input")
    axs[2].set_ylabel("Force $u$ [N]")
    axs[2].grid(True)
    axs[2].legend(loc="lower right")

    # 4. Barrier Value h(x)
    axs[3].plot(t, res_nominal["h"], "r--", label="Nominal $h(x)$")
    axs[3].plot(t, res_cbf["h"], "b-", label="CBF $h(x)$")
    axs[3].axhline(y=0.0, color="k", linestyle=":", label="Safety Boundary ($h=0$)")
    axs[3].set_xlabel("Time [s]")
    axs[3].set_ylabel("Barrier $h(x)$")
    axs[3].grid(True)
    axs[3].legend(loc="lower right")

    
    # 4. Slack term
    axs[4].plot(t_ctrl, res_cbf["slacks"], "b-", label="CLF $\\delta$")
    axs[4].set_xlabel("Time [s]")
    axs[4].set_ylabel("Slack $\\delta$")
    axs[4].grid(True)
    axs[4].legend(loc="lower right")

    plt.tight_layout()

    assets_path = Path(__file__).resolve().parent.parent / "assets"
    assets_path.mkdir(exist_ok=True)
    fig_path = assets_path / "acc_cutin_comparison.png"
    plt.savefig(fig_path, dpi=300)
    print(f"Comparison plot saved successfully to: {fig_path}")
    plt.show()


if __name__ == "__main__":
    main()