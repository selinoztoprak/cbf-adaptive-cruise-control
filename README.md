# **Adaptive Cruise Control with CLF-CBF-QP Safety Filter**
**_Author: Sophie Selin Oztoprak_**

This repository implements an Adaptive Cruise Control (ACC) system that uses a Control Barrier Function (CBF) Safety Filter. This is a personal project that explores and implements modern control laws, with a focus on state-space modelling and Quadratic Programs (QPs) formed with Control Lyapunov and Barrier Functions (CLFs and CBFs).

## Mathematical Formulation 

The ego vehicle is modeled as a continuous time, control-affine nonlinear system, $\dot{x} = f(x) + g(x)u$. 

Two states are tracked: the relative bumper distance to the lead vehicle, D, and the ego vehicle's longitudinal velocity, v:

$$
\begin{bmatrix} D \\ v \end{bmatrix}
$$


The equations of motion are governed by Newton's second law, incorporating an empirical aerodynamic drag formula, $F_r(v)$:

$$
\dot{D} = v_L - v \\
$$

$$
m\dot{v} = u - F_r(v) \\
$$

$$
F_r(v) = f_0 + f_1v + f_2v^2
$$

where $v_L$ is the lead vehicle velocity, $u$ is the control wheel force and $m$ is the vehicle mass. 

The full state-space model is therefore as such:

```math
\begin{bmatrix} \dot{D} \\ \dot{v} \end{bmatrix}
=
\begin{bmatrix} v_L - v \\ -\frac{F_r(v)}{m} \end{bmatrix}
+
\begin{bmatrix} 0 \\ \frac{1}{m} \end{bmatrix}u
```





## Control Lyapunov Functions and Control Barrier Functions (CLF-CBFs)

A nominal controller is used as comparison to the CLF-CBF controller. The nominal controller only tracks the desired cruising speed, $v_{cruise}$ using a feedback-feedforward law:

$$
u_{nom} = -k_v(v - v_{cruise}) + F_r(v_{cruise})
$$

### Control Lyapunov Function (CLF)

To regulate the vehicle to $v_{cruise}$, a quadratic Control Lyapunov Function (CLF) is defined:

$$
V(x) = (v - v_{cruise})^2
$$

Its Lie derivatives along the system dynamics are:

$$L_f V(x) = -\frac{2(v - v_{\text{cruise}}) F_r(v)}{m}, \quad L_g V(x) = \frac{2(v - v_{\text{cruise}})}{m}$$

The exponential convergence condition requires:

$$L_f V(x) + L_g V(x)u \le -c V(x)$$

where $c > 0$ denotes the convergence rate.

### Control Barrier Function (CBF)


The CBF ensures safety via distance invariance. The safe operating set $\mathcal{C}$ is defined by the zero-superlevel set of the barrier function h(x):

$$
h(x) = D - \tau v - D_{min} \geq 0
$$

where $\tau$ is the time headway and $D_{min}$ is the minimum allowable bumper-to-bumper distance between the ego and lead vehicle. 


The Lie derivatives along the system trajectories are:

$$
L_fh(x) = v_L - v + \frac{\tau F_r(v)}{m}
$$

$$
L_gh(x) = \frac{-\tau}{m}
$$

Forward invariance of $\mathcal{C}$ is ensured by enforcing:

$$L_f h(x) + L_g h(x) u \ge -\gamma h(x)$$

## Actuator Saturation 

Real vehicles have strict actuator limits due to engine acceleration and tire friction braking:

$$
u_{min} \leq u \leq u_{max}
$$

When a slower vehicle cuts in front of the egov ehicle, the relative distance D drops abruptly, such that h(x) becomes highly negative. The zeroing CBF conditions stipulates that the vehicle cannot approach the safety boundary too fast, demanding an impossible amountof instantaneous braking force to satisfy the constraint. 

As a result, the et of admissible inputs that satisfy botht he safety barrier and actuator bounds becomes empty. Standard optimsiation sovlers would crash with an infeasibility error. 

## The CLF-CBF-QP Formulation

To guarantee safety as an uncompromising requirement, the CBF condition is treated as a hard constraint. To resolve conflicts between speed regulation and collision avoidance during critical scenarios (such as abrupt cut-ins), a slack variable $\delta \ge 0$ is introduced to relax the CLF tracking condition.

The quadratic program is solved at each time step:

$$u^*, \delta^* = \arg\min_{u, \delta} \frac{1}{2}(u - u_{\text{nom}})^2 + p \delta^2$$

$$\begin{aligned}
\text{s.t.} \quad & L_f h(x) + L_g h(x)u \ge -\gamma h(x) && \text{(Hard Safety Constraint)} \\
& L_f V(x) + L_g V(x)u \le -c V(x) + \delta && \text{(Soft Tracking Constraint)} \\
& u_{\min} \le u \le u_{\max} && \text{(Actuator Limits)} \\
& \delta \ge 0
\end{aligned}$$

Under nominal conditions, a large slack penalty $p$ drives $\delta^* \to 0$, enabling asymptotic speed tracking while maintaining headway. When an abrupt disturbance occurs and safe deceleration conflicts with the target cruising speed, the QP relaxes the CLF constraint by increasing $\delta > 0$, prioritizing the safety barrier condition and decelerating the vehicle safely.

## Simulation Results 

The benchamark script run_simulation.py coapres two closed-loop scenarios:

1. **Unshielded Cruise Control:** The nominal controller is blind to the distance of the lead car and fails to react to the sudden cut-in, eventually crossing the $D = 0$ threshold and crashing. 

2. **CBF-QP Shielded Controller:** The safety filter overrides the nominal throttle, heavily relaxing the slack variable to remain feasible while commanding the absolute brake limit to safely arrest the vehicle and rebuild a safe following distance. 

## Quickstart Guide

This project requires standard scientific Python packages: `numpy`, `scipy`, `cvxpy`, and `matplotlib`.

To set up your environment, run the simulation, and generate the comparison plots, execute the following from the project root:

```bash
# 1. Create a virtual environment
python -m venv .venv

# 2. Activate the virtual environment
# For Windows PowerShell:
venv\Scripts\Activate.ps1
# (For Mac/Linux use: source venv/bin/activate)

# 3. Install dependencies
pip install -r requirements.txt

# 4. Run the benchmark simulation
cd scripts
python run_comparison.py
