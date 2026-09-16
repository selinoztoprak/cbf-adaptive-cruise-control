# **Adaptive Cruise Control with CBF-QP Safety Filter**
**_Author: Sophie Selin Oztoprak_**

This repository implements an Adaptive Cruise Control (ACC) system that uses a Control Barrier Function (CBF) Safety Filter. This is a personal project that explores and implements modern control laws, with a focus on state-space modelling, Control Barrier Functions (CBFs) and the limits of quadratic programming under actuator saturation.

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





## Safety and Control Barrier Functions (CBFs)

The nominal controller attempts to track a desired cruising speed, $v_{cruise}$ using a feedback-feedforward law:

$$
u_{nom} = -k_v(v - v_{cruise}) + F_r(v_{cruise})
$$

A time-headway separation is added for safety. The safe operating set $\mathcal{C}$ is defined by the zero-superlevel set of the barrier function h(x):

$$
h(x) = D - \tau v - D_{min} \geq 0
$$

where $\tau$ is the time headway and D_{min} is the minimum allowable bumper-to-bumper distance between the ego and lead vehicle. 

###Lie Derivatives 
The Lie derivatives for drift dynamics $(L_fh(x))$ and the control vector field $(L_gh(x))$are found by taking the time derivatives of the barrier function along the system's trajectories. 

$$
L_fh(x) = v_L - v + \frac{\tau F_r(v)}{m}
$$

$$
L_gh(x) = \frac{-\tau}{m}
$$

##Actuator Saturation 

Real vehicles have strict actuator limits due to engine acceleration and tire friction braking:

$$
u_{min} \leq u \leq u_{max}
$$

When a slower vehicle cuts in front of the egov ehicle, the relative distance D drops abruptly, such that h(x) becomes highly negative. The zeroing CBF conditions stipulates that the vehicle cannot approach the safety boundary too fast, demanding an impossible amountof instantaneous braking force to satisfy the constraint. 

As a result, the et of admissible inputs that satisfy botht he safety barrier and actuator bounds becomes empty. Standard optimsiation sovlers would crash with an infeasibility error. 

## The Relaxed CBF-QP Formulation 

To maintain point-wise feasibility for the Quadratic Program during distrubances, a slack variable $\delta $ is introduced. 

The inline optimisation filter is solved at each time step:

$$\begin{aligned} u^*(x), \delta^*(x) = \arg\min_{u, \delta} & \quad \frac{1}{2} (u - u_{\text{nom}})^2 + p \delta^2 \\ \text{s.t.} & \quad L_f h(x) + L_g h(x) u \ge -\gamma h(x) - \delta \\ & \quad u_{\min} \le u \le u_{\max} \\ & \quad \delta \ge 0 \end{aligned}$$

Under normal operation, a large penalty $p$ (initially set to $10^6$) forces the solver to set $\delta^* = 0$, preserving the original CBF condition. When a cut-in occurs and safety cannot be maintained, the QP increases $\delta > 0$ just enough to maintain mathematical feasibility. This clamps the actuator to the maximum possible physical deceleration ($u^* = u_{min}$), restoring forward invariance. 

## Simulation Results 

The benchamark script run_simulation.py coapres two closed-loop scenarios:

1. **Unshielded Cruise Control:** The nominal controller is blind to the distance of the lead car and fails to react to the sudden cut-in, eventually crossing the $D = 0$ threshold and crashing. 

2. **CBF-QP Shielded Controller:** The safety filter overrides the nominal throttle, heavily relaxing the slack variable to remain feasible while commanding the absolute brake limit to safely arrest the vehicle and rebuild a safe following distance. 

## Quickstart Guide 

This project requires standard sceintific Python packages: numpy, scipy, cvxpy and matplotlib. 

To run the simulation and generate comparison plots, execute:

`cd scripts` 

`python run_comparison.py`