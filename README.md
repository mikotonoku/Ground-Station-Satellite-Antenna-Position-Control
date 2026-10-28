# DC Servo Antenna Position Control 🛰

![MATLAB](https://img.shields.io/badge/MATLAB-R2026b-0076A8?logo=Mathworks&logoColor=white)
![Simulink](https://img.shields.io/badge/Simulink-Model--Based%20Design-FF6F00?logo=Mathworks&logoColor=white)
![Control System Toolbox](https://img.shields.io/badge/Control%20System-Toolbox-00629B)
![Fuzzy Logic Toolbox](https://img.shields.io/badge/Fuzzy%20Logic-Toolbox-6A1B9A)
![PID Control](https://img.shields.io/badge/Control-PID-2E8B57)
![LQR](https://img.shields.io/badge/Control-LQR-8A2BE2)
![Fuzzy PI](https://img.shields.io/badge/Control-Fuzzy--PI-FF8C00)
![Academic Project](https://img.shields.io/badge/Type-Academic%20Project-success)
[![Research Basis](https://img.shields.io/badge/Research%20Basis-IEEE%20Access%202023-00629B)](https://doi.org/10.1109/ACCESS.2023.3313976)

## About the Project

This repository contains my **RMS course project** on modelling and controlling the angular position of a ground-station satellite antenna driven by a DC servo motor. The work is based on the control-system concept presented in the IEEE Access article *Enhanced Antenna Positioning Control System Using Adapted DC Servo Motor and Fuzzy-PI Controller*.

The aim is to command the antenna to a required angle and make the motor position follow that reference accurately, quickly, and with acceptable overshoot and oscillation. The MATLAB script defines the motor parameters, derives both a transfer-function and a state-space model, and prepares the controllers. The Simulink model implements the closed feedback loops and visualizes the reference and the actual motor position.

Three control strategies are studied under the same plant model: a model-tuned PID controller, an LQR full-state-feedback controller, and a Mamdani Fuzzy-PI controller with adaptive proportional and integral gains. The repository is designed both as a reproducible simulation study and as a beginner-friendly example of how mathematical modelling, controller design, and Simulink validation fit together in an antenna-positioning task.

> Slovak version: [README_SK.md](README_SK.md)

---

<a id="contents"></a>

## Table of Contents
- [Project goal](#project-goal)
- [Scientific basis](#scientific-basis)
- [Learning map: technologies and concepts](#learning-map-technologies-and-concepts)
- [Mathematical model](#mathematical-model)
- [Controllers](#controllers)
- [Repository implementation](#repository-implementation)
- [Parameters and tuning logic](#parameters-and-tuning-logic)
- [Results and interpretation](#results-and-interpretation)
- [Getting started](#getting-started)
- [Repository structure](#repository-structure)
- [Limitations](#limitations)
- [List of figures](#list-of-figures)
- [List of tables](#list-of-tables)
- [List of equations](#list-of-equations)
- [References](#references)
- [Author](#author)

---
## Lists of figures, tables and equations

<a id="list-of-figures"></a>

### List of figures

1. [Figure 1. PID control loop in Simulink](#fig-pid)
2. [Figure 2. LQR state-feedback control loop in Simulink](#fig-lqr)
3. [Figure 3. Mamdani Fuzzy-PI control loop in Simulink](#fig-fuzzy)
4. [Figure 4. Comparison of simulated antenna-position responses](#fig-results)

<a id="list-of-tables"></a>

### List of tables

1. [Table 1. Learning map: technologies and concepts](#tab-learning-map)
2. [Table 2. Motor data used](#tab-motor-data)
3. [Table 3. Parameters and tuning logic](#tab-tuning)
4. [Table 4. Results and interpretation](#tab-results)

<a id="list-of-equations"></a>

### List of equations

1. [Equation 1. State vector](#eq-state-vector)
2. [Equation 2. Parallel PID law](#eq-pid-law)
3. [Equation 3. Filtered PID transfer function](#eq-filtered-pid)
4. [Equation 4. LQR performance index](#eq-lqr-index)
5. [Equation 5. Fuzzy-PI gain scaling](#eq-fuzzy-gains)
6. [Equation 6. DC motor electrical and mechanical model](#eq-motor-dynamics)
7. [Equation 7. Voltage-to-position transfer function](#eq-transfer-function)
8. [Equation 8. State-space representation](#eq-state-space)

[Back to contents](#contents)

## Project goal

This repository is my implementation of a DC servo motor antenna-positioning task. The model accepts a desired antenna angle and simulates the motor position. I used it to explore three approaches to feedback control:

1. **PID** - an automatically tuned, filtered parallel PID controller;
2. **LQR** - full-state feedback designed from a continuous-time state-space model;
3. **Fuzzy-PI** - a Mamdani inference system that adapts proportional and integral gains from the position error and its change.

The project is intended as a first practical introduction to control engineering: derive a plant model, create an executable simulation, tune controllers, and compare reference tracking rather than treating controller gains as arbitrary constants.

[Back to contents](#contents)

## Scientific basis

The model is based on the article by Fkirin and Khira, *Enhanced Antenna Positioning Control System Using Adapted DC Servo Motor and Fuzzy-PI Controller* (IEEE Access, 2023) [1]. The paper studies a ground-data-receiving-station antenna driven by a T-36001 DC servo motor and compares PID, LQR, and Fuzzy-PI approaches.

This repository implements the article's modelling idea and controller families for an educational MATLAB/Simulink experiment. It is **not** a claim of hardware validation or an exact reproduction of every controller setting and result in the paper. In particular, the LQR weighting `R = 10`, PID crossover frequency, and Fuzzy-PI scaling values are the settings used in this project and are documented below.

The included paper is distributed under its stated Creative Commons Attribution-NonCommercial-NoDerivatives license. Please cite the original authors when reusing its ideas or the unmodified paper.

[Back to contents](#contents)

## Learning map: technologies and concepts

<a id="tab-learning-map"></a>

**Table 1. Learning map: technologies and concepts.**

| Topic | What it means | Where to see it in this repository |
|---|---|---|
| MATLAB | Numerical environment used to define parameters, transfer functions, state-space models, and controllers. | [`Zadanie_1.m`](Zadanie_1.m) |
| Simulink | Block-diagram environment used to assemble the feedback loops and run time-domain simulations. [2] | [`Zadanie_1_sim.slx`](Zadanie_1_sim.slx) |
| Transfer function | Input-output description in the Laplace domain: `G(s) = Theta(s)/Va(s)`. | `num`, `den`, and `G = tf(num,den)` |
| State-space model | First-order vector model `xdot = Ax + Bu`, `y = Cx + Du`; needed for LQR. | `A`, `B`, `C`, `D` in `Zadanie_1.m` |
| PID control | Feedback law built from proportional, integral, and derivative actions. [3][4] | PID branch of the model |
| LQR | State-feedback method minimizing a quadratic trade-off between state error and control effort. [5] | LQR branch and `lqr(A,B,Q,R)` |
| Fuzzy inference | Rule-based controller using linguistic membership functions and Mamdani inference. [6] | [`Fuzzy_PI.fis`](Fuzzy_PI.fis) |
| Validation by simulation | Compare reference tracking, overshoot, oscillation, and settling behaviour before hardware testing. | Simulink Scope blocks |

### MATLAB and Simulink

MATLAB provides the numerical model and controller-design commands. Simulink turns the equations into connected blocks, so signals and feedback paths can be inspected visually. This is a Model-Based Design workflow: the same model is used to formulate, simulate, and examine the system before any physical implementation. [2]

### Transfer functions and state variables

A transfer function is useful when the main question is: *how does output position change when armature voltage changes?* A state-space model is useful when internal variables such as armature current and angular velocity also matter. Here the state vector is

<a id="eq-state-vector"></a>

**Equation 1. State vector.**

$$x = \begin{bmatrix} i_a & \omega & \theta \end{bmatrix}^{T},$$

where $i_a$ is armature current, $\omega$ is angular velocity, and $\theta$ is angular position in radians.

### PID control in plain language

For error $e(t)=r(t)-\theta(t)$, a parallel continuous-time PID controller is

<a id="eq-pid-law"></a>

**Equation 2. Parallel PID law.**

$$u(t)=K_p e(t)+K_i\int_0^t e(\tau)\,d\tau+K_d\frac{de(t)}{dt}.$$

- $K_p$ responds to the current error and usually makes the response faster.
- $K_i$ accumulates past error and can remove steady-state offset, but too much integral action can cause overshoot or a slow oscillatory tail.
- $K_d$ reacts to the error trend and can add damping; practical implementations filter the derivative because differentiation amplifies high-frequency noise.

The MATLAB `pid` representation used by `pidtune` includes a first-order derivative filter [4]:

<a id="eq-filtered-pid"></a>

**Equation 3. Filtered PID transfer function.**

$$C(s)=K_p+\frac{K_i}{s}+\frac{K_d s}{T_f s+1}.$$

### LQR in plain language

LQR chooses the state-feedback input $u=-Kx$ to minimize

<a id="eq-lqr-index"></a>

**Equation 4. LQR performance index.**

$$J=\int_0^\infty \left(x^TQx+u^TRu\right)dt.$$

`Q` expresses how strongly each state deviation is penalized; `R` expresses how expensive large control input is. MATLAB computes the gain `K` from the associated Riccati equation. [5] LQR assumes that the states used for feedback are available or estimated.

### Fuzzy-PI in plain language

Fuzzy logic replaces one fixed formula for the gains with rules such as “if error is positive high and changing quickly, use a high/low gain.” The controller here has two inputs - `Error` and `ChangeError` - and two outputs - normalized `Kp` and `Ki`. It uses seven Gaussian membership functions for each input and 49 Mamdani rules. The actual gains are recovered by output scaling:

<a id="eq-fuzzy-gains"></a>

**Equation 5. Fuzzy-PI gain scaling.**

$$K_p(t)=G_{Kp}K_{p,\mathrm{fuzzy}}(t),\qquad
K_i(t)=G_{Ki}K_{i,\mathrm{fuzzy}}(t).$$

[Back to contents](#contents)

## Mathematical model

The armature circuit and mechanical dynamics are represented by

<a id="eq-motor-dynamics"></a>

**Equation 6. DC motor electrical and mechanical model.**

$$V_a(t)=R_a i_a(t)+L_a\frac{di_a(t)}{dt}+K_b\omega(t),$$

$$J_m\frac{d\omega(t)}{dt}+B_m\omega(t)=K_t i_a(t),\qquad \dot{\theta}(t)=\omega(t).$$

With zero initial conditions, the voltage-to-position transfer function is [1]

<a id="eq-transfer-function"></a>

**Equation 7. Voltage-to-position transfer function.**

$$G(s)=\frac{\Theta(s)}{V_a(s)}=
\frac{K_t}{s\left[(R_a+L_a s)(J_m s+B_m)+K_tK_b\right]}.$$

The equivalent state-space representation used for the LQR experiment is

<a id="eq-state-space"></a>

**Equation 8. State-space representation.**

$$\dot{x}=Ax+Bu,\qquad y=Cx+Du,$$

$$A=\begin{bmatrix}
-R_a/L_a & -K_b/L_a & 0\\
K_t/J_m & -B_m/J_m & 0\\
0 & 1 & 0
\end{bmatrix},\quad
B=\begin{bmatrix}1/L_a\\0\\0\end{bmatrix},\quad
C=\begin{bmatrix}0&0&180/\pi\end{bmatrix}.$$

`C` converts the simulated position from radians to degrees for the state-space output. The project keeps the physical controller model in radians: `Deg` is the human-readable target in degrees, while `Ref = Deg*pi/180` is the corresponding simulation reference in radians.

### Motor data used

<a id="tab-motor-data"></a>

**Table 2. Motor data used.**

| Symbol | Meaning | Value used in code |
|---|---|---:|
| $R_a$ | Armature resistance | 13.3 Ohm |
| $L_a$ | Armature inductance | 29 H |
| $J_m$ | Rotor inertia | 51.386 |
| $J_a$ | Load inertia (declared for the study) | 3.5946 |
| $B_m$ | Viscous friction coefficient | 13.558 |
| $K_t$ | Torque constant | 138 |
| $K_b$ | Back-EMF constant | 187 |
| `Deg` | Default reference angle | 150 deg |

The numerical values above are taken from the project source and follow the motor data used as the paper's basis. Consult the cited paper for the original context and units. [1]

[Back to contents](#contents)

## Controllers

### PID branch

<a id="fig-pid"></a>

**Figure 1. PID control loop in Simulink.** The reference position is compared with the measured motor position; the PID block converts the error into the motor control signal.

<p align="center">
  <img width="793" height="147" alt="PID control scheme" src="https://github.com/user-attachments/assets/23735932-f219-45c6-88d0-50e975695eab" />
</p>
The script creates the plant with `tf` and obtains the controller programmatically:

```matlab
wc = 0.5;                     % target crossover frequency [rad/s]
C_pid = pidtune(G, 'PID', wc);

Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;
```

This is not trial-and-error: `pidtune` designs a PID controller for the linear model and aims to balance performance and robustness. [3][7] The gains are intentionally not hard-coded; they are recomputed from the current plant whenever the script runs.

### LQR branch

<a id="fig-lqr"></a>

**Figure 2. LQR state-feedback control loop in Simulink.** The state vector is multiplied by the calculated gain `K_lqr`; the resulting control action drives the state-space motor model.

<p align="center">
  <img width="1360" height="217" alt="LQR control scheme" src="https://github.com/user-attachments/assets/e7e5f1f7-9897-424f-a535-fc09efdf6fab" />
</p>
The project uses full-state feedback and calculates `K_lqr` with MATLAB's `lqr` function.

```matlab
Q = 1e6 * eye(3);
R = 10;
K_lqr = lqr(A, B, Q, R);
```

The matrix `Q` weights current, speed, and angle equally in this experiment. The scalar `R=10` is the control-effort weight selected for this repository's response study. Changing either matrix changes the trade-off and therefore the gain and response.

### Fuzzy-PI branch

<a id="fig-fuzzy"></a>

**Figure 3. Mamdani Fuzzy-PI control loop in Simulink.** Error and change of error enter the FIS, whose normalized outputs are scaled to time-varying proportional and integral gains.

<p align="center">
  <img width="1546" height="217" alt="Fuzzy-PI control scheme" src="https://github.com/user-attachments/assets/ea786de8-078d-4e58-b375-f1f0c505e5b9" />
</p>
`Fuzzy_PI.fis` defines a Mamdani FIS with `min` implication, `max` aggregation, and centroid defuzzification. The script loads it through `readfis` and sets the following scales:

```matlab
Ge  = 1/Ref;
Gde = 0.8;
GKp = 130;
GKi = 20;
```

The two input variables are normalized error and error change. The FIS output is mapped back to time-varying PI gains by `GKp` and `GKi`.

[Back to contents](#contents)

## Repository implementation

The central workflow is:

```text
Motor parameters --> G(s), A/B/C/D --> controller design --> Simulink model
                                                               |
Reference angle (Deg --> Ref in rad) --------------------------+
                                                               |
                                                        Scope / plots
```

1. `Zadanie_1.m` clears the workspace, declares the motor constants, and converts the desired angle from degrees to radians.
2. It creates both `G` and the `A,B,C,D` state-space model.
3. It computes the PID gains through `pidtune`, the LQR gain through `lqr`, and loads the Fuzzy-PI FIS if available.
4. `Zadanie_1_sim.slx` uses these workspace variables in its controller blocks.
5. The scopes and exported plots show reference and motor position.

The simulation results are viewed directly in the Simulink Scope blocks.

[Back to contents](#contents)

## Parameters and tuning logic

<a id="tab-tuning"></a>

**Table 3. Parameters and tuning logic.**

| Controller | Parameters in this implementation | Selection logic | Reproducibility note |
|---|---|---|---|
| PID | `type='PID'`, `wc=0.5 rad/s` | Automatic model-based tuning with `pidtune(G,'PID',wc)`. | Run `Zadanie_1.m`; inspect the printed `Kp`, `Ki`, `Kd`. |
| LQR | `Q=1e6*eye(3)`, `R=10` | `Q` penalizes state deviation; `R` penalizes control effort. `lqr` solves the continuous-time LQR problem. | The script recomputes `K_lqr`. |
| Fuzzy-PI | `Ge=1/Ref`, `Gde=0.8`, `GKp=130`, `GKi=20`; 49 rules | Normalize the input domain, infer normalized gains, then scale them. | The `.fis` file must be in the MATLAB current folder. |

When comparing controllers, use the same reference and simulation configuration. Look at:

- **Rise time**: how quickly the response first approaches the target;
- **Overshoot**: how far the response goes beyond the target;
- **Settling**: how quickly the response remains near the target;
- **Control effort**: whether the demanded voltage/control signal would be practical for a real actuator.

[Back to contents](#contents)

## Results and interpretation

For the default simulation, `Deg = 150`, i.e. $150\pi/180 \approx 2.618$ rad. Run the Simulink model and compare the traces in the Scope blocks using the same reference angle.
<a id="fig-results"></a>

**Figure 4. Comparison of simulated antenna-position responses.** The yellow trace is the reference angle; the controller responses show how quickly and smoothly the antenna position follows it.

<p align="center">
  <img width="2238" height="1170" alt="Simulated antenna-position responses" src="https://github.com/user-attachments/assets/2fff9ea7-b040-4d7c-9ff3-ae9a578ca18f" />
</p>

<a id="tab-results"></a>

**Table 4. Results and interpretation.**

| Controller | What to evaluate in your simulation | Interpretation |
|---|---|---|
| PID | Rise time, overshoot, and oscillation. | A faster loop is not automatically a better loop; transient constraints matter. |
| LQR | Reference tracking using the configured `Q` and `R` values. | The quadratic weights determine the trade-off between state error and control effort. |
| Fuzzy-PI | Tracking using the FIS rule base and configured scaling factors. | Adaptive gains change the transient behaviour; performance depends on rule base and scaling. |

The final comparison should be made from the traces produced by your own run of the model. PID, LQR, and Fuzzy-PI do not have a universal ranking: the outcome depends on the plant, controller settings, reference, and performance criterion. The source paper reports its own experimental comparison under its own settings. [1]

[Back to contents](#contents)

## Getting started

### Requirements

- MATLAB with **Control System Toolbox** (`tf`, `ss`, `pidtune`, `lqr`);
- Simulink;
- **Fuzzy Logic Toolbox** to load and simulate `Fuzzy_PI.fis` / the fuzzy controller branch.

### Run the study

1. Clone or download this repository.
2. In MATLAB, set the Current Folder to the repository root (the folder containing `Zadanie_1.m`).
3. Run:

   ```matlab
   Zadanie_1
   ```

4. Open `Zadanie_1_sim.slx`.
5. Run the simulation and inspect the scopes.
6. To test a different target angle, edit only:

   ```matlab
   Deg = 150;
   ```

   The script automatically sets `Ref = Deg*pi/180` so that the model receives radians.

### Important note about PID Tuner

The Simulink **Tune...** button can require a Simulink Control Design license. This repository does not depend on that interactive app: it uses the `pidtune` command on the LTI plant. If `pidtune` is unavailable in a local installation, the PID branch cannot be regenerated automatically; use an installation that includes Control System Toolbox.

[Back to contents](#contents)

## Repository structure

```text
Project root/
├── README.md              # English documentation
├── README_SK.md           # Slovak overview
├── Zadanie_1.m            # Parameters, plant, controller design
├── Zadanie_1_sim.slx      # Simulink model
└── Fuzzy_PI.fis           # Mamdani Fuzzy-PI inference system
```
## Limitations

- This is a simulation study, not a real-time hardware controller.
- The plant is linear and does not include actuator saturation, sensor noise, gear backlash, parameter uncertainty, or load disturbances unless added by the user.
- LQR full-state feedback assumes the required states are measured or estimated.
- The controller comparison is configuration-dependent; change only one variable at a time when making conclusions.

## References

1. M. A. Fkirin and M. A.-E. Khira, “Enhanced Antenna Positioning Control System Using Adapted DC Servo Motor and Fuzzy-PI Controller,” *IEEE Access*, vol. 11, pp. 102661-102668, 2023. [DOI: 10.1109/ACCESS.2023.3313976](https://doi.org/10.1109/ACCESS.2023.3313976).
2. MathWorks, “[Simulink Documentation](https://www.mathworks.com/help/simulink/).”
3. MathWorks, “[PID Controller Tuning](https://www.mathworks.com/help/control/pid-controller-design.html).”
4. MathWorks, “[`pid` - PID controller in parallel form](https://www.mathworks.com/help/control/ref/pid.html).”
5. MathWorks, “[`lqr` - Linear-Quadratic Regulator design](https://www.mathworks.com/help/control/ref/lti.lqr.html).”
6. MathWorks, “[Fuzzy Logic Toolbox Documentation](https://www.mathworks.com/help/fuzzy/).”
7. MathWorks, “[`pidtune` - PID tuning algorithm for linear plant model](https://www.mathworks.com/help/control/ref/dynamicsystem.pidtune.html).”

## Author

**Ievgeniia Ievgrafova**

RMS course project 

Faculty of Electrical Engineering and Information Technology

Slovak University of Technology in Bratislava

2026
