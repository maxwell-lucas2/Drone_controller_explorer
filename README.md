# Quadrotor Control Lab — 6-DOF Flight Dynamics Simulator

An interactive browser-based simulation of a quadrotor UAV implementing four distinct control architectures over full 6-DOF rigid body dynamics. Built for controller comparison, gain tuning experimentation, and control theory education.

**[Live Demo →](https://yourusername.github.io/quadrotor-control-lab/)**

---

## Features

- **Full 6-DOF dynamics** — Rigid body equations of motion with Euler angle kinematics, aerodynamic drag, and RK4 integration at 120 Hz
- **Four control architectures** — Cascaded PID, Sliding Mode Control, Super-Twisting SMC, and Model Predictive Control
- **Real-time gain tuning** — Adjust every controller parameter live and observe immediate effects
- **Seven trajectory modes** — Hover, step response, circle, helix, figure-8, waypoint square, custom waypoints, and keyboard piloting
- **Live telemetry** — Position, velocity, attitude, error norm, thrust, and motor speeds
- **Five synchronized charts** — Desired vs actual (XYZ), error/sliding surface, control effort, phase portrait, motor speeds
- **Data export** — Full state recording to CSV for MATLAB/Python post-processing
- **Wind disturbance** — Stochastic wind model to test robustness
- **3D visualization** — Realistic X-frame drone with spinning propellers, actual path trail, desired trajectory preview, and MPC prediction horizon
- **Camera orbit** — Click-drag to orbit, scroll to zoom

---

## Control Architectures

### 1. Cascaded PID

Classical linear controller with cascaded position (outer) and attitude (inner) loops.

**Outer loop (position → desired attitude):**

$$a_{des} = K_p \cdot e + K_i \int e \, dt + K_d \cdot \dot{e}$$

$$\phi_{des} = \arcsin\left(\frac{m(a_{x,des} \sin\psi - a_{z,des} \cos\psi)}{T}\right)$$

**Inner loop (attitude → torques):**

$$\tau = K_{p,att}(\phi_{des} - \phi) - K_{d,att} \cdot p$$

**Characteristics:** Simple tuning, well-understood stability margins, no formal robustness guarantees.

### 2. Sliding Mode Control (Standard)

Defines sliding surfaces in the error state space and uses discontinuous control to enforce invariance.

**Sliding surface:** $s = \dot{e} + \lambda e$

**Control law:** $u = u_{eq} + \eta \cdot \text{sat}(s, \phi)$

Where $\phi > 0$ introduces a boundary layer to reduce chattering, and $\eta$ must exceed the bound of matched disturbances.

**Characteristics:** Finite-time reaching, robust to matched uncertainties, chattering from discontinuous switching.

### 3. Super-Twisting SMC (2nd Order)

Continuous sliding mode algorithm that eliminates chattering while maintaining finite-time convergence.

**Control law:**

$$u_1 = -\alpha_1 |s|^{1/2} \text{sign}(s) + v$$

$$\dot{v} = -\alpha_2 \text{sign}(s)$$

**Sufficient condition:** $\alpha_1^2 \geq 4\alpha_2$ (for convergence in the presence of Lipschitz disturbances)

**Characteristics:** Chattering-free, continuous control signal, integral term compensates constant disturbances.

### 4. Model Predictive Control

Solves a finite-horizon optimization at each timestep using a linearized double-integrator model.

**Cost function:**

$$J = \sum_{k=1}^{N} Q \|x_k - x_{ref,k}\|^2 + R \|u_k\|^2$$

The yellow prediction line in the 3D view shows the anticipated trajectory over the horizon.

**Characteristics:** Handles constraints, anticipates trajectory changes, computational cost scales with horizon N.

---

## Dynamics Model

The simulation implements standard quadrotor rigid body dynamics:

**Translational (world frame):**

$$m\ddot{p} = R \cdot T\hat{e}_3 - mg\hat{e}_3 - C_d \dot{p} + f_{wind}$$

**Rotational (body frame — Euler equations):**

$$I\dot{\omega} = \tau - \omega \times (I\omega)$$

**Euler angle kinematics (ZYX convention):**

$$\dot{\phi} = p + (q \sin\phi + r\cos\phi)\tan\theta$$

**Motor mixing (X-configuration):**

| Signal   | Formula |
|----------|---------|
| Thrust   | $T = k_T(\omega_1^2 + \omega_2^2 + \omega_3^2 + \omega_4^2)$ |
| Roll τ   | $\tau_\phi = k_T L(-\omega_1^2 - \omega_2^2 + \omega_3^2 + \omega_4^2)/\sqrt{2}$ |
| Pitch τ  | $\tau_\theta = k_T L(-\omega_1^2 + \omega_2^2 + \omega_3^2 - \omega_4^2)/\sqrt{2}$ |
| Yaw τ    | $\tau_\psi = k_D(-\omega_1^2 + \omega_2^2 - \omega_3^2 + \omega_4^2)$ |

Integration via 4th-order Runge-Kutta at 120 Hz.

---

## Project Structure

```
quadrotor-control-lab/
├── index.html              # Main page — layout, telemetry HUD, sidebar UI
├── css/
│   └── style.css           # Aerospace mission control theme
├── js/
│   ├── physics.js          # 6-DOF rigid body dynamics, RK4 integrator, motor allocation
│   ├── controllers.js      # PID, SMC, STS-SMC, MPC implementations
│   ├── trajectories.js     # Predefined paths, custom waypoints, keyboard input
│   ├── drone3d.js          # Three.js scene, drone model, path visualization
│   ├── charts.js           # Chart.js telemetry graphs
│   ├── keyboard.js         # Keyboard input manager
│   ├── dataLogger.js       # Recording & CSV export
│   └── app.js              # Main loop, UI binding, orchestration
└── README.md
```

---

## Usage

### Quick Start

1. Clone or download the repository
2. Serve with any static file server (or open `index.html` directly)
3. Select a controller and trajectory, then observe the behavior

### Keyboard Controls (Manual Mode)

| Key | Action |
|-----|--------|
| W/S or ↑/↓ | Forward / Backward (Z axis) |
| A/D or ←/→ | Left / Right (X axis) |
| Space | Ascend |
| Shift | Descend |
| Q / E | Yaw left / right |

### Data Export

1. Click **⏺ Record** to begin capturing state data
2. Run your experiment
3. Click **⏹ Stop** then **↓ Export CSV (Full)** to download
4. CSV includes: timestamp, full 12-state vector, target reference, control inputs, sliding surfaces, and motor speeds

---

## Experiments to Try

1. **Chattering comparison:** Set trajectory to Hover. Switch between SMC (φ=0) and STS. Observe the control effort chart — SMC shows high-frequency switching; STS is smooth.

2. **Robustness test:** Set wind to 5+ m/s. Compare PID (watch integral windup) vs SMC (maintains invariance on the sliding surface).

3. **Tracking fidelity:** Use Figure-8 at high speed. MPC with long horizon anticipates curvature changes; PID exhibits phase lag.

4. **Step response:** Use the Step trajectory. Measure rise time, overshoot, and settling time across all four controllers by exporting data.

5. **Gain sensitivity:** With PID on Circle, progressively increase Kp. Observe the transition from underdamped to critically damped to unstable.

---

## References

- Bouabdallah, S. "Design and control of quadrotors with application to autonomous flying." EPFL, 2007.
- Levant, A. "Sliding order and sliding accuracy in sliding mode control." *International Journal of Control*, 1993.
- Moreno, J.A. and Osorio, M. "Strict Lyapunov functions for the super-twisting algorithm." *IEEE TAC*, 2012.
- Camacho, E.F. and Bordons, C. *Model Predictive Control.* Springer, 2007.

---

## License

UVM — Use freely for education and research.
