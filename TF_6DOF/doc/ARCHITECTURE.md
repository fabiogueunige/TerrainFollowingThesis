# 6-DOF AUV Terrain Following System - Architecture

## Overview

This project implements a **6 Degree-of-Freedom (6-DOF) Autonomous Underwater Vehicle (AUV)** terrain-following control system using **Extended Kalman Filter (EKF)** for state estimation and **adaptive terrain generation** with a finite state machine for robust operation.

**Vehicle**: BlueROV2-inspired dynamics  
**Environment**: Underwater terrain following at constant altitude  
**Sensors**: 4 Single-Beam Echo Sounders (SBES), AHRS (Attitude Heading Reference System), DVL (Doppler Velocity Log)  
**Control**: PID-based with anti-windup and delta control  
**Estimation**: EKF for altitude and terrain angles (α, β)  

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        MAIN SIMULATION LOOP                              │
│                       (main_6DOF_3D.m)                                   │
└────┬───────────────────────────────────────────────────────────────┬────┘
     │                                                                │
     ├─> State Machine ──────────────────────────────────────────────┤
     │   (Idle → TargetAltitude → ContactSearch → Following)         │
     │   (Recovery states: MovePitch, MoveRoll, RecoveryAltitude)    │
     │                                                                │
     ├─> Goal Definition ────────────────────────────────────────────┤
     │   (Set surge, sway, altitude, roll, pitch, yaw targets)       │
     │                                                                │
     ├─> Controller ─────────────────────────────────────────────────┤
     │   (PID + Anti-windup + Delta control)                         │
     │                                                                │
     ├─> Robot Dynamics ─────────────────────────────────────────────┤
     │   (BlueROV2 model: mass, damping, buoyancy)                   │
     │                                                                │
     ├─> Sensors ────────────────────────────────────────────────────┤
     │   ├─ AHRS (roll, pitch, yaw)                                  │
     │   ├─ DVL (position, velocity)                                 │
     │   └─ SBES (4 beams: front, rear, left, right)                 │
     │                                                                │
     ├─> Terrain Generator ──────────────────────────────────────────┤
     │   (Dynamic plane generation with circular buffer)             │
     │                                                                │
     ├─> EKF Estimation ─────────────────────────────────────────────┤
     │   ├─ Prediction: f(x, u)                                      │
     │   ├─ Update: h(x) with SBES measurements                      │
     │   └─ State: [altitude, alpha, beta]                           │
     │                                                                │
     └─> Visualization ──────────────────────────────────────────────┘
         (3D plots: robot, sensors, terrain, trajectory)
```

---

## Module Breakdown

### 1. **Controller Module** (`controller/`)

Implements PID control with anti-windup and delta formulation.

| File | Purpose |
|------|---------|
| `gainComputation.m` | Compute PID gains (Kp, Ki, Kd, Kt) based on linearized AUV dynamics |
| `input_control.m` | Main control logic: error computation, anti-windup, delta control |
| `tau0_values.m` | Compute equilibrium forces/torques for linearization point |

**Key Features:**
- **Anti-windup**: Prevents integrator saturation
- **Delta control**: Incremental PID for smoother response
- **Frame transformations**: Control computed in terrain frame, executed in robot frame

---

### 2. **EKF Filter Module** (`ekf_filter/`)

Extended Kalman Filter for state estimation.

| File | Purpose |
|------|---------|
| `f.m` | State prediction function: `x_{k+1} = f(x_k, u_k)` |
| `h.m` | Measurement function: `z_k = h(x_k)` |
| `jacobian_f.m` | Jacobian of f w.r.t. state (for covariance propagation) |
| `jacobian_h.m` | Jacobian of h w.r.t. state (for Kalman gain) |

**State Vector:**
```
x = [altitude, alpha, beta]'
```
- `altitude`: Distance from terrain [m]
- `alpha`: Terrain roll angle [rad]
- `beta`: Terrain pitch angle [rad]

**Measurement Vector:**
```
z = [y1, y2, y3, y4]'
```
Range measurements from 4 SBES sensors [m]

---

### 3. **Math Functions** (`math_function/`)

Utility functions for numerical operations.

| File | Purpose |
|------|---------|
| `derivator.m` | Discrete derivative (Tustin method) |
| `integrator.m` | Discrete integral (trapezoidal rule) |
| `intersection_check.m` | Check if ray-plane intersection is within segment |
| `plane_computation.m` | Compute plane normal from sensor intersection points |
| `reference_correction.m` | Ensure terrain normal points downward (z > 0 check) |
| `vector_normalization.m` | Normalize vector to unit length |

---

### 4. **Robot Dynamics** (`model/`)

BlueROV2 dynamics model.

| File | Purpose |
|------|---------|
| `dynamic_model.m` | Compute accelerations from forces/torques (linearized) |
| `kinematic_model.m` | (Empty placeholder for future use) |

**Parameters:**
- Mass: 11.5 kg
- Inertia: diag([0.21, 0.245, 0.245]) kg⋅m²
- Added mass, linear damping, quadratic damping

---

### 5. **Noise Module** (`noise/`)

| File | Purpose |
|------|---------|
| `noise_setup.m` | Configure Q (process noise) and R (measurement noise) matrices |

**Tuning:**
- Process noise: altitude (0.099 m), angles (0.5°)
- SBES noise: ~0.18 m per sensor
- AHRS noise: 0.5° (roll/pitch), 0.08° (yaw)

---

### 6. **Rotation Utilities** (`rotation/`)

3D rotation matrices and derivatives.

| File | Purpose |
|------|---------|
| `rotx.m`, `roty.m`, `rotz.m` | Rotation matrices around X, Y, Z axes |
| `d_rotx.m`, `d_roty.m`, `d_rotz.m` | Derivatives for Jacobian computation |

**Convention:** Right-hand rule, ZYX Euler angles

---

### 7. **Sensors** (`sensors/`)

Simulate AUV sensors.

| File | Purpose |
|------|---------|
| `AHRS_measurement.m` | Measure roll, pitch, yaw (with optional noise) |
| `DVL_measurament.m` | Measure position and velocity in world frame |
| `SBES_definition.m` | Define 4 SBES beam directions (±22.5° from vertical) |
| `SBES_measurament.m` | Ray-casting to find terrain intersections, compute measurements |
| `measurament.m` | (Legacy, not actively used) |

**SBES Configuration:**
```
Sensor 1 (rear):   pitch = -22.5°
Sensor 2 (front):  pitch = +22.5°
Sensor 3 (left):   roll = +22.5°
Sensor 4 (right):  roll = -22.5°
```

---

### 8. **State Machine** (`state_machine/`)

Finite state machine for mission management.

| File | Purpose |
|------|---------|
| `state_machine.m` | State transitions based on command flags |
| `goal_def.m` | Define velocity/orientation goals per state |
| `goal_controller.m` | Update command flags based on sensor status |

**States:**
1. **Idle**: Waiting for start
2. **TargetAltitude**: Descend to target altitude
3. **ContactSearch**: Search for terrain contact
4. **Following**: Terrain-following mode
5. **MovePitch / MoveRoll**: Recover from sensor loss
6. **RecoveryAltitude**: Altitude adjustment recovery
7. **Emergency**: Safety mode (altitude too low)
8. **EndSimulation**: Terminate

**Recovery Logic:**
- Grace period (0.5s) for transient sensor failures
- Timeout (5s) for recovery maneuvers
- Escalation: MovePitch/Roll → RecoveryAltitude → Reset

---

### 9. **Terrain Generation** (`world_generator/`)

Dynamic terrain with circular buffer.

| File | Purpose |
|------|---------|
| `terrain_init.m` | Initialize terrain with first planes |
| `terrain_generator.m` | Generate new planes ahead of robot |

**Features:**
- Circular buffer (500 planes max)
- Smooth angle changes (rate-limited)
- Direction based on robot velocity
- Maintains 15m+ ahead of robot

---

### 10. **Visualization** (`visualization/`)

Plotting and debugging.

| File | Purpose |
|------|---------|
| `m_visualization.m` | 3D plot of robot, sensors, terrain patches |
| `plot_results.m` | End-of-simulation plots (states, angles, parallelism) |
| `printDebug.m` | Conditional debug printing (global DEBUG flag) |

---

## Data Flow

### Single Iteration (k):

```
1. State Machine Decision
   └─> next_state = state_machine(state_{k-1}, cmd, k)

2. Goal Definition
   └─> goal = goal_def(state_k, rob_rot_{k-1}, x_est_{k-1}, k)

3. Control Computation
   └─> pid_k = input_control(goal, x_est_{k-1}, rob_rot_{k-1}, ...)

4. Dynamics Update
   └─> u_k = dynamic_model(pid_k, ...)
   └─> rob_rot_k = AHRS_measurement(rob_rot_{k-1}, u_k, Ts)
   └─> p_robot_k = DVL_measurement(p_robot_{k-1}, u_k, wRr, Ts)

5. Terrain Update
   └─> planes = terrain_generator(planes, p_robot_k, vel_w, ...)

6. Sensor Measurements
   └─> z_meas_k = SBES_measurement(planes, p_robot_k, wRr, ...)
       ├─> cmd.contact(1:4) updated
       └─> cmd.sensor_fail updated

7. EKF Prediction
   └─> x_pred_k = f(x_est_{k-1}, u_k, Ts, wRr_k)
   └─> P_pred = F * P_{k-1} * F' + Q

8. EKF Update
   └─> K = P_pred * H' / (H * P_pred * H' + R)
   └─> x_est_k = x_pred_k + K * (z_meas_k - h(x_pred_k))

9. Goal Controller
   └─> cmd = goal_controller(cmd, x_est_k, rob_rot_k, goal_k, ...)
       ├─> Updates diagnostic flags for k+1
       └─> Triggers state transitions
```

---

## Coordinate Frames

### World Frame (W)
- Origin: Sea surface
- Z-axis: Downward (depth positive)
- X-axis: North (forward)
- Y-axis: East (lateral)

### Robot Frame (R)
- Origin: Robot center of mass
- X-axis: Forward (surge)
- Y-axis: Lateral (sway)
- Z-axis: Downward (heave)

### Terrain Frame (T)
- Origin: Point on terrain plane
- Z-axis: Normal to terrain (downward)
- X, Y: Tangent to terrain

**Rotation Convention:** ZYX Euler angles (yaw-pitch-roll)

---

## Key Parameters

### Simulation
- **Sampling time**: 1 ms (Ts = 0.001 s)
- **Duration**: 40 s (N = 40,000 iterations)
- **Target altitude**: 3 m

### Robot
- **Surge speed**: 0.3 m/s (following), 0.02 m/s (recovery)
- **Sway speed**: -0.2 m/s (following), -0.05 m/s (recovery)

### Terrain
- **Angle range**: ±30° (alpha, beta)
- **Step length**: 4 m between planes
- **Buffer size**: 500 planes

### EKF
- **Initial covariance**: P0 = diag([1, 0.08, 2]) + noise
- **Process noise**: Q = diag([0.01, 0.0001, 0.00025])
- **Measurement noise**: R = diag([0.031, 0.034, 0.031, 0.034])

---

## Performance Metrics

Computed in `plot_results.m`:

1. **State tracking**: altitude, alpha, beta vs desired
2. **Robot angles**: roll, pitch, yaw
3. **Control inputs**: u, v, w, p, q, r
4. **Normal parallelism**: angle between n_est, n_mes, z_robot
5. **Sensor failures**: count and percentage
6. **Trajectory**: 3D path colored by time

---

## Future Improvements

1. **Kalman Filter for position**: Currently uses perfect DVL
2. **Non-linear dynamics**: Full 6-DOF model (commented out in `dynamic_model.m`)
3. **Obstacle avoidance**: Add static/dynamic obstacles
4. **Multi-AUV**: Cooperative terrain following
5. **Real sensor data**: Integration with ROS/Gazebo

---

## Dependencies

- **MATLAB**: R2020b or later recommended
- **Toolboxes**: None required (custom rotation matrices)
- **External libraries**: None

---

## Quick Start

```matlab
% Open MATLAB in project root
cd matlab_3D
main_6DOF_3D  % Run simulation (40s, ~40,000 iterations)
```

**Output:**
- Console: State transitions, diagnostics
- Figures: States, angles, inputs, normal analysis, 3D trajectory

---

## License & Citation

If you use this code, please cite:
```
[Your thesis/paper citation here]
```

---

**Author**: Fabio Gueunige  
**Project**: Terrain Following Thesis (TFThesis)  
**Last Updated**: 2025-01-23
