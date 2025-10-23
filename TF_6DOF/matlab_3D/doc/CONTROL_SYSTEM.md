# PID Control System - Technical Documentation

## Overview

The AUV uses a **PID controller with anti-windup and delta formulation** to track desired altitude and terrain orientation. Control is computed in the terrain frame and executed in the robot frame.

---

## Control Architecture

```
Goal Definition → Error Computation → PID Controller → Dynamic Model → Robot
     ↑                                                                    ↓
     └────────────────────── State Feedback ────────────────────────────┘
```

**Files:**
- `controller/gainComputation.m`: Compute PID gains from robot dynamics
- `controller/input_control.m`: Main control logic
- `controller/tau0_values.m`: Equilibrium forces/torques
- `model/dynamic_model.m`: Robot dynamics (acceleration from forces)

---

## State & Input Vectors

### State (from EKF)
```
x = [altitude, alpha, beta]'
```

### Goal (from state machine)
```
goal = {surge, sway, altitude, roll, pitch, yaw}
```

### Control Input (to dynamics)
```
pid = [surge_ref, sway_ref, heave_ref, roll_ref, pitch_ref, yaw_ref]'
```
Units: [m/s, m/s, m/s, rad/s, rad/s, rad/s]

### Robot Velocities
```
u = [u, v, w, p, q, r]'
```

---

## PID Formulation

### Classic PID

For each DOF i:
```
e_i(k) = goal_i - state_i(k)

PID_i(k) = Kp_i * e_i(k) + Ki_i * ∫e_i(t)dt + Kd_i * de_i(k)/dt
```

### Delta PID (Anti-Windup)

**Used for surge, sway, heave, roll, pitch**:

```
term_i(k) = Ki_i * e_i(k) - Kp_i * state_dot_i(k) - Kd_i * state_ddot_i(k) - Kt_i * pid_i(k-1)

integral_i(k) = integral_i(k-1) + term_i(k) * Ts

pid_i(k) = saturate(integral_i(k), [-1, +1])
```

**Anti-windup**: 
```
integral_i(k) = pid_i(k)  % Reset to saturated value
```

**Advantage**: Prevents integrator wind-up when control saturates.

---

## Gain Computation

**File**: `controller/gainComputation.m`

### Robot Parameters (BlueROV2)

```matlab
m = 11.5 kg              % Total mass
I = diag([0.21, 0.245, 0.245]) kg⋅m²  % Inertia tensor
B = ρ * V * g = 111.0 N  % Buoyancy
z_B = 0.042 m            % Buoyancy center offset
```

### Damping Model

```matlab
% Added mass
tau_a = -[27.08, 25.952, 29.9081, 1, 1, 1]'

% Linear damping
tau_r = -[0.1213, 1.1732, 1.1130, 0.5, 0.5, 0.5]'

% Quadratic damping
tau_d = -[23.9, 46.27, 50.278, 1, 1, 1]'

% Virtual mass
m_v = [m, m, m, I_xx, I_yy, I_zz]' - tau_a

% Dissipative forces (at linearization point v0)
d_v = -tau_r - tau_d .* abs(v0)
d_v_lin = -tau_r - 2 * tau_d .* abs(v0)
```

### PID Gains

**Surge & Sway** (PI control):
```matlab
ω_n = 0.4     % Natural frequency [rad/s]
ζ = 0.6       % Damping ratio

Kp = 2 * ζ * ω_n * m_v - d_v_lin
Ki = ω_n² * m_v
Kd = 0
```

**Heave, Roll, Pitch** (PID control):
```matlab
p = 10        % Pole placement parameter

Kp = m_v * (ω_n² + 2*ζ*p*ω_n)
Ki = p * ω_n² * m_v
Kd = (p + 2*ζ*ω_n) * m_v - d_v_lin
```

**Anti-windup gain**:
```matlab
Ti = Kp / Ki
Td = Kd / Kp
Kt = 1 / sqrt(Ti * Td)
```

**Typical values:**
```
Surge:  Kp = 6.5,  Ki = 2.0,  Kd = 0,    Kt = 0.55
Heave:  Kp = 26.0, Ki = 5.2,  Kd = 12.0, Kt = 0.20
Roll:   Kp = 2.5,  Ki = 0.4,  Kd = 0.8,  Kt = 0.56
```

---

## Control Loop

**File**: `controller/input_control.m`

### Frame Transformations

**Terrain frame velocities:**
```matlab
s_speed = wRt' * wRr * u(SURGE:HEAVE)
s_acc = wRt' * wRr * u_dot(SURGE:HEAVE)
```

**Why terrain frame?**
- Goal altitude is relative to terrain normal
- Simplifies error computation (heave error = altitude error)

### Error Computation

```matlab
err(SURGE) = goal.surge - s_speed(SURGE)
err(SWAY) = goal.sway - s_speed(SWAY)
err(HEAVE) = goal.altitude - x(ALTITUDE)
err(ROLL) = goal.roll - rob_rot(PHI)
err(PITCH) = goal.pitch - rob_rot(THETA)
err(YAW) = goal.yaw - rob_rot(PSI)
```

### Delta Control Terms

**For surge/sway:**
```matlab
i_err = Ki(j) * err(j)
p_err = Kp(j) * s_acc(j)  % Acceleration feedback
d_err = 0
term(j) = i_err - p_err - Kt(j)*pid_old(j)
```

**For heave:**
```matlab
i_err = Ki(j) * err(j)
p_err = Kp(j) * s_speed(j)  % Velocity feedback
d_err = Kd(j) * s_acc(j)    % Acceleration feedback
term(j) = i_err - p_err - d_err - Kt(j)*pid_old(j)
```

**For roll/pitch:**
```matlab
i_err = Ki(j) * err(j)
p_err = Kp(j) * u(j)  % Angular velocity feedback
d_err = Kd(j) * u_dot(j)  % Angular acceleration feedback
term(j) = i_err - p_err - d_err - Kt(j)*pid_old(j)
```

### Integration & Saturation

```matlab
integral(j) = integral_old(j) + term(j) * Ts

pid_sat = saturate(integral(j), [-1, +1])

pid(j) = integral(j) - pid_sat  % Anti-windup correction
```

**Saturation limits:**
```
max_pid = [1, 1, 1, 1, 1, 1]  % All DOFs
```

### Robot Frame Transformation

**Control computed in terrain frame, transform back:**
```matlab
tp_speed = wRr' * wRt * [term(SURGE); term(SWAY); term(HEAVE)]

term(SURGE) = tp_speed(1)
term(SWAY) = tp_speed(2)
term(HEAVE) = tp_speed(3)
```

---

## Linearized Dynamics

**File**: `model/dynamic_model.m`

### Equations of Motion

**General form:**
```
m_v * a = τ - d_v * v
```

**Linearized around (v0, τ0):**
```
Δτ = τ - τ0
Δv = v - v0

a = (Δτ - d_v_lin * Δv) / m_v
```

### Specific DOFs

**Surge:**
```matlab
a_u = (tau(SURGE) - tau0(SURGE) - d_v_lin(U) * (u - u0)) / m_v(U)
```

**Heave:**
```matlab
a_w = (tau(HEAVE) - tau0(HEAVE) - d_v_lin(W) * (w - w0)) / m_v(W)
```

**Roll:**
```matlab
a_p = (tau(ROLL) - tau0(ROLL) - d_v_lin(P) * (p - p0)) / m_v(P)
```

**Pitch:**
```matlab
a_q = (tau(PITCH) - tau0(PITCH) - d_v_lin(Q) * (q - q0)) / m_v(Q)
```

### Velocity Integration

```matlab
u_new = integrator(u_old, a, a_old, Ts)
```

**Trapezoidal rule:**
```
u_{k+1} = u_k + (a_k + a_{k-1}) * Ts / 2
```

---

## Equilibrium Forces

**File**: `controller/tau0_values.m`

### Linearization Point

**Typical operating point:**
```
u0 = [0.2, 0.1, 0, 0, 0, 0]'  % Slow forward + lateral motion
```

### Equilibrium Computation

**Surge:**
```matlab
tau0(SURGE) = d_v(U) * u0(U)
```

**Heave:**
```matlab
tau0(HEAVE) = d_v(W) * u0(W)
```

**Roll:**
```matlab
tau0(ROLL) = d_v(P) * u0(P) + z_B * B * sin(θ0) * sin(φ0)
```

**Pitch:**
```matlab
tau0(PITCH) = d_v(Q) * u0(Q) + z_B * B * sin(θ0)
```

**Interpretation:**
- tau0 counteracts damping at operating point
- Restoring moment from buoyancy offset (z_B ≠ 0)

---

## Tuning Guidelines

### Step 1: Identify System Parameters

From robot specifications or experiments:
- Mass (m)
- Inertia (I)
- Damping coefficients (tau_r, tau_d)
- Buoyancy (B, z_B)

### Step 2: Choose Desired Response

**Natural frequency (ω_n):**
- Higher → faster response, more oscillations
- Lower → sluggish, stable

**Damping ratio (ζ):**
- 0.6-0.8 → slight overshoot, good settling time
- 1.0 → critically damped (no overshoot)

**Typical:**
```
ω_n = 0.4 rad/s  (~1.6s settling time)
ζ = 0.6          (10% overshoot)
```

### Step 3: Compute Gains

Run `gainComputation.m` with chosen parameters.

### Step 4: Test & Iterate

**Symptoms:**
- **Oscillations** → Reduce ω_n or increase ζ
- **Sluggish** → Increase ω_n
- **Overshoot** → Increase ζ or reduce Kp
- **Steady-state error** → Increase Ki

**Debug:**
```matlab
global DEBUG;
DEBUG = true;
```
Monitor error, PID terms in console.

---

## State Machine Integration

### Goal Definition per State

**File**: `state_machine/goal_def.m`

| State | Surge | Sway | Altitude | Roll | Pitch | Yaw |
|-------|-------|------|----------|------|-------|-----|
| **Idle** | 0 | 0 | h_ref | 0 | 0 | 0 |
| **TargetAltitude** | 0 | 0 | h_ref | φ | θ | ψ |
| **ContactSearch** | 0.2 | -0.15 | h_ref | α_est | β_est | ψ |
| **Following** | 0.3 | -0.2 | h_ref | α_est | β_est | 0 |
| **MovePitch** | 0.02 | -0.05 | h_ref | α_est | β_est ± π/12 | ψ |
| **MoveRoll** | 0.02 | -0.1 | h_ref | α_est ± π/12 | β_est | ψ |
| **RecoveryAltitude** | 0.02 | -0.05 | h_ref - 1 | α_est | β_est | ψ |
| **Emergency** | 0 | 0 | h_ref | φ | θ | ψ |

**Key:**
- α_est, β_est: EKF-estimated terrain angles
- h_ref: Reference altitude (typically 3m)
- π/12 = 15°: Recovery angle adjustment

---

## Performance Metrics

### Tracking Error

**Altitude:**
```
e_h = |h_desired - h_actual|
```
**Typical:** < 0.5 m (steady-state)

**Orientation:**
```
e_φ = |φ_desired - φ_actual|
e_θ = |θ_desired - θ_actual|
```
**Typical:** < 5° (transient), < 1° (steady-state)

### Control Effort

**Saturation frequency:**
```
saturation_rate = sum(abs(pid) >= 0.9) / N * 100
```
**Typical:** < 5% of samples

### Settling Time

**Time to reach ±5% of final value:**
```
t_settle ≈ 4 / (ζ * ω_n) = 4 / (0.6 * 0.4) ≈ 17 seconds
```

---

## Advanced Topics

### Non-Linear Dynamics (Commented Out)

Full 6-DOF model in `dynamic_model.m`:

```matlab
% Surge (with Coriolis)
a_u = (tau(U) + m_v(V)*v*r - m_v(W)*w*q - d_v(U)*u) / m_v(U)

% Roll (with coupling)
a_p = (tau(P) + (m_v(V)-m_v(W))*v*w + (m_v(Q)-m_v(R))*q*r - d_v(P)*p - z_B*B*cos(θ)*sin(φ)) / m_v(P)
```

**Coriolis terms**: (m_v × v) coupling between DOFs

**When to use:**
- High-speed maneuvers
- Large angles (> 30°)
- Accurate trajectory tracking

### Adaptive Control

**Idea**: Estimate damping coefficients online

```matlab
% Parameter estimator (RLS)
θ_hat = (R' * R)^(-1) * R' * y

% Update control gains
Kp_adaptive = Kp_nominal * (θ_hat / θ_nominal)
```

**Not implemented** (future work).

### Model Predictive Control (MPC)

**Advantage**: Optimize over future horizon, handle constraints

**Challenge**: Computational cost (solve QP at 1 kHz)

---

## Debugging Tools

### Print Control Terms

```matlab
fprintf('Error: surge=%.2f sway=%.2f heave=%.2f\n', err(SURGE), err(SWAY), err(HEAVE));
fprintf('PID: u=%.3f v=%.3f w=%.3f p=%.3f q=%.3f r=%.3f\n', pid(1), pid(2), pid(3), pid(4), pid(5), pid(6));
```

### Plot Control History

**In `plot_results.m`:**
```matlab
figure; plot(time, u(SURGE,:)); title('Surge Velocity');
figure; plot(time, u(HEAVE,:)); title('Heave Velocity');
```

### Check Saturation

```matlab
if abs(pid(j)) >= 0.9
    warning('DOF %d saturated at step %d', j, k);
end
```

---

## Common Issues

### Issue 1: Oscillations

**Symptom**: Altitude/angles oscillate around setpoint  
**Cause**: Kp too high, ω_n too high, or ζ too low  
**Solution**: Reduce ω_n from 0.4 to 0.3, or increase ζ from 0.6 to 0.8

### Issue 2: Steady-State Error

**Symptom**: Never reaches exact setpoint  
**Cause**: Ki too low, or saturation preventing integral action  
**Solution**: Increase Ki, or check saturation frequency

### Issue 3: Sluggish Response

**Symptom**: Takes > 20s to reach altitude  
**Cause**: ω_n too low  
**Solution**: Increase ω_n from 0.4 to 0.6 (test stability)

### Issue 4: Integral Wind-Up

**Symptom**: Large overshoot after saturation  
**Cause**: Anti-windup not working  
**Solution**: Verify `pid(j) = integral(j) - pid_sat` line executes

---

## References

1. **PID Tuning**: Åström & Hägglund, "PID Controllers: Theory, Design, and Tuning"
2. **Marine Vehicle Control**: Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control"
3. **Anti-Windup**: Åström & Wittenmark, "Computer-Controlled Systems"

---

**Related Files:**
- `controller/gainComputation.m`
- `controller/input_control.m`
- `controller/tau0_values.m`
- `model/dynamic_model.m`
- `state_machine/goal_def.m`

---

**Author**: Fabio Gueunige  
**Last Updated**: 2025-01-23
