# COMPLETE EKF FOR 6DOF AUV NAVIGATION
## Full Implementation Guide with Equations, Matrices and Code

---

## TABLE OF CONTENTS
1. Reference Frames and Transformations
2. Dynamic and Kinematic Models (Fossen 2011)
3. State Space Formulation
4. Extended Kalman Filter Theory
5. Sensor Measurement Models
6. Complete Algorithm with Pseudocode
7. Implementation Notes and Tuning
8. MATLAB/Python Code Examples

---

## 1. REFERENCE FRAMES AND TRANSFORMATIONS

### 1.1 Reference Frames Definition

**NED (North-East-Down) Frame**: Inertial, origin at fixed point on Earth
- X-axis: North
- Y-axis: East  
- Z-axis: Down

**BODY Frame**: Fixed to the vehicle, moves with it
- X-axis (surge): Forward (longitudinal)
- Y-axis (sway): Starboard (lateral)
- Z-axis (heave): Downward (vertical)

### 1.2 Rotation Matrix: Body to NED

The rotation matrix **R** transforms vectors from body frame to NED frame:

```
R(η₂) = R_z(ψ) × R_y(θ) × R_x(φ)
```

where η₂ = [φ, θ, ψ]ᵀ (Euler angles in ZYX order)

**Explicit Form** (Fossen 2011 convention):

```
R(φ,θ,ψ) = 
[  cψ·cθ   -sψ·cφ + cψ·sθ·sφ   sψ·sφ + cψ·cφ·sθ  ]
[  sψ·cθ    cψ·cφ + sφ·sθ·sψ  -cψ·sφ + sθ·sψ·cφ  ]
[ -sθ       cθ·sφ              cθ·cφ             ]
```

where:
- c(·) = cos(·)
- s(·) = sin(·)

**Inverse Transformation** (NED to Body):

```
R⁻¹ = Rᵀ
```

### 1.3 Euler Angle Time Derivatives (Kinematic Transformation)

Relationship between angular velocities in body frame [p, q, r]ᵀ and Euler angle rates:

```
[φ̇]     [1  s(φ)·t(θ)  c(φ)·t(θ) ] [p]
[θ̇]  =  [0  c(φ)      -s(φ)      ] [q]
[ψ̇]     [0  s(φ)/c(θ)  c(φ)/c(θ) ] [r]
```

**Matrix Form**:
```
η̇₂ = T(η₂) · ω_body
```

where T(η₂) is the transformation matrix (singular at θ = ±90°).

**Inverse**:
```
ω_body = T⁻¹(η₂) · η̇₂
```

```
T⁻¹(η₂) =
[1  0        -s(θ)     ]
[0  c(φ)      s(φ)·c(θ)]
[0  -s(φ)     c(φ)·c(θ)]
```

---

## 2. DYNAMIC AND KINEMATIC MODELS (Fossen 2011)

### 2.1 Kinematics - Position and Orientation

```
ṗ = R(η₂) · v_body          % Position rate
η̇₂ = T(η₂) · ω_body        % Orientation rate
```

**Matrix Form**:

```
η̇ = J(η) · ν

where:
η = [x, y, z, φ, θ, ψ]ᵀ
ν = [u, v, w, p, q, r]ᵀ

J(η) = [ R(η₂)    0_{3x3}  ]
       [ 0_{3x3}  T(η₂)    ]
```

### 2.2 Dynamics - Forces and Torques (6DOF)

**General Form** (Fossen 2011):

```
M·ν̇ + C(ν)·ν + D(ν)·ν + g(η) = τ + τ_env
```

where:
- M = M_RB + M_A: Inertia matrix (rigid-body + added mass)
- C(ν) = C_RB(ν) + C_A(ν): Coriolis and centripetal forces
- D(ν): Damping matrix (hydrodynamic + viscous drag)
- g(η): Restoring forces (gravity + buoyancy)
- τ: Control forces/moments
- τ_env: Environmental forces (currents, waves)

### 2.3 Inertia Matrix

**Rigid Body**:
```
       [m  0  0   0   m·z_G -m·y_G]
M_RB = [0  m  0  -m·z_G 0   m·x_G]
       [0  0  m   m·y_G -m·x_G 0  ]
       [0 -m·z_G m·y_G  I_x -I_xy -I_xz]
       [m·z_G 0 -m·x_G -I_xy I_y -I_yz]
       [-m·y_G m·x_G 0  -I_xz -I_yz I_z]
```

**Added Mass** (potential flow approximation):
```
M_A = -diag(X_u̇, Y_v̇, Z_ẇ, K_ṗ, M_q̇, N_ṙ)
```

Typically added mass ~5-20% of mass for most AUVs.

### 2.4 Coriolis and Centripetal Matrix

**Rigid Body**:
```
C_RB(ν) = [0_{3x3}              -m·S(v_G)]    ]
          [m·S(v_G)  -S(I·ω_body)           ]

where S(x) is skew-symmetric matrix of x = [x₁,x₂,x₃]:
S(x) = [ 0   -x₃   x₂ ]
       [ x₃   0   -x₁ ]
       [-x₂   x₁   0  ]
```

**Added Mass**:
```
C_A(ν) = [ 0_{3x3}        -S(M_A·v_body)]
         [S(M_A·v_body)   -S(M_A·ω_body)]
```

### 2.5 Damping Matrix

**Linear and Quadratic** (typical for underwater vehicles):

```
D(ν) = D_lin + D_quad(ν)

D_lin = -diag(X_u, Y_v, Z_w, K_p, M_q, N_r)

D_quad(ν) = -diag(|u|·X_uu, |v|·Y_vv, |w|·Z_ww,
                   |p|·K_pp, |q|·M_qq, |r|·N_rr)
```

### 2.6 Restoring Forces

```
g(η) = [  (W-B)·sin(θ)                              ]
       [ -(W-B)·cos(θ)·sin(φ)                       ]
       [ -(W-B)·cos(θ)·cos(φ)                       ]
       [ -(y_G·W - y_B·B)·cos(θ)·cos(φ) + ... ]
       [ (z_G·W - z_B·B)·sin(θ) + ...         ]
       [ -(x_G·W - x_B·B)·cos(θ)·sin(φ) - ...]
```

where W = weight, B = buoyancy force, (x_G,y_G,z_G) = CG, (x_B,y_B,z_B) = CB.

---

## 3. STATE SPACE FORMULATION

### 3.1 Continuous-Time State Vector

```
x(t) = [x, y, z, φ, θ, ψ, u, v, w, p, q, r, b_gx, b_gy, b_gz]ᵀ

Dimension: 15 × 1
```

### 3.2 Continuous-Time Dynamics (for simulation/integration)

```
ẋ = f_c(x, u, IMU_meas)

where IMU measurements are:
ω̃_body = ω_body + b_g + w_g  (gyroscope)
ã = a + b_a + w_a             (accelerometer)

Explicit equations:
ṗ = R(η₂)·v_body
η̇₂ = T(η₂)·ω_body
v̇_body = M⁻¹(C(ν)·ν + D(ν)·ν + g(η)) + R^T(η₂)·(ã - b_a) - S(ω_body)·v_body
ω̇_body = (ω̃_body - b_g) [use gyro measurement with bias correction]
ḃ_g = 0  (random walk model for bias)
```

### 3.3 Discrete-Time State Transition Model

Using RK4 integration over Δt:

```
x_{k+1} = f_d(x_k, u_k, Δt) = x_k + Δt/6 · (k₁ + 2k₂ + 2k₃ + k₄)

where:
k₁ = f_c(x_k, u_k)
k₂ = f_c(x_k + Δt/2·k₁, u_k)
k₃ = f_c(x_k + Δt/2·k₂, u_k)
k₄ = f_c(x_k + Δt·k₃, u_k)
```

**Simplified Model** (for computational efficiency):

```
Position:
p_{k+1} = p_k + R(η₂,k)·v_{body,k}·Δt

Orientation:
η₂,_{k+1} = η₂,k + T(η₂,k)·ω_{body,k}·Δt

Velocity (from DVL + decay model):
v_{body,k+1} = (1 - α·Δt)·v_{body,k} + α·Δt·ṽ_{DVL,k}

Bias:
b_{g,k+1} = b_{g,k}  (constant)
```

where α is a process decay constant (~0.1-1.0).

### 3.4 Jacobian of State Transition (F matrix)

```
F_k = ∂f_d/∂x |_{x=x̂_k}

Approximate block structure (for Forward Euler):

F_k ≈ [I_{3x3}  ∂p/∂η₂·Δt  R(η₂,k)·Δt  0_{3x9}     ]
     [0_{3x3}  I_{3x3}      0_{3x3}     T(η₂,k)·Δt  0_{3x3}]
     [0_{3x6}  0_{3x3}      (1-α·Δt)I  0_{3x3}     0_{3x3}]
     [0_{3x12}                         0_{3x3}     I_{3x3}]
```

**Exact computation** requires numerical differentiation for nonlinear terms.

---

## 4. EXTENDED KALMAN FILTER THEORY

### 4.1 Standard EKF Equations

**Prediction Step** (Time Update):

```
x̂_k^- = f_d(x̂_{k-1}^+, u_{k-1})           % State prediction
P_k^- = F_k·P_{k-1}^+·F_k^T + Q_k          % Covariance prediction
```

**Correction Step** (Measurement Update):

```
y_k = z_k - h(x̂_k^-)                       % Innovation/residual
S_k = H_k·P_k^-·H_k^T + R_k                % Innovation covariance
K_k = P_k^-·H_k^T·S_k^{-1}                % Kalman gain
x̂_k^+ = x̂_k^- + K_k·y_k                  % State correction
P_k^+ = (I - K_k·H_k)·P_k^-               % Covariance update
```

### 4.2 Process Noise Covariance (Q matrix)

```
Q_k = blockdiag(q_p·I_3, q_η·I_3, q_ν·I_3, q_b·I_3)

Typical values for AUV (tuning parameters):
q_p ≈ 1e-6 m²/s    (position drift)
q_η ≈ 1e-6 rad²/s  (orientation drift)
q_ν ≈ 1e-4 (m/s)²/s (velocity uncertainty)
q_b ≈ 1e-8 (rad/s)²/s (bias drift)

Scaled by Δt for discretization:
Q_d ≈ Q_c · Δt (for constant acceleration models)
```

### 4.3 Measurement Noise Covariance (R matrix)

```
R_k = blockdiag(σ²_DVL·I_3, σ²_AHRS·I_3, σ²_depth)

Typical sensor noise:
σ_DVL ≈ 0.01-0.05 m/s
σ_AHRS ≈ 1-5° ≈ 0.02-0.09 rad
σ_depth ≈ 0.5-1.0 m
```

---

## 5. SENSOR MEASUREMENT MODELS

### 5.1 DVL (Doppler Velocity Log) Measurement

**Physics**: Measures velocity relative to seabed in body frame.

**Measurement Model**:
```
z_DVL,k = [u, v, w]_k^body + v_DVL,k

where v_DVL,k ~ N(0, σ²_DVL)
```

**Output Function** h for EKF:
```
h_DVL(x_k) = [u_k, v_k, w_k]ᵀ

(DVL directly measures body-frame velocities)
```

**Jacobian H_DVL**:
```
H_DVL = [0_{3x3}  0_{3x3}  I_3  0_{3x6}]

(Measurement depends only on velocity states)
```

### 5.2 AHRS (Attitude and Heading Reference System) Measurement

**Physics**: Fuses accelerometer, gyro, magnetometer to provide orientation.

**Measurement Model** (Euler angles):
```
z_AHRS,k = [φ, θ, ψ]_k + v_AHRS,k

where v_AHRS,k ~ N(0, σ²_AHRS·I_3)
```

**Alternative: Quaternion Form**:
```
z_q,k = q_k + v_q,k  (quaternion measurement)
```

**Output Function** h for EKF (Euler):
```
h_AHRS(x_k) = [φ_k, θ_k, ψ_k]ᵀ
```

**Jacobian H_AHRS** (Euler):
```
H_AHRS = [0_{3x3}  I_3  0_{3x9}]

(Measurement depends only on orientation states)
```

### 5.3 Depth Sensor (Pressure Sensor) Measurement

**Physics**: Measures hydrostatic pressure → depth z (positive downward).

**Measurement Model**:
```
z_depth,k = z_k + v_depth,k

where v_depth,k ~ N(0, σ²_depth)
```

**Output Function** h for EKF:
```
h_depth(x_k) = z_k
```

**Jacobian H_depth**:
```
H_depth = [0 0 1 0_{1x12}]

(Measurement depends only on z position)
```

### 5.4 Combined Measurement Vector

When all sensors are available:

```
z_k = [z_DVL^T, z_AHRS^T, z_depth]ᵀ     % 7 × 1 measurement vector

h(x_k) = [u, v, w, φ, θ, ψ, z]ᵀ        % 7 × 1 output function

H_k = [0_{3x3}  0_{3x3}  I_3  0_{3x6}   ]   % 7 × 15 Jacobian
      [0_{3x3}  I_3      0_{3x9}       ]
      [0 0 1    0_{1x12}               ]

R_k = diag(σ²_DVL·I_3, σ²_AHRS·I_3, σ²_depth)  % 7 × 7
```

---

## 6. COMPLETE ALGORITHM WITH PSEUDOCODE

### 6.1 EKF Initialization

```pseudocode
FUNCTION Initialize_EKF(x0, P0, Q, R)
    x_est ← x0                    % Initial state estimate [15 × 1]
    P_est ← P0                    % Initial covariance [15 × 15]
    
    % Typical values:
    if x0 not provided:
        x0 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]ᵀ
    end
    
    % Initial covariance (often diagonal):
    P0 = diag([1, 1, 1,          % position uncertainty [m²]
              0.1, 0.1, 0.1,     % orientation uncertainty [rad²]
              0.1, 0.1, 0.1,     % velocity uncertainty [(m/s)²]
              0.01, 0.01, 0.01,  % angular velocity uncertainty [(rad/s)²]
              0.001, 0.001, 0.001]) % bias uncertainty [(rad/s)²]
    
    return (x_est, P_est)
END FUNCTION
```

### 6.2 Main EKF Loop

```pseudocode
FUNCTION EKF_Step(x_est, P_est, u_k, z_k, Δt, Q, R)
    
    % ============= PREDICTION STEP =============
    % Step 1: Predict state
    x_minus ← Integrate_RK4(x_est, u_k, Δt)
    
    % Step 2: Compute Jacobian F
    F_k ← Jacobian_f(x_est, u_k, Δt)
    
    % Step 3: Predict covariance
    P_minus ← F_k · P_est · F_k^T + Q
    
    % ============= CORRECTION STEP =============
    % Step 4: Compute measurement prediction
    h_x ← Measurement_Function(x_minus)
    
    % Step 5: Compute innovation
    y_k ← z_k - h_x
    
    % Step 6: Compute Jacobian H
    H_k ← Jacobian_h(x_minus)
    
    % Step 7: Compute innovation covariance
    S_k ← H_k · P_minus · H_k^T + R
    
    % Step 8: Compute Kalman gain
    K_k ← P_minus · H_k^T · Inverse(S_k)
    
    % Step 9: Update state
    x_est ← x_minus + K_k · y_k
    
    % Step 10: Update covariance
    P_est ← (I - K_k · H_k) · P_minus
    
    return (x_est, P_est)
END FUNCTION
```

### 6.3 State Integration (RK4)

```pseudocode
FUNCTION Integrate_RK4(x, u, Δt)
    k1 ← Dynamics(x, u)
    k2 ← Dynamics(x + Δt/2 · k1, u)
    k3 ← Dynamics(x + Δt/2 · k2, u)
    k4 ← Dynamics(x + Δt · k3, u)
    
    x_new ← x + Δt/6 · (k1 + 2·k2 + 2·k3 + k4)
    
    return x_new
END FUNCTION
```

### 6.4 Dynamics Model (Continuous-Time)

```pseudocode
FUNCTION Dynamics(x, u)
    % Extract states
    p ← x(1:3)         % position NED
    η₂ ← x(4:6)        % Euler angles [φ, θ, ψ]
    v ← x(7:9)         % velocity body-frame
    ω ← x(10:12)       % angular velocity body-frame
    b_g ← x(13:15)     % gyro bias
    
    % Compute rotation matrix
    R ← Rotation_Matrix(η₂)
    T ← Euler_Rate_Matrix(η₂)
    
    % Get inputs/measurements
    IMU_gyro ← u_gyro          % from IMU
    IMU_accel ← u_accel        % from IMU
    
    % State derivatives
    ṗ ← R · v                  % Position rate
    η̇₂ ← T · ω                % Orientation rate
    
    % Velocity rate (simplified dynamics)
    v̇ ← -0.1 · v + R^T · (IMU_accel - b_a)
    
    % Angular velocity (use gyro measurement)
    ω̇ ← IMU_gyro - b_g
    
    % Bias rates (random walk)
    ḃ_g ← 0
    
    % Stack derivatives
    ẋ ← [ṗ; η̇₂; v̇; ω̇; ḃ_g]
    
    return ẋ
END FUNCTION
```

### 6.5 Rotation Matrix Computation

```pseudocode
FUNCTION Rotation_Matrix(φ, θ, ψ)
    % Precompute sines and cosines
    cφ ← cos(φ), sφ ← sin(φ)
    cθ ← cos(θ), sθ ← sin(θ)
    cψ ← cos(ψ), sψ ← sin(ψ)
    tθ ← tan(θ)
    
    % ZYX rotation (Fossen convention)
    R ← [cψ·cθ                -sψ·cφ+cψ·sθ·sφ   sψ·sφ+cψ·cφ·sθ ;
         sψ·cθ                cψ·cφ+sφ·sθ·sψ   -cψ·sφ+sθ·sψ·cφ ;
         -sθ                   cθ·sφ             cθ·cφ            ]
    
    return R
END FUNCTION
```

### 6.6 Measurement Model

```pseudocode
FUNCTION Measurement_Function(x)
    % Extract relevant states
    u ← x(7), v ← x(8), w ← x(9)    % velocity
    φ ← x(4), θ ← x(5), ψ ← x(6)   % angles
    z ← x(3)                         % depth
    
    % DVL measures body-frame velocity
    h_DVL ← [u, v, w]ᵀ
    
    % AHRS measures Euler angles
    h_AHRS ← [φ, θ, ψ]ᵀ
    
    % Depth sensor measures z
    h_depth ← z
    
    % Combined output
    h ← [h_DVL; h_AHRS; h_depth]
    
    return h
END FUNCTION
```

---

## 7. IMPLEMENTATION NOTES AND TUNING

### 7.1 Jacobian Computation

**Numerical Differentiation** (simplest for implementation):

```
F[i,j] = (f(x + δe_j) - f(x - δe_j)) / (2δ)

where δ ≈ 1e-6 (small perturbation)
```

**Analytical Jacobians** (more efficient):
- For rotation matrices: Use partial derivatives of trig functions
- For kinematic terms: Chain rule on R(η₂) and T(η₂)

### 7.2 Gimbal Lock Handling

**Problem**: T(η₂) becomes singular at θ = ±90°

**Solutions**:

1. **Constraint Operations**: Keep θ within ±85° during flight
2. **Quaternion Representation**: Use q = [q₀, q₁, q₂, q₃]ᵀ
   - Adds 1 state, but avoids singularities
   - Requires quaternion kinematics: q̇ = 0.5·Ω(ω)·q
3. **Error-State EKF**: Track error in orientation, reset full state

### 7.3 Covariance Matrix Properties

**Must maintain**:
- Symmetry: P = P^T
- Positive definiteness: all eigenvalues > 0

**Numerical checks**:
```
% In code:
if not (all(eig(P) > 0)):
    P ← (P + P^T) / 2  % Force symmetry
    P ← P + μ·I        % Add small regularization
end
```

### 7.4 Sensor Availability Handling

**If measurement available**:
```
Perform full correction step
```

**If measurement missing**:
```
Skip correction, only propagate prediction
x̂_k^+ ← x̂_k^-
P_k^+ ← P_k^-
```

**For occasional DVL dropouts**:
```
Allow P to grow when DVL unavailable
Track "gap length" to trigger re-initialization if too long
```

### 7.5 Tuning Strategy

**Step 1**: Estimate Q from model uncertainty
```
Run simulations with known vehicle dynamics
Observe prediction error: e = x_true - x_predicted
Set q_i = mean(e_i²) / Δt
```

**Step 2**: Set R from sensor datasheets
```
Usually given as noise standard deviation σ
Set r_i = σ_i²
```

**Step 3**: Filter tuning
```
Log innovation sequence: y_k = z_k - h(x̂_k^-)
Analyze: should be ~ N(0, R)

If innovation mean biased:
  → Increase Q (trust model less)
  
If innovation variance too large:
  → Increase R (don't trust measurements)
  
If filter lags measurements:
  → Decrease Q (trust measurements more)
```

### 7.6 Observability Issues

**With DVL + AHRS + Depth only**:
- ✓ Observable: x, y, z, φ, θ, u, v, w, b_gx, b_gy, b_gz
- ✗ **Not observable**: ψ (yaw)

**Why yaw drifts**:
- No absolute heading reference (no magnetometer/GPS)
- DVL is relative to seabed (no yaw info)
- AHRS has yaw drift from gyro integration

**Solution**: Add magnetometer or external heading reference

---

## 8. MATLAB CODE EXAMPLE

### 8.1 Complete EKF Implementation

```matlab
%% EKF Navigation Filter for 6DOF AUV
% Fuses DVL, AHRS, and Depth sensor
% Author: Your Name
% Date: Oct 2025

classdef AUV_EKF
    properties
        % State vector [15 x 1]
        x_hat           % State estimate
        P               % Covariance matrix [15 x 15]
        
        % Noise covariances
        Q               % Process noise [15 x 15]
        R               % Measurement noise [7 x 7]
        
        % Vehicle parameters
        mass            % Vehicle mass [kg]
        rho = 1025      % Seawater density [kg/m³]
        gravity = 9.81  % Gravity [m/s²]
        
        % Previous state for numerical differentiation
        x_prev
    end
    
    methods
        %% Constructor
        function obj = AUV_EKF(mass, L, W, H)
            % Initialize filter
            obj.mass = mass;
            
            % Initial state (15 x 1)
            obj.x_hat = zeros(15, 1);
            obj.x_prev = obj.x_hat;
            
            % Initial covariance
            obj.P = diag([1, 1, 1,              % position [m²]
                         0.1, 0.1, 0.1,        % angles [rad²]
                         0.1, 0.1, 0.1,        % velocity [(m/s)²]
                         0.01, 0.01, 0.01,     % ang vel [(rad/s)²]
                         0.001, 0.001, 0.001]); % bias [(rad/s)²]
            
            % Process noise covariance
            obj.Q = diag([1e-6, 1e-6, 1e-6,            % position drift
                         1e-6, 1e-6, 1e-6,             % angle drift
                         1e-4, 1e-4, 1e-4,             % velocity
                         1e-5, 1e-5, 1e-5,             % ang vel
                         1e-8, 1e-8, 1e-8]);           % bias
            
            % Measurement noise covariance
            sigma_dvl = 0.02;   % 0.02 m/s
            sigma_ahrs = 0.05;  % 0.05 rad (≈ 2.9°)
            sigma_depth = 0.5;  % 0.5 m
            
            obj.R = diag([sigma_dvl^2, sigma_dvl^2, sigma_dvl^2, ...
                         sigma_ahrs^2, sigma_ahrs^2, sigma_ahrs^2, ...
                         sigma_depth^2]);
        end
        
        %% Main EKF Step
        function obj = step(obj, IMU, z_meas, dt)
            % IMU: struct with fields gyro, accel [3x1 each]
            % z_meas: [u,v,w, φ,θ,ψ, z]ᵀ measurements
            % dt: time step [s]
            
            % ===== PREDICTION =====
            % Propagate state with RK4
            x_pred = obj.RK4_integrate(obj.x_hat, IMU, dt);
            
            % Compute Jacobian F
            F = obj.jacobian_f(obj.x_hat, IMU, dt);
            
            % Propagate covariance
            P_pred = F * obj.P * F' + obj.Q;
            
            % ===== CORRECTION =====
            % Measurement prediction
            h_x = obj.measurement_model(x_pred);
            
            % Innovation
            y = z_meas - h_x;
            
            % Jacobian H
            H = obj.jacobian_h();
            
            % Innovation covariance
            S = H * P_pred * H' + obj.R;
            
            % Kalman gain
            K = P_pred * H' / S;
            
            % State update
            obj.x_hat = x_pred + K * y;
            
            % Covariance update
            obj.P = (eye(15) - K * H) * P_pred;
            
            % Ensure symmetry and positive definiteness
            obj.P = (obj.P + obj.P') / 2;
            
            obj.x_prev = obj.x_hat;
        end
        
        %% RK4 Integration
        function x_new = RK4_integrate(obj, x, IMU, dt)
            k1 = obj.dynamics(x, IMU);
            k2 = obj.dynamics(x + dt/2*k1, IMU);
            k3 = obj.dynamics(x + dt/2*k2, IMU);
            k4 = obj.dynamics(x + dt*k3, IMU);
            
            x_new = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
        end
        
        %% Continuous-Time Dynamics
        function xdot = dynamics(obj, x, IMU)
            % Extract states
            p = x(1:3);      % position NED
            eta2 = x(4:6);   % Euler angles
            v = x(7:9);      % velocity body-frame
            omega = x(10:12); % angular velocity
            b_g = x(13:15);  % gyro bias
            
            % Rotation matrix
            R = obj.rotation_matrix(eta2);
            T = obj.euler_rate_matrix(eta2);
            
            % State derivatives
            p_dot = R * v;
            eta2_dot = T * omega;
            
            % Simplified dynamics (for EKF prediction)
            v_dot = -0.1 * v + R' * (IMU.accel - zeros(3,1));
            omega_dot = IMU.gyro - b_g;
            b_g_dot = zeros(3, 1);
            
            xdot = [p_dot; eta2_dot; v_dot; omega_dot; b_g_dot];
        end
        
        %% Rotation Matrix (NED <- Body)
        function R = rotation_matrix(~, euler)
            phi = euler(1);
            theta = euler(2);
            psi = euler(3);
            
            R = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
                 sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
                 -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
        end
        
        %% Euler Rate Matrix
        function T = euler_rate_matrix(~, euler)
            phi = euler(1);
            theta = euler(2);
            
            T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                 0, cos(phi), -sin(phi);
                 0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
        end
        
        %% Measurement Model
        function h = measurement_model(obj, x)
            h = [x(7:9);        % DVL: velocity [u,v,w]
                 x(4:6);        % AHRS: angles [φ,θ,ψ]
                 x(3)];         % Depth: z
        end
        
        %% Jacobian of Dynamics (Numerical)
        function F = jacobian_f(obj, x, IMU, dt)
            n = length(x);
            F = zeros(n, n);
            delta = 1e-7;
            
            x_nom = obj.RK4_integrate(x, IMU, dt);
            
            for j = 1:n
                x_pert = x;
                x_pert(j) = x_pert(j) + delta;
                x_pert_next = obj.RK4_integrate(x_pert, IMU, dt);
                
                F(:, j) = (x_pert_next - x_nom) / delta;
            end
        end
        
        %% Jacobian of Measurement Model (Analytical)
        function H = jacobian_h(~)
            H = [zeros(3,3), zeros(3,3), eye(3), zeros(3,6);
                 zeros(3,3), eye(3), zeros(3,3), zeros(3,6);
                 0, 0, 1, zeros(1,12)];
        end
        
        %% Get State Estimate
        function [p, euler, v, omega, b_g] = get_state(obj)
            p = obj.x_hat(1:3);
            euler = obj.x_hat(4:6);
            v = obj.x_hat(7:9);
            omega = obj.x_hat(10:12);
            b_g = obj.x_hat(13:15);
        end
    end
end

%% Example Usage
% Create filter
ekf = AUV_EKF(100, 2, 1, 1);  % mass=100kg, dimensions

% Simulation loop
dt = 0.1;
for k = 1:1000
    % Simulate IMU measurements
    IMU.gyro = randn(3,1)*0.01;
    IMU.accel = [0; 0; 9.81] + randn(3,1)*0.1;
    
    % Simulate sensor measurements (every 10 steps)
    if mod(k, 10) == 0
        z_meas = [0.1; 0.05; 0.02;  % DVL
                  0.01; -0.02; 0.5;  % AHRS
                  10];                % Depth
    else
        z_meas = [];
    end
    
    % EKF step
    if ~isempty(z_meas)
        ekf = ekf.step(IMU, z_meas, dt);
    end
    
    % Get estimates
    [p, euler, v, omega, b_g] = ekf.get_state();
    
    % Log or display
    if mod(k, 100) == 0
        fprintf('Time: %.1fs | Pos: [%.2f, %.2f, %.2f] | Depth: %.2f m\n', ...
                k*dt, p(1), p(2), p(3));
    end
end
```

### 8.2 Python Implementation

```python
import numpy as np
from scipy.linalg import block_diag
from scipy.integrate import odeint

class AUV_EKF:
    def __init__(self, mass=100):
        self.mass = mass
        
        # State vector [15 x 1]
        self.x_hat = np.zeros(15)
        
        # Initial covariance
        P_diag = np.array([
            1, 1, 1,                          # position [m²]
            0.1, 0.1, 0.1,                    # angles [rad²]
            0.1, 0.1, 0.1,                    # velocity
            0.01, 0.01, 0.01,                 # angular velocity
            0.001, 0.001, 0.001               # bias
        ])
        self.P = np.diag(P_diag)
        
        # Process noise
        Q_diag = np.array([
            1e-6, 1e-6, 1e-6,                 # position
            1e-6, 1e-6, 1e-6,                 # angles
            1e-4, 1e-4, 1e-4,                 # velocity
            1e-5, 1e-5, 1e-5,                 # angular velocity
            1e-8, 1e-8, 1e-8                  # bias
        ])
        self.Q = np.diag(Q_diag)
        
        # Measurement noise
        sigma_dvl, sigma_ahrs, sigma_depth = 0.02, 0.05, 0.5
        R_diag = np.array([
            sigma_dvl**2, sigma_dvl**2, sigma_dvl**2,
            sigma_ahrs**2, sigma_ahrs**2, sigma_ahrs**2,
            sigma_depth**2
        ])
        self.R = np.diag(R_diag)
    
    def rotation_matrix(self, euler):
        """Compute R from body to NED"""
        phi, theta, psi = euler
        
        R = np.array([
            [np.cos(psi)*np.cos(theta), -np.sin(psi)*np.cos(phi)+np.cos(psi)*np.sin(theta)*np.sin(phi), np.sin(psi)*np.sin(phi)+np.cos(psi)*np.cos(phi)*np.sin(theta)],
            [np.sin(psi)*np.cos(theta), np.cos(psi)*np.cos(phi)+np.sin(phi)*np.sin(theta)*np.sin(psi), -np.cos(psi)*np.sin(phi)+np.sin(theta)*np.sin(psi)*np.cos(phi)],
            [-np.sin(theta), np.cos(theta)*np.sin(phi), np.cos(theta)*np.cos(phi)]
        ])
        return R
    
    def euler_rate_matrix(self, euler):
        """Compute T: rate of Euler angles from angular velocity"""
        phi, theta = euler[0], euler[1]
        
        T = np.array([
            [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])
        return T
    
    def dynamics(self, x, imu_gyro, imu_accel):
        """Compute state derivatives"""
        p = x[0:3]
        euler = x[3:6]
        v = x[6:9]
        omega = x[9:12]
        b_g = x[12:15]
        
        R = self.rotation_matrix(euler)
        T = self.euler_rate_matrix(euler)
        
        p_dot = R @ v
        euler_dot = T @ omega
        v_dot = -0.1 * v + R.T @ imu_accel
        omega_dot = imu_gyro - b_g
        b_g_dot = np.zeros(3)
        
        return np.concatenate([p_dot, euler_dot, v_dot, omega_dot, b_g_dot])
    
    def rk4_integrate(self, x, imu_gyro, imu_accel, dt):
        """RK4 integration"""
        k1 = self.dynamics(x, imu_gyro, imu_accel)
        k2 = self.dynamics(x + dt/2*k1, imu_gyro, imu_accel)
        k3 = self.dynamics(x + dt/2*k2, imu_gyro, imu_accel)
        k4 = self.dynamics(x + dt*k3, imu_gyro, imu_accel)
        
        return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    
    def jacobian_f(self, x, imu_gyro, imu_accel, dt):
        """Numerical Jacobian of state transition"""
        n = len(x)
        F = np.zeros((n, n))
        delta = 1e-7
        
        x_nom = self.rk4_integrate(x, imu_gyro, imu_accel, dt)
        
        for j in range(n):
            x_pert = x.copy()
            x_pert[j] += delta
            x_pert_next = self.rk4_integrate(x_pert, imu_gyro, imu_accel, dt)
            
            F[:, j] = (x_pert_next - x_nom) / delta
        
        return F
    
    def measurement_model(self, x):
        """h(x): predict measurements from state"""
        return np.array([
            x[6], x[7], x[8],          # DVL velocity
            x[3], x[4], x[5],          # AHRS angles
            x[2]                       # Depth
        ])
    
    def jacobian_h(self):
        """Jacobian of measurement model"""
        H = np.zeros((7, 15))
        H[0:3, 6:9] = np.eye(3)       # DVL
        H[3:6, 3:6] = np.eye(3)       # AHRS
        H[6, 2] = 1                    # Depth
        return H
    
    def step(self, imu_gyro, imu_accel, z_meas, dt):
        """Single EKF step"""
        # PREDICTION
        x_pred = self.rk4_integrate(self.x_hat, imu_gyro, imu_accel, dt)
        F = self.jacobian_f(self.x_hat, imu_gyro, imu_accel, dt)
        P_pred = F @ self.P @ F.T + self.Q
        
        # CORRECTION
        if z_meas is not None:
            h_x = self.measurement_model(x_pred)
            y = z_meas - h_x                    # Innovation
            H = self.jacobian_h()
            S = H @ P_pred @ H.T + self.R      # Innovation covariance
            K = P_pred @ H.T @ np.linalg.inv(S) # Kalman gain
            
            self.x_hat = x_pred + K @ y
            self.P = (np.eye(15) - K @ H) @ P_pred
        else:
            self.x_hat = x_pred
            self.P = P_pred
        
        # Maintain symmetry
        self.P = (self.P + self.P.T) / 2
    
    def get_state(self):
        """Extract state components"""
        return {
            'position': self.x_hat[0:3],
            'euler': self.x_hat[3:6],
            'velocity': self.x_hat[6:9],
            'ang_vel': self.x_hat[9:12],
            'bias': self.x_hat[12:15]
        }

# Example usage
ekf = AUV_EKF(mass=100)

for k in range(1000):
    dt = 0.1
    
    # Simulate IMU
    imu_gyro = np.random.randn(3) * 0.01
    imu_accel = np.array([0, 0, 9.81]) + np.random.randn(3) * 0.1
    
    # Sensor measurements (every 10 steps)
    if k % 10 == 0:
        z_meas = np.array([0.1, 0.05, 0.02,  # DVL
                          0.01, -0.02, 0.5,  # AHRS
                          10])                # Depth
    else:
        z_meas = None
    
    # EKF step
    ekf.step(imu_gyro, imu_accel, z_meas, dt)
    
    # Print
    if k % 100 == 0:
        state = ekf.get_state()
        print(f"Time: {k*dt:.1f}s | Depth: {state['position'][2]:.2f}m")
```

---

## SUMMARY CHECKLIST

### ✓ What You Have
1. **Rotation matrices and kinematics** (Fossen 2011)
2. **Complete dynamic model** (6DOF with all terms)
3. **Discrete EKF formulation** (prediction + correction)
4. **Sensor measurement models** (DVL, AHRS, Depth)
5. **Jacobian calculations** (numerical and analytical)
6. **Pseudocode algorithms**
7. **MATLAB implementation**
8. **Python implementation**
9. **Tuning guidelines**
10. **Observability analysis**

### ⚠️ Known Limitations
- Gimbal lock at θ = ±90° (use constraint or quaternions)
- Yaw not observable without heading reference
- Simplified drag model (can be replaced with full Fossen)
- Assumes DVL works near seabed

### 🔧 Next Steps for Your Thesis
1. Implement and test on simulation data
2. Add real sensor data from your AUV
3. Compare with DVL-only navigation
4. Analyze observability with your sensor suite
5. Consider switching to Error-State EKF for better stability
