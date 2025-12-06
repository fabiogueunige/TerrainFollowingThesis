# Sensor Suite - Technical Documentation

## Overview

The AUV uses three complementary sensor types for terrain-following navigation:
1. **SBES (Single-Beam Echo Sounders)**: 4 acoustic rangefinders for terrain profiling
2. **AHRS (Attitude Heading Reference System)**: Inertial measurement for orientation
3. **DVL (Doppler Velocity Log)**: Velocity and position estimation

---

## 1. SBES (Single-Beam Echo Sounders)

**File**: `sensors/SBES_measurament.m`, `sensors/SBES_definition.m`

### Configuration

Four downward-looking acoustic beams arranged in a cross pattern:

| Sensor | Direction | Angle | Purpose |
|--------|-----------|-------|---------|
| 1 (Rear) | Pitch down | -22.5° | Rear terrain profile |
| 2 (Front) | Pitch up | +22.5° | Forward terrain profile |
| 3 (Left) | Roll left | +22.5° | Left terrain profile |
| 4 (Right) | Roll right | -22.5° | Right terrain profile |

**Geometry:**
```
          2 (Front)
            ↗ 
            │
  3 (Left) ─┼─ 4 (Right)
            │
            ↙
          1 (Rear)
```

### Beam Definition (Robot Frame)

```matlab
Gamma  = -pi/8;  % -22.5° (rear)
Lambda = +pi/8;  % +22.5° (front)
Eta    = +pi/8;  % +22.5° (left)
Zeta   = -pi/8;  % -22.5° (right)

r_s(:,1) = [sin(Gamma), 0, cos(Gamma)]'   % Rear
r_s(:,2) = [sin(Lambda), 0, cos(Lambda)]' % Front
r_s(:,3) = [0, -sin(Eta), cos(Eta)]'      % Left
r_s(:,4) = [0, -sin(Zeta), cos(Zeta)]'    % Right
```

**Transformed to world frame:**
```matlab
s(:,j) = wRr * r_s(:,j)  % Robot rotation matrix applied
```

---

### Ray-Casting Algorithm

For each sensor j, find intersection with terrain planes:

```matlab
for ii = 1:max_planes
    % Check if ray intersects plane
    ray_plane_dot = dot(s(:,j), plane(ii).n_w)
    
    if abs(ray_plane_dot) > tolerance
        % Compute intersection parameter t
        t = -dot((p_robot - plane(ii).point_w), plane(ii).n_w) / ray_plane_dot
        
        if t > 0
            % Intersection point
            p_int = p_robot + t * s(:,j)
            
            % Check if within plane segment
            if intersection_check(plane(ii+1).point_w, plane(ii).point_w, p_int, plane(ii).dir_w)
                if t < t_star(j)
                    t_star(j) = t  % Keep closest intersection
                    plane_contact_idx(j) = ii
                end
            end
        end
    end
end

y(j) = t_star(j)  % Measurement = range [m]
```

**Key Features:**
- **Circular buffer search**: Scans all terrain planes efficiently
- **Closest intersection**: Multiple planes may intersect, keep nearest
- **Boundary check**: Ensure intersection within plane segment
- **Failure detection**: If no valid intersection, mark sensor failed

---

### Measurement Model

From ray-casting, measurement vector:
```
z = [y1, y2, y3, y4]'
```

**Physical interpretation:**
- y_j: Range from robot to terrain along sensor j direction [m]

**Noise model:**
```matlab
v_sbes ~ N(0, R_sbes)
z_meas = z_true + v_sbes

R_sbes = diag([0.177², 0.185², 0.177², 0.185²])
```

**Noise characteristics:**
- Acoustic scattering: ~5% range error
- Resolution: ~1 cm (limited by sampling rate)
- Front/rear sensors slightly noisier due to robot motion

---

### Failure Modes

**Cause**: Terrain too steep, sensor parallel to plane, out of range

**Detection:**
```matlab
if isinf(t_star(j)) || t_star(j) <= 0
    command.contact(j) = false
    command.sensor_fail = command.sensor_fail + 1
    Rm(:,:) = Rm(:,:) * 150  % Inflate measurement noise
end
```

**Impact:**
- 1 sensor lost: Tolerated, EKF uses remaining 3
- 2 sensors lost (same axis): Triggers recovery maneuver
- 4 sensors lost: Emergency reset

**Statistics** (from typical run):
- Failure rate: < 2% of samples
- Duration: Usually transient (< 0.5s)

---

### Plane Estimation from SBES

**File**: `math_function/plane_computation.m`

From 4 intersection points, estimate terrain normal:

```matlab
% Use first 3 valid points
p1 = p_int(:,1)
p2 = p_int(:,2)
p3 = p_int(:,3)

% Two vectors in plane
v1 = p2 - p1
v2 = p3 - p1

% Normal (cross product)
n_mes = cross(v1, v2)
n_mes = n_mes / norm(n_mes)
```

**Requirements:**
- At least 3 valid sensors (4th redundant)
- Points not collinear
- Normal points downward (checked by `reference_correction`)

---

## 2. AHRS (Attitude Heading Reference System)

**File**: `sensors/AHRS_measurement.m`

### Measured Variables

```
roll  (φ):   Rotation around X-axis (forward)
pitch (θ):   Rotation around Y-axis (lateral)
yaw   (ψ):   Rotation around Z-axis (vertical)
```

### Integration from Angular Velocities

**Linearized model** (small angles):
```matlab
φ_{k+1} = φ_k + p_k * Ts
θ_{k+1} = θ_k + q_k * Ts
ψ_{k+1} = ψ_k + r_k * Ts
```

where (p, q, r) are angular velocities from gyroscope [rad/s].

**Rotation matrix:**
```matlab
wRr = rotz(ψ) * roty(θ) * rotx(φ)
```

### Noise Model

```matlab
v_ahrs ~ N(0, R_ahrs)
angles_meas = angles_true + v_ahrs

R_ahrs = diag([deg2rad(0.5)², deg2rad(0.5)², deg2rad(0.08)²])
```

**Interpretation:**
- Roll/pitch: 0.5° std dev (typical MEMS IMU)
- Yaw: 0.08° std dev (magnetometer-aided)

### Usage in System

**Not estimated by EKF** (assumed measured directly):
- `rob_rot` used for coordinate transformations
- `wRr` computed from measured angles
- Provides robot orientation in world frame

**Note**: In real implementation, attitude estimation would use its own filter (e.g., Madgwick, complementary filter). Here simplified as direct integration.

---

## 3. DVL (Doppler Velocity Log)

**File**: `sensors/DVL_measurament.m`

### Measured Variables

```
velocity:  [u, v, w]' in robot frame [m/s]
```

### Position Integration

**Perfect integration** (no drift in simulation):
```matlab
% Velocity in world frame
w_speed = wRr * u(SURGE:HEAVE)

% Position update (Euler integration)
p_robot_{k+1} = p_robot_k + w_speed * Ts
```

**In reality**: DVL has drift, would require GPS/USBL for position resets.

### Noise Model

**Currently**: No noise added (perfect measurement)

**Realistic model** (commented out):
```matlab
v_dvl ~ N(0, σ_dvl²)
σ_dvl ≈ 0.01 m/s  % 1% velocity error
```

### Usage in System

- **Terrain generation**: Velocity direction determines new plane orientation
- **Control**: Not directly used (altitude controlled via EKF estimate)
- **Visualization**: Robot trajectory plotted

---

## Sensor Fusion Strategy

### Complementary Roles

| Sensor | Measures | Used For | Update Rate |
|--------|----------|----------|-------------|
| SBES | Terrain range | EKF: altitude, angles | 1 kHz |
| AHRS | Orientation | Coordinate transforms | 1 kHz |
| DVL | Velocity, position | Terrain generation, traj. | 1 kHz |

**No sensor redundancy for position** → Weakness in real deployment

**SBES redundancy**: 4 sensors → robust to 1-2 failures

---

## Coordinate Transformations

### Robot → World

```matlab
w_speed = wRr * r_speed
```

### World → Terrain

```matlab
wRt = rotz(0) * roty(β) * rotx(α) * rotx(π)
t_speed = wRt' * w_speed
```

### Sensor Direction (Robot → World)

```matlab
s_world(:,j) = wRr * r_s(:,j)
```

---

## Sensor Visualization

**File**: `visualization/m_visualization.m`

**Generated every 2000 iterations** (or at step 30):

- Robot box (oriented with wRr)
- Robot axes (X_r, Y_r, Z_r)
- Sensor rays (only if `contact(j) == true`)
- Intersection points (marked with 'x')
- Terrain patches (centered at intersections)
- Altitude projection (dashed line to terrain)

**Example:**
```
   Y_r (green)
    ↑
    │   ┌─────┐ Robot box (blue)
    │   │  ×  │ (center of mass)
    │   └─────┘
    │ ↙ ↓ ↘ (sensor rays)
    │ * * *   (intersections)
────┴────────── Terrain
```

---

## Real-World Considerations

### SBES Limitations

1. **Beam width**: Actual SBES has ~10° cone, not infinitesimal ray
2. **Multi-path**: Reflections from uneven terrain
3. **Acoustic attenuation**: Range limited to ~100 m
4. **Update rate**: Typically 5-10 Hz, not 1 kHz

**Simulation simplifications:**
- Infinitesimal ray (perfect directionality)
- Unlimited range
- Instantaneous update

### AHRS Drift

- **Gyro bias**: Accumulates over time (~0.01 °/s)
- **Magnetometer distortion**: Near metallic structures
- **Accelerometer noise**: During maneuvering

**Mitigation** (not in simulation):
- Kalman filter for attitude estimation
- Zero-velocity updates (ZUPTs) when stationary
- External heading reference (GPS compass)

### DVL Dead-Reckoning Error

- **Current drift**: Unobservable with bottom-lock DVL
- **Altitude dependence**: DVL accuracy degrades > 30 m altitude
- **Beam geometry**: 4-beam Janus configuration

**Mitigation** (not in simulation):
- USBL (Ultra-Short Baseline) for absolute positioning
- Particle filter for current estimation
- Integrate with terrain-based localization

---

## Calibration Parameters

### SBES

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| Beam angle | ±22.5° | deg | Design choice |
| Noise std | 0.18 | m | Empirical tuning |
| Max range | ∞ | m | Simplified |
| Update rate | 1000 | Hz | Simulation |

### AHRS

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| Roll/pitch noise | 0.5 | deg | MEMS IMU spec |
| Yaw noise | 0.08 | deg | Magnetometer-aided |
| Sampling rate | 1000 | Hz | Standard IMU |

### DVL

| Parameter | Value | Unit | Source |
|-----------|-------|------|--------|
| Velocity noise | 0 | m/s | Ideal (no noise) |
| Position drift | 0 | m | Perfect integration |
| Altitude range | ∞ | m | Simplified |

---

## Diagnostics & Debugging

### Check Sensor Health

**Command flags** (updated in `goal_controller.m`):
```matlab
command.contact(1:4)            % Boolean: sensor has valid contact
command.sensor_fail             % Count: number of failed sensors
command.pitch_sensors_lost      % Boolean: sensors 1-2 both lost
command.roll_sensors_lost       % Boolean: sensors 3-4 both lost
command.diagonal_sensors_lost   % Boolean: diagonal pair lost
command.sensor_fail_persistent  % Boolean: failure > 0.5s
```

### Print Debug Info

**Enable in main:**
```matlab
global DEBUG;
DEBUG = true;
```

**Output example:**
```
SBES Measurement:
h real: 2.984 | y1: 3.021 | y2: 3.015 | y3: 3.008 | y4: 3.013
DVL Measurement:
Point x: 12.45 | y: -3.21 | z: 12.98
AHRS Measurement:
phi: 5.23° | theta: -2.15° | yaw: 0.05°
```

### Visualize Sensor Geometry

**Plot sensor rays** in real-time:
```matlab
if mod(ite, 2000) == 0
    m_visualization(p_robot, gen_point, n_new, num_s, p_int, wRr, ite, planes, plane_contact_idx, command.contact);
end
```

**Toggle**: Change `mod(ite, 2000)` to `mod(ite, 100)` for more frequent updates (slower).

---

## Failure Recovery Examples

### Scenario 1: Pitch Sensors Lost (1-2)

**Cause**: Robot pitched up, sensors point above terrain

**Detection:**
```matlab
command.pitch_sensors_lost = ~contact(1) && ~contact(2)
```

**Recovery**: `MovePitch` state
- Pitch down slightly (+15° to +22.5°)
- Slow forward motion (0.02 m/s)
- Timeout: 5s → escalate to `RecoveryAltitude`

### Scenario 2: Roll Sensors Lost (3-4)

**Cause**: Robot rolled, sensors point off terrain edge

**Detection:**
```matlab
command.roll_sensors_lost = ~contact(3) && ~contact(4)
```

**Recovery**: `MoveRoll` state
- Roll adjustment
- Slow lateral motion
- Timeout: 5s → escalate

### Scenario 3: Diagonal Loss (1-4 or 2-3)

**Cause**: Terrain too steep, asymmetric loss

**Detection:**
```matlab
command.diagonal_sensors_lost = (~contact(1) && ~contact(4)) || (~contact(2) && ~contact(3))
```

**Recovery**: `RecoveryAltitude` state
- Reduce target altitude (-1m, closer to terrain)
- Ultra-slow motion (0.02 m/s)
- Timeout: 5s → `Reset`

---

## Performance Metrics

### SBES Contact Statistics

**Computed in**: `plot_results.m`

```matlab
valid_contacts = sum(~isnan(aa12)) / N * 100
fprintf('Valid SBES contacts: %.1f%%\n', valid_contacts);
```

**Typical values:**
- Flat terrain: > 98% contact
- Rolling terrain (±30°): > 90% contact
- Steep features: > 85% contact

### Normal Parallelism

**Metric**: Angle between n_est, n_mes, z_robot

```matlab
angle_est_mes = acos(abs(dot(n_est, n_mes)))
```

**Threshold**: < 5° → well-aligned

**Interpretation:**
- Small angle → EKF estimate matches sensor data
- Large angle → Model mismatch or sensor noise

---

## Future Enhancements

1. **Realistic SBES model**: Beam width, multi-path, attenuation
2. **AHRS filter**: Madgwick or complementary filter for attitude
3. **DVL noise**: Add realistic drift and current estimation
4. **Sensor fusion EKF**: Integrate all sensors in single filter
5. **Obstacle detection**: Use SBES for collision avoidance

---

**Related Files:**
- `sensors/SBES_measurament.m`
- `sensors/SBES_definition.m`
- `sensors/AHRS_measurement.m`
- `sensors/DVL_measurament.m`
- `math_function/intersection_check.m`
- `math_function/plane_computation.m`

---

**Author**: Fabio Guelfi
**Last Updated**: 2025-01-23
