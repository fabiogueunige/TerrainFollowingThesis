# 6-DOF AUV Terrain Following System

**Autonomous Underwater Vehicle (AUV) terrain-following control using Extended Kalman Filter (EKF) and adaptive state machine.**

![Status](https://img.shields.io/badge/status-active-success.svg)
![MATLAB](https://img.shields.io/badge/MATLAB-R2020b+-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)

---

## 📋 Overview

This project implements a complete simulation environment for a 6-DOF underwater vehicle performing **autonomous terrain following** at constant altitude. The system uses:

- **Extended Kalman Filter (EKF)** for state estimation (altitude, terrain angles)
- **4 Single-Beam Echo Sounders (SBES)** for terrain profiling
- **PID control with anti-windup** for trajectory tracking
- **Finite State Machine** for robust mission management with sensor failure recovery
- **Dynamic terrain generation** with realistic angle variations

**Vehicle Model:** BlueROV2-inspired dynamics (11.5 kg, 6-DOF)  
**Target Application:** Underwater mapping, inspection, terrain-relative navigation

---

## 🎯 Key Features

✅ **Robust State Estimation**: EKF fuses 4 SBES measurements for altitude and terrain orientation  
✅ **Sensor Failure Recovery**: Automatic detection and recovery from 1-4 sensor losses  
✅ **Adaptive Terrain Generation**: Dynamic plane generation ahead of robot (circular buffer)  
✅ **Real-Time Visualization**: 3D plots of robot, sensors, terrain, and trajectory  
✅ **Comprehensive Diagnostics**: Normal parallelism analysis, failure statistics, state tracking  
✅ **Modular Design**: Clean separation of control, estimation, sensors, and state machine

---

## 📂 Project Structure

```
matlab_3D/
├── main_6DOF_3D.m          # Main simulation loop (40s, 1kHz)
├── controller/             # PID control with anti-windup
│   ├── gainComputation.m   # Compute PID gains from robot dynamics
│   ├── input_control.m     # Control logic (error, PID, frame transforms)
│   └── tau0_values.m       # Equilibrium forces for linearization
├── ekf_filter/             # Extended Kalman Filter
│   ├── f.m                 # State prediction: x_{k+1} = f(x_k, u_k)
│   ├── h.m                 # Measurement model: z_k = h(x_k)
│   ├── jacobian_f.m        # ∂f/∂x (covariance propagation)
│   └── jacobian_h.m        # ∂h/∂x (Kalman gain)
├── kf_filter/              # (Placeholder for future position KF)
|   ├── kf_estimation.m     # position estimation
├── math_function/          # Numerical utilities
│   ├── derivator.m         # Discrete derivative (Tustin)
│   ├── integrator.m        # Discrete integral (trapezoidal)
│   ├── intersection_check.m# Ray-plane intersection validator
│   ├── plane_computation.m # Terrain normal from sensor points
│   ├── reference_correction.m # Ensure normal points downward
│   └── vector_normalization.m
├── model/                  # Robot dynamics
│   ├── dynamic_model.m     # 6-DOF dynamics (linearized BlueROV2)
│   └── kinematic_model.m   # 6-DOF kinematics
├── noise/                  # Noise configuration
│   └── noise_setup.m       # Process (Q) & measurement (R) covariances
├── rotation/               # 3D rotation utilities
│   ├── rotx.m, roty.m, rotz.m       # Rotation matrices
│   └── d_rotx.m, d_roty.m, d_rotz.m # Derivatives for Jacobians
├── sensors/                # Sensor suite
│   ├── SBES_definition.m   # Define 4 SBES beam directions (±22.5°)
│   ├── SBES_measurament.m  # Ray-casting for terrain measurements
│   ├── AHRS_measurement.m  # Attitude measurement (roll, pitch, yaw)
│   ├── DVL_measurament.m   # Velocity & position (Doppler log)
│   └── measurament.m       # (Legacy, not used)
├── state_machine/          # Mission state machine
│   ├── state_machine.m     # State transitions (8 states)
│   ├── goal_def.m          # Goal velocities/angles per state
│   └── goal_controller.m   # Command flag updates (failure detection)
├── visualization/          # Plotting & debugging
│   ├── m_visualization.m   # 3D plot (robot, sensors, terrain)
│   ├── plot_results.m      # End-of-sim analysis (states, angles, stats)
│   └── printDebug.m        # Conditional debug printing
├── world_generator/        # Terrain generation
│   ├── terrain_init.m      # Initialize terrain planes
│   └── terrain_generator.m # Dynamic plane generation (circular buffer)
├── data_management/        # **NEW: Data saving & analysis**
│   ├── save_simulation_data.m    # Save run to timestamped folder
│   ├── collect_simulation_data.m # Collect all variables
│   ├── load_simulation_data.m    # Load saved runs
│   ├── analyze_statistics.m      # Statistical analysis across runs
│   ├── example_batch_analysis.m  # Example analysis script
│   └── README.md                 # Data management documentation
└── doc/                    # Documentation
    ├── ARCHITECTURE.md     # System overview & data flow
    ├── EKF_ALGORITHM.md    # EKF mathematical details
    ├── SENSORS.md          # SBES, AHRS, DVL specifications
    ├── CONTROL_SYSTEM.md   # PID tuning & dynamics
    └── STATE_MACHINE_IMPROVEMENTS.md # FSM logic & recovery
```

---

## 🚀 Quick Start

### Prerequisites

- **MATLAB** R2020b or later (no toolboxes required)
- **OS**: Windows, macOS, or Linux

### Installation

```bash
git clone https://github.com/fabiogueunige/TFThesis.git
cd TFThesis/TF_6DOF/matlab_3D
```

### Run Simulation

```matlab
% Open MATLAB in project root
main_6DOF_3D
```

**Expected Output:**

- Console: State transitions, diagnostics (~40,000 iterations)
- Figures (auto-generated):
  - State tracking (altitude, alpha, beta)
  - Robot angles (roll, pitch, yaw)
  - Control inputs (surge, sway, heave, p, q, r)
  - Normal parallelism analysis
  - 3D trajectory

**Duration:** ~10 seconds on modern laptop (80x real-time)

---

## 💾 Data Management & Statistical Analysis

### Save Simulation Data

After running a simulation, you'll be prompted to save data:

```
Save data? (Y/N): Y
1. Auto-generate run name (run_YYYYMMDD_HHMMSS)
2. Specify custom run name
Choose option (1 or 2): 1
```

**Saved Data Structure:**
```
results/
  run_20251023_143022/
    ├── ekf_states.mat       # States (true, estimated, predicted)
    ├── ekf_covariance.mat   # Innovation, covariance matrices
    ├── sensor_data.mat      # SBES, AHRS measurements
    ├── control_data.mat     # PID outputs, velocities
    ├── trajectory.mat       # Position, rotation matrices
    ├── parameters.mat       # All simulation parameters
    └── metadata.txt         # Human-readable summary
```

### Load and Analyze Data

```matlab
% Load specific run
sim_data = load_simulation_data('run_20251023_143022');

% Load with interactive selection
sim_data = load_simulation_data('');

% Analyze all runs statistically
stats = analyze_statistics();

% Run batch analysis with plots
example_batch_analysis;
```

**See:** `data_management/README.md` for complete documentation

---

## 📊 System Configuration

### Key Parameters (Editable in `main_6DOF_3D.m`)

```matlab
% Simulation
Ts = 0.001;        % Sampling time [s] (1 kHz)
Tf = 40;           % Total time [s]
DEBUG = false;     % Set true for verbose console output

% Target
h_ref(:) = 3;      % Reference altitude [m]

% Terrain
max_planes = 500;  % Circular buffer size
step_length = 4;   % Distance between planes [m]
angle_range = [-pi/5, pi/5]; % ±36° terrain slopes

% Sensors
num_s = 4;         % Number of SBES (fixed: front, rear, left, right)

% EKF
P0 = diag([1, 0.08, 2]) * 1.1; % Initial covariance
Q = diag([0.01, 0.0001, 0.00025]); % Process noise
R = diag([0.031, 0.034, 0.031, 0.034]); % Measurement noise
```

### Robot Dynamics (BlueROV2)

```matlab
m = 11.5 kg              % Total mass
I = diag([0.21, 0.245, 0.245]) kg⋅m²  % Inertia
B = 111.0 N              % Buoyancy
damping: linear + quadratic (empirical)
```

---

## 🎛️ State Machine

### States

1. **Idle**: Waiting for start command
2. **TargetAltitude**: Descend to h_ref (3m)
3. **ContactSearch**: Search for terrain contact with all sensors
4. **Following**: Main terrain-following mode
5. **MovePitch**: Recover from pitch sensor loss (sensors 1-2)
6. **MoveRoll**: Recover from roll sensor loss (sensors 3-4)
7. **RecoveryAltitude**: Altitude adjustment for diagonal sensor loss
8. **Emergency**: Safety mode (altitude < 0.7m)
9. **EndSimulation**: Clean termination

### Recovery Logic

| Failure Pattern | Action | Timeout |
|-----------------|--------|---------|
| 1 sensor lost | Tolerated (EKF uses 3 sensors) | N/A |
| Sensors 1-2 lost | MovePitch: pitch ±15°, slow motion | 5s → RecoveryAltitude |
| Sensors 3-4 lost | MoveRoll: roll ±15°, slow motion | 5s → RecoveryAltitude |
| Diagonal (1-4 or 2-3) | RecoveryAltitude: h_ref - 1m | 5s → Reset |
| All 4 sensors lost | Reset: return to TargetAltitude | N/A |

**Grace Period:** 0.5s (avoids reacting to transient glitches)

---

## 📈 Performance Metrics

### Typical Results (Flat + Rolling Terrain)

| Metric | Value | Unit |
|--------|-------|------|
| **Altitude Error (steady-state)** | < 0.5 | m |
| **Angle Error (steady-state)** | < 1 | ° |
| **Sensor Contact Rate** | > 95 | % |
| **Normal Parallelism** | < 3 | ° |
| **Iteration Time** | 0.08 | ms |
| **Real-Time Factor** | 80× | - |

**Tested Terrain:**
- Flat (α=0°, β=0°): Perfect tracking
- Rolling (±30° slopes): Robust following
- Steep features (> 40°): Triggers recovery

---

## 🔧 Customization

### Change Altitude Target

```matlab
% In main_6DOF_3D.m (line ~75)
h_ref(:) = 5;  % Change from 3m to 5m
```

### Enable Debug Output

```matlab
% In main_6DOF_3D.m (line ~29)
DEBUG = true;
```

### Adjust PID Gains

```matlab
% In controller/gainComputation.m (line ~10)
wn = 0.5;   % Increase from 0.4 for faster response
damp = 0.7; % Increase from 0.6 for less overshoot
```

### Add Noise to Sensors

```matlab
% In main_6DOF_3D.m (line ~192)
% Uncomment DVL noise:
v_dvl = mvnrnd(zeros(3,1), R_dvl)';
dvl_speed_w = dvl_speed_w + v_dvl;
```

---

## 📚 Documentation

Comprehensive technical documentation in `/doc`:

- **[ARCHITECTURE.md](doc/ARCHITECTURE.md)**: System overview, module breakdown, data flow
- **[EKF_ALGORITHM.md](doc/EKF_ALGORITHM.md)**: EKF formulation, Jacobians, tuning
- **[SENSORS.md](doc/SENSORS.md)**: SBES, AHRS, DVL specifications & calibration
- **[CONTROL_SYSTEM.md](doc/CONTROL_SYSTEM.md)**: PID tuning, dynamics, anti-windup
- **[STATE_MACHINE_IMPROVEMENTS.md](doc/STATE_MACHINE_IMPROVEMENTS.md)**: FSM logic, recovery strategies

---

## 🐛 Troubleshooting

### Issue: "NaN in Kalman Gain"

**Cause**: Singular innovation covariance (sensors parallel to terrain)  
**Solution**: Check SBES geometry, increase R noise, verify terrain normal

### Issue: Robot Oscillates

**Cause**: PID gains too high  
**Solution**: Reduce `wn` from 0.4 to 0.3 in `gainComputation.m`

### Issue: Sensors Always Fail

**Cause**: Terrain generation issue or robot too far from terrain  
**Solution**: Check `h_ref` is reasonable (1-10m), verify `terrain_init`

### Issue: Slow Simulation

**Cause**: Frequent visualization updates  
**Solution**: Change `mod(ite, 2000)` to `mod(ite, 10000)` in `SBES_measurament.m` (line 108)

---

## 🔬 Research & Extensions

### Completed Features
✅ EKF for altitude & terrain angles  
✅ 4-sensor SBES with failure recovery  
✅ Dynamic terrain generation (circular buffer)  
✅ State machine with 8 states  
✅ Real-time 3D visualization  

### Future Work
🔲 Position Kalman Filter (currently uses perfect DVL)  
🔲 Non-linear dynamics (full 6-DOF model commented out)  
🔲 Obstacle avoidance (static/dynamic)  
🔲 Multi-AUV coordination  
🔲 ROS/Gazebo integration  
🔲 Real sensor data (sonar logs, IMU)  
🔲 Machine learning for terrain classification  

---

## 📄 Citation

If you use this code in your research, please cite:

```bibtex
@mastersthesis{guelfi2025terrain,
  author  = {Fabio Guelfi},
  title   = {Advanced AUV Motion Control and Terrain Following for Automatic Seabed Inspection},
  school  = {University of Genoa},
  year    = {2025},
  type    = {Master's Thesis},
  url     = {https://github.com/fabiogueunige/TFThesis}
}
```

---

## 📧 Contact

**Author:** Fabio Guelfi 
**Email:** [fabio.guelfi@libero.it]
**GitHub:** [@fabiogueunige](https://github.com/fabiogueunige)  
**Project:** Terrain Following Thesis (TFThesis)  

---

## 📝 License

This project is licensed under the **MIT License** - see [LICENSE](LICENSE) file for details.

---

**Last Updated:** 2025-01-23  
**Version:** 1.0.0  
**Status:** Active Development
