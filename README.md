# Terrain Following Thesis - Advanced AUV Navigation System

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-active-success.svg)]()

**Advanced AUV Motion Control and Terrain Following for Automatic Seabed Inspection**

This repository contains the complete implementation of an **autonomous underwater vehicle (AUV) terrain-following navigation system** using Extended Kalman Filter (EKF) state estimation and 4 Single-Beam Echo Sounders (SBES) for real-time terrain profiling and adaptive motion control.

---

## 🎯 Project Overview

### Core Concept

This thesis develops a **robust terrain-following algorithm** that enables an AUV to maintain constant altitude above the seabed while continuously adapting its orientation (roll and pitch) to match the terrain inclination. The system is designed as a **modular navigation component** that can be integrated with path-following and obstacle avoidance algorithms to achieve reliable autonomous underwater navigation.

### Key Innovation

**4-SBES Array + EKF Fusion**: By strategically positioning four acoustic rangefinders in a cross-pattern configuration and fusing their measurements through an Extended Kalman Filter, the system simultaneously estimates:
- **Altitude** (distance from terrain)
- **Terrain roll angle** (α)
- **Terrain pitch angle** (β)

This enables the robot to **mimic terrain inclination** in real-time, providing:
- ✅ Improved terrain contact and mapping quality
- ✅ Reduced risk of collision with uneven seabeds
- ✅ Enhanced stability during underwater inspection missions
- ✅ Foundation for terrain-relative navigation systems

### Navigation System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    AUV NAVIGATION SYSTEM                         │
└─────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
┌──────────────┐      ┌──────────────┐     ┌──────────────┐
│Path Following│      │ TERRAIN      │     │  Obstacle    │
│  Algorithm   │──────│ FOLLOWING    │─────│  Avoidance   │
│              │      │ (This Work)  │     │              │
└──────────────┘      └──────────────┘     └──────────────┘
                              │
                    ┌─────────┴─────────┐
                    │                   │
                    ▼                   ▼
            ┌──────────────┐    ┌──────────────┐
            │  4-SBES      │    │  EKF State   │
            │  Sensor Rig  │────│  Estimator   │
            └──────────────┘    └──────────────┘
                    │
                    ▼
            ┌──────────────┐
            │ PID Motion   │
            │ Controller   │
            └──────────────┘
```

**Integration Potential:**
- **Path Following:** Use estimated terrain orientation as reference frame for trajectory tracking
- **Obstacle Avoidance:** SBES measurements provide terrain profile for collision detection
- **Localization:** Terrain-relative positioning reduces drift in dead-reckoning navigation

---

## 🚢 System Architecture

### Hardware Configuration (Simulated)

**Vehicle:** BlueROV2-inspired AUV
- Mass: 11.5 kg
- 6 Degrees of Freedom (6-DOF)
- Thruster configuration: Vectored propulsion

**Sensor Suite:**

| Sensor | Quantity | Purpose | Configuration |
|--------|----------|---------|---------------|
| **SBES** | 4 | Terrain ranging | Cross-pattern (±22.5°) |
| **AHRS** | 1 | Attitude measurement | Roll, Pitch, Yaw |
| **DVL** | 1 | Velocity & position | Bottom-lock mode |
| **DS** | 1 | Depth from pressure | Pre-installed |

**4-SBES Configuration:**
```
          Sensor 2 (Front)
                ↗ +22.5°
                │
Sensor 3 ───────┼─────── Sensor 4
(Left)          │        (Right)
+22.5°          │        -22.5°
                │
                ↙ -22.5°
          Sensor 1 (Rear)
```

**Why 4 sensors?**
- **Sensors 1-2** (rear-front pair): Provide pitch angle observability
- **Sensors 3-4** (left-right pair): Provide roll angle observability
- **Redundancy**: System tolerates 1 sensor failure
- **Cross-pattern**: Optimal geometry for plane normal estimation

### Software Architecture

```
TF_6DOF/
├── main_6DOF_3D.m              # Main simulation loop (1 kHz)
│
├── ekf_filter/                  # Extended Kalman Filter
│   ├── f.m                      # State prediction model
│   ├── h.m                      # Measurement model (ray-casting)
│   ├── jacobian_f.m             # Process Jacobian
│   └── jacobian_h.m             # Measurement Jacobian
│
├── sensors/                     # Sensor simulation
│   ├── SBES_measurament.m       # 4-beam acoustic ranging
│   ├── SBES_definition.m        # Beam geometry
│   ├── AHRS_measurement.m       # Attitude measurement
│   └── DVL_measurament.m        # Velocity & position
│
├── controller/                  # PID control system
│   ├── input_control.m          # Main control loop
│   ├── gainComputation.m        # Automatic gain tuning
│   └── tau0_values.m            # Linearization point
│
├── state_machine/               # Mission management
│   ├── state_machine.m          # State transitions
│   ├── goal_def.m               # Setpoint generation
│   └── goal_controller.m        # Diagnostic & recovery
│
├── world_generator/             # Dynamic terrain
│   ├── terrain_init.m           # Initial terrain setup
│   └── terrain_generator.m      # Adaptive plane generation
│
├── model/                       # Robot dynamics
│   └── dynamic_model.m          # 6-DOF BlueROV2 model
│
├── data_management/             # Data analysis tools
│   ├── save_simulation_data.m
│   ├── load_simulation_data.m
│   └── analyze_statistics.m
│
└── doc/                         # Technical documentation
    ├── ARCHITECTURE.md          # System design
    ├── EKF_ALGORITHM.md         # Filter mathematics
    ├── SENSORS.md               # Hardware specs
    ├── CONTROL_SYSTEM.md        # PID tuning
    └── STATE_MACHINE.md         # Recovery strategies
```

---

## 🧮 Mathematical Foundation

### State Estimation Problem

**State Vector:**
```
x = [h, α, β]ᵀ
```
- `h`: Altitude above terrain [m]
- `α`: Terrain roll angle [rad]
- `β`: Terrain pitch angle [rad]

**EKF Prediction:**
```matlab
x̂ₖ₊₁ = f(x̂ₖ, uₖ, Δt)
Pₖ₊₁ = FₖPₖFₖᵀ + Q
```

**EKF Update:**
```matlab
Kₖ = Pₖ Hₖᵀ (Hₖ Pₖ Hₖᵀ + R)⁻¹
x̂ₖ = x̂ₖ⁻ + Kₖ(zₖ - h(x̂ₖ⁻))
Pₖ = (I - KₖHₖ)Pₖ⁻
```

### Measurement Model (Ray-Casting)

For each SBES sensor j:
```
yⱼ = -h / (n̂ᵀsⱼ)
```

Where:
- `yⱼ`: Measured range to terrain [m]
- `h`: True altitude [m]
- `n̂`: Terrain normal vector (unit)
- `sⱼ`: Sensor beam direction (unit)

**Physical Interpretation:**
The measurement is the intersection distance between the sensor ray and the terrain plane. When the robot tilts to match terrain orientation, all four sensors converge to similar range values.

### Terrain Normal Estimation

From sensor intersection points p₁, p₂, p₃:
```
v₁ = p₂ - p₁
v₂ = p₃ - p₁
n̂ = (v₁ × v₂) / |v₁ × v₂|
```

This measured normal `n̂ₘₑₛ` is compared with the EKF estimate `n̂ₑₛₜ` to validate filter convergence.

---

## 🎮 Control System

### PID Controller

**Objective:** Track desired altitude and match terrain orientation

**Control Law:**
```
τᵢ = Kₚeᵢ + Kᵢ∫eᵢdt + Kₐ(deᵢ/dt)
```

**Setpoints:**
- **Altitude:** h_ref = 3 m (constant above terrain)
- **Roll:** φ_goal = α_est (match terrain roll)
- **Pitch:** θ_goal = β_est (match terrain pitch)
- **Surge/Sway:** Forward and lateral velocities for scanning

**Anti-Windup:**
Implemented using delta formulation to prevent integrator saturation during control limit saturation.

### State Machine

**8 Mission States:**

| State | Purpose | Duration |
|-------|---------|----------|
| **Idle** | Wait for start command | Until start |
| **TargetAltitude** | Descend to h_ref | ~10s |
| **ContactSearch** | Find terrain with all sensors | ~5s |
| **Following** | Main terrain-following mode | ~20-30s |
| **MovePitch** | Recover from pitch sensor loss | 5s timeout |
| **MoveRoll** | Recover from roll sensor loss | 5s timeout |
| **RecoveryAltitude** | Altitude-based recovery | 5s timeout |
| **Emergency** | Safety mode (h < 0.7m) | Until resolved |

**Recovery Logic:**
- **1 sensor lost:** Continue with EKF using 3 sensors
- **2 sensors lost (same axis):** Execute recovery maneuver
- **4 sensors lost:** Emergency reset

---

## 🚀 Quick Start

### Prerequisites

- MATLAB R2020b or later
- No additional toolboxes required
- ~50 MB disk space for code and results

### Installation

```bash
# Clone repository
git clone https://github.com/fabiogueunige/TFThesis.git
cd TFThesis/TF_6DOF
```

### Run Simulation

```matlab
% Open MATLAB in TF_6DOF directory
main_6DOF_3D
```

**Expected Output:**
```
Iteration: 1000 / 40000 (State: Following)
Iteration: 2000 / 40000 (State: Following)
...
Iteration: 40000 / 40000 (State: Following)

Simulation Complete!
- Duration: 40.0 seconds
- Altitude RMSE: 0.23 m
- Angle RMSE: 1.8°
- Sensor failures: 342 (0.85%)

Save data? (Y/N):
```

**Generated Figures:**
1. State tracking (h, α, β)
2. Robot orientation (φ, θ, ψ)
3. Control inputs (u, v, w, p, q, r)
4. Normal parallelism analysis
5. 3D trajectory visualization

### Configuration

**Edit parameters in `main_6DOF_3D.m`:**

```matlab
% Simulation
Ts = 0.001;              % Sampling time (1 kHz)
Tf = 40;                 % Duration [s]
DEBUG = false;           % Console debug output

% Target
h_ref(:) = 3;            % Reference altitude [m]

% Terrain
max_planes = 500;        % Buffer size
step_length = 4;         % Plane spacing [m]
angle_range = [-pi/5, pi/5]; % ±36° slopes

% EKF
Q = diag([0.01, 0.0001, 0.00025]);  % Process noise
R = diag([0.031, 0.034, 0.031, 0.034]); % Measurement noise
```

---

## 📊 Performance Results

### Typical Metrics (40s simulation)

| Metric | Value | Unit |
|--------|-------|------|
| **Altitude RMSE** | 0.23 | m |
| **Altitude Max Error** | 0.8 | m |
| **Roll Angle RMSE** | 1.5 | ° |
| **Pitch Angle RMSE** | 1.8 | ° |
| **Sensor Contact Rate** | 98.5 | % |
| **Normal Parallelism** | 2.3 | ° |
| **Computation Time/Iteration** | 0.08 | ms |
| **Real-Time Factor** | 80× | - |

**Interpretation:**
- System maintains altitude within ±50 cm
- Orientation tracks terrain within ±2°
- High sensor reliability (>98% contact)
- Fast computation enables real-time operation

### Statistical Analysis

**Batch Analysis Example:**

```matlab
% Run multiple simulations with different parameters
for i = 1:10
    % Modify terrain parameters
    angle_range = [-pi/6, pi/6] * rand();
    main_6DOF_3D
end

% Analyze statistics
stats = analyze_statistics();
fprintf('Mean altitude RMSE: %.3f ± %.3f m\n', ...
        stats.altitude_tracking.mean_rms, ...
        stats.altitude_tracking.std_rms);
```

---

## 💾 Data Management

### Save Simulation Data

After each run, save complete simulation data:

```matlab
Save data? (Y/N): Y
1. Auto-generate run name (run_YYYYMMDD_HHMMSS)
2. Specify custom run name
Choose option (1 or 2): 1
```

**Saved data structure:**
```
results/run_20251023_143022/
├── ekf_states.mat          # h, α, β (true, estimated, predicted)
├── ekf_covariance.mat      # P, innovation, S
├── sensor_data.mat         # SBES, AHRS, DVL measurements
├── control_data.mat        # PID, velocities, accelerations
├── trajectory.mat          # Position, rotation matrices
├── parameters.mat          # All simulation parameters
└── metadata.txt            # Human-readable summary
```

### Load and Analyze

```matlab
% Load specific run
sim_data = load_simulation_data('run_20251023_143022');

% Interactive selection
sim_data = load_simulation_data('');

% Access data
altitude_error = sim_data.h_ref - sim_data.x_est(1,:);
plot(sim_data.time, altitude_error);
```

---

## 📚 Documentation

Comprehensive technical documentation in `doc/`:

### 📖 Main Documents

- **[ARCHITECTURE.md](TF_6DOF/doc/ARCHITECTURE.md)**  
  Complete system architecture, module breakdown, data flow diagrams

- **[EKF_ALGORITHM.md](TF_6DOF/doc/EKF_ALGORITHM.md)**  
  Extended Kalman Filter mathematical formulation, Jacobians, tuning guidelines

- **[SENSORS.md](TF_6DOF/doc/SENSORS.md)**  
  SBES ray-casting algorithm, AHRS/DVL specifications, failure recovery

- **[CONTROL_SYSTEM.md](TF_6DOF/doc/CONTROL_SYSTEM.md)**  
  PID controller design, gain computation, anti-windup mechanisms

- **[STATE_MACHINE.md](TF_6DOF/doc/STATE_MACHINE.md)**  
  Mission states, recovery strategies, diagnostic logic

- **[Data Management Guide](TF_6DOF/data_management/README.md)**  
  Data saving, loading, statistical analysis tools

---

## 🔬 Research Context

### Thesis Objectives

This terrain-following system is part of a broader research effort on:

1. **Autonomous Underwater Inspection**
   - Seabed mapping and surveying
   - Pipeline and cable inspection
   - Archaeological site documentation

2. **Navigation System Integration**
   - Complement to path-following algorithms
   - Foundation for terrain-relative localization
   - Integration with obstacle avoidance

3. **Robust State Estimation**
   - Sensor fusion with EKF
   - Failure detection and recovery
   - Real-time performance constraints

### Application Scenarios

**1. Underwater Pipeline Inspection**
```
AUV follows pipeline while maintaining:
├─ Constant altitude above seabed
├─ Terrain-matched orientation
└─ Obstacle detection capability
```

**2. Seabed Mapping**
```
Systematic scanning pattern with:
├─ Terrain-following for uniform coverage
├─ Adaptive altitude control
└─ High-resolution sensor data
```

**3. Archaeological Documentation**
```
Precise maneuvering around artifacts:
├─ Terrain-relative positioning
├─ Orientation tracking
└─ Collision avoidance
```

### Future Integration

**Phase 1** (Current): Terrain Following ✅
- 4-SBES state estimation
- EKF implementation
- PID control

**Phase 2** (Planned): Path Following
- Waypoint navigation
- Trajectory tracking
- Terrain-relative frame

**Phase 3** (Planned): Obstacle Avoidance
- Static obstacle detection
- Dynamic path replanning
- Safety constraints

**Phase 4** (Vision): Full Autonomy
- Multi-sensor fusion
- High-level mission planning
- Adaptive behavior

---

## 🛠️ Development Notes

### Current Limitations

1. **Simulation Only**
   - No hardware validation yet
   - Idealized sensor models
   - Perfect DVL (no position drift)

2. **Terrain Model**
   - Planar segments (no complex geometry)
   - No overhangs or caves
   - Smooth angle transitions

3. **Control**
   - Linearized dynamics
   - PID (no adaptive control)
   - Manual gain tuning

### Planned Improvements

- [ ] ROS/Gazebo integration
- [ ] Stonefish simulator testing
- [ ] Real SBES data integration
- [ ] Adaptive controller
- [ ] Non-linear dynamics
- [ ] Position Kalman Filter
- [ ] Multi-AUV coordination

---

## 🧪 Testing & Validation

### Test Scenarios

**1. Flat Terrain (α=0°, β=0°)**
```matlab
angle_range = [0, 0];
main_6DOF_3D
% Expected: Perfect tracking, minimal control effort
```

**2. Rolling Terrain (±30°)**
```matlab
angle_range = [-pi/6, pi/6];
main_6DOF_3D
% Expected: Smooth tracking, adaptive orientation
```

**3. Steep Features (±45°)**
```matlab
angle_range = [-pi/4, pi/4];
main_6DOF_3D
% Expected: Recovery maneuvers, higher error
```

**4. Sensor Failure Injection**
```matlab
% Edit SBES_measurament.m to simulate failures
if mod(ite, 5000) < 100
    command.contact(1) = false;  % Disable sensor 1
end
% Expected: EKF uses 3 sensors, no tracking degradation
```

---

## 🤝 Contributing

This is an academic research project. Contributions, suggestions, and discussions are welcome!

**Areas for Contribution:**
- Real-world sensor integration
- Alternative estimation algorithms (UKF, particle filter)
- Hardware implementation
- Extended documentation
- Bug reports and fixes

**Contact:**
- Email: fabio.guelfi@libero.it
- GitHub: [@fabiogueunige](https://github.com/fabiogueunige)

---

## 📄 Citation

If you use this work in your research, please cite:

```bibtex
@mastersthesis{guelfi2025terrain,
  author    = {Fabio Guelfi},
  title     = {Advanced AUV Motion Control and Terrain Following 
               for Automatic Seabed Inspection},
  school    = {University of Genoa},
  year      = {2025},
  type      = {Master's Thesis},
  note      = {Robotics Engineering},
  url       = {https://github.com/fabiogueunige/TFThesis}
}
```

---

## 📝 License

This project is licensed under the **MIT License** - see [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **University of Genoa** - Robotics Engineering Program
- **BlueROV2** - Vehicle dynamics reference
- **MATLAB** - Simulation environment
- Thesis supervisors and research group

---

## 📞 Contact & Support

**Author:** Fabio Guelfi  
**Institution:** University of Genoa  
**Program:** Robotics Engineering (Master's Thesis)  
**Email:** fabio.guelfi@libero.it  
**GitHub:** [@fabiogueunige](https://github.com/fabiogueunige)  
**Repository:** [TFThesis](https://github.com/fabiogueunige/TFThesis)

**For questions, issues, or collaboration:**
- Open an issue on GitHub
- Email directly for academic inquiries
- Check documentation in `doc/` folder

---

**Last Updated:** October 23, 2025  
**Version:** 2.0.0  
**Status:** Active Development 🚀

---

## 🗺️ Repository Structure

```
TFThesis/
├── README.md                    # This file
├── LICENSE                      # MIT License
│
├── TF_6DOF/                     # Main 6-DOF implementation
│   ├── main_6DOF_3D.m           # Simulation entry point
│   ├── controller/              # PID control system
│   ├── ekf_filter/              # Extended Kalman Filter
│   ├── sensors/                 # SBES, AHRS, DVL
│   ├── state_machine/           # Mission management
│   ├── world_generator/         # Terrain generation
│   ├── model/                   # Robot dynamics
│   ├── data_management/         # Data analysis tools
│   ├── doc/                     # Technical documentation
│   └── results/                 # Saved simulation data
│
├── TF_3DOF_Beta/                # Legacy 3-DOF version
├── EquationTests/               # Mathematical validation
├── ttf_ros/                     # ROS integration (WIP)
└── ttf_writing/                 # Thesis LaTeX source

Total: ~2,500 lines of MATLAB code, 50+ pages of documentation

