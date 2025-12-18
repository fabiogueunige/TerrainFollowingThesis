# Data Management System

## Overview

This data management system allows you to save, load, and analyze simulation data for statistical analysis and performance evaluation of the 6-DOF AUV terrain following system.

**UPDATED:** Complete rewrite with comprehensive metrics for terrain following analysis including:
- EKF SBES (altitude, α, β estimation)
- EKF Position Filter (15-state with ground truth comparison)
- PID Controllers with anti-windup
- State Machine analysis
- Sensor failure tracking

## 🚀 Quick Start

### Single Run Analysis (All Results + All Plots)
```matlab
analyze_single_run();                         % Analyze most recent run
analyze_single_run('run_20251201_105813');    % Analyze specific run
analyze_single_run('run_name', 'save_plots', true);  % Save plots to files
```

This generates:
- All performance metrics
- 20+ separate figures (each plot in its own window)
- State machine transition graph
- Quality grade (A-F)

### Batch Statistical Analysis (Multiple Runs)
```matlab
stats = batch_statistical_analysis();   % Analyze all runs

% Access results:
stats.summary       % Mean, std, variance, min, max for 13 metrics
stats.covariance    % Covariance matrix between metrics
stats.correlation   % Pearson correlation coefficients
stats.confidence    % 95% and 99% confidence intervals
stats.distribution  % Skewness, kurtosis, normality
stats.data_matrix   % Raw data [runs × metrics]
```

### Quick Statistics
```matlab
stats = analyze_statistics();   % Simpler statistical summary
```

## 📁 Directory Structure

After running simulations with data saving enabled:

```
TF_6DOF/
├── results/
│   ├── run_20251201_105813/
│   │   ├── ekf_sbes_states.mat      % EKF SBES: h, α, β estimation
│   │   ├── ekf_sbes_covariance.mat  % Innovation, covariance
│   │   ├── ekf_position_filter.mat  % 15-state position filter
│   │   ├── sensor_data.mat          % SBES measurements, failures
│   │   ├── control_data.mat         % PID, velocities, errors
│   │   ├── trajectory.mat           % 3D path, rotations
│   │   ├── parameters.mat           % All simulation parameters
│   │   ├── state_machine.mat        % State history
│   │   └── metadata.txt             % Human-readable summary
│   └── batch_analysis_YYYYMMDD_HHMMSS.mat
└── data_management/
    ├── save_simulation_data.m
    ├── collect_simulation_data.m
    ├── load_simulation_data.m
    ├── compute_performance_metrics.m
    ├── display_performance_metrics.m
    ├── analyze_single_run.m
    ├── analyze_statistics.m
    ├── batch_statistical_analysis.m
    └── README.md
```

## 📊 Saved Data Categories

### 1. EKF SBES States (`ekf_sbes_states.mat`)
| Variable | Size | Description |
|----------|------|-------------|
| `x_true` | 3×N | True terrain states [h, α, β] |
| `x_est` | 3×N | EKF estimated states |
| `x_pred` | 3×N | EKF predicted states |
| `h_ref` | 1×N | Reference altitude profile |
| `time` | 1×N | Time vector |

### 2. EKF SBES Covariance (`ekf_sbes_covariance.mat`)
| Variable | Size | Description |
|----------|------|-------------|
| `ni` | 4×N | Innovation (measurement residual) |
| `S` | 4×4×N | Innovation covariance |
| `P_final` | 3×3 | Final state covariance |
| `P0` | 3×3 | Initial covariance |

### 3. EKF Position Filter (`ekf_position_filter.mat`)
| Variable | Size | Description |
|----------|------|-------------|
| `x_loc` | 15×N | Full EKF state [pos(3), att(3), vel(3), ω(3), bias(3)] |
| `eta_gt` | 6×N | Ground truth position & orientation |
| `nu_gt` | 6×N | Ground truth body velocities |
| `wRr_gt` | 3×3×N | Ground truth rotation matrices |

### 4. Sensor Data (`sensor_data.mat`)
| Variable | Size | Description |
|----------|------|-------------|
| `z_meas` | 4×N | SBES range measurements |
| `z_pred` | 4×N | Predicted measurements |
| `sensor_fail` | 1×N | Number of failed sensors |
| `n_mes` | 3×N | Measured terrain normals |
| `n_est` | 3×N | Estimated terrain normals |
| `rob_rot` | 3×N | Robot orientation (noisy) |
| `clean_rot` | 3×N | Robot orientation (clean) |

### 5. Control Data (`control_data.mat`)
| Variable | Size | Description |
|----------|------|-------------|
| `u` | 6×N | Body velocities [u,v,w,p,q,r] |
| `u_dot` | 6×N | Body accelerations |
| `pid` | 6×N | PID controller outputs |
| `integral_err` | 6×N | Integral error terms |
| `Kp`, `Ki`, `Kd`, `Kt` | 1×6 | PID gains |

### 6. Trajectory (`trajectory.mat`)
| Variable | Size | Description |
|----------|------|-------------|
| `prob` | 3×N | Robot position [x,y,z] world frame |
| `wRr` | 3×3×N | Robot rotation matrices |
| `wRt` | 3×3×N | Terrain rotation matrices |
| `goal` | struct(N) | Goal setpoints |

### 7. State Machine (`state_machine.mat`)
| Variable | Size | Description |
|----------|------|-------------|
| `state_numeric` | 1×N | State index (1-10) |
| `state_names` | cell | State name lookup |

### 8. Parameters (`parameters.mat`)
All simulation configuration:
- Time parameters (Ts, Tf, N)
- EKF matrices (Q, R, P0)
- Controller gains (Kp, Ki, Kd, Kt)
- Terrain parameters (max_planes, step_length, angle_range, rate_of_change)
- AHRS noise parameters
- Initial conditions

## 📈 Performance Metrics Computed

### `compute_performance_metrics(sim_data)` returns:

#### 1. Altitude Tracking (`metrics.altitude`)
- `rms_error` - Root mean square error [m]
- `mae` - Mean absolute error [m]
- `max_error` - Maximum error [m]
- `settling_time` - Time to settle [s]
- `transient_rms` - RMS during transient (first 20%)
- `steady_state_rms` - RMS during steady state

#### 2. Angle Tracking (`metrics.angle_tracking`)
- `phi_alpha_mean` - Mean |φ-α| error (roll vs terrain)
- `theta_beta_mean` - Mean |θ-β| error (pitch vs terrain)
- `combined_rms` - Combined angle tracking RMS

#### 3. EKF SBES (`metrics.ekf_sbes`)
- `alpha_rmse`, `beta_rmse` - Terrain angle estimation RMSE
- `h_rmse` - Altitude estimation RMSE
- `innovation_mean`, `innovation_max` - Innovation statistics
- `consistency_ratio` - Filter consistency check

#### 4. EKF Position Filter (`metrics.ekf_position`)
- `pos_rmse_x/y/z`, `pos_rmse_total` - Position errors [m]
- `ang_rmse_roll/pitch/yaw` - Orientation errors [deg]
- `vel_rmse_surge/sway/heave` - Velocity errors [m/s]
- `rate_rmse_p/q/r` - Angular rate errors [deg/s]

#### 5. Sensors (`metrics.sensors`)
- `failure_rate` - % time with failures
- `max_simultaneous_failures` - Worst case
- `sensor_X_failure_rate` - Per-sensor rates
- `mean_time_between_failures` - MTBF

#### 6. State Machine (`metrics.state_machine`)
- `total_transitions` - Number of state changes
- `transitions_per_minute` - Transition rate
- `following_percentage` - % time in Following state
- `recovery_percentage` - % time in recovery states
- `occupancy` - Per-state occupancy array

#### 7. Control (`metrics.control`)
- `surge_rms` through `yaw_rms` - Per-axis RMS
- `total_effort_rms` - Overall control effort
- `energy` - Control energy proxy

#### 8. Geometry (`metrics.geometry`)
- `normal_parallelism_mean` - ∠(n_est, n_mes) [deg]
- `robot_alignment_mean` - ∠(z_robot, n_est) [deg]
- `normal_parallelism_below_5deg` - % time well aligned

#### 9. Overall (`metrics.overall`)
- `score` - Performance score [0-100]
- `grade` - Letter grade (A-F)

## 🎨 Plots Generated by `analyze_single_run`

Each plot opens in its own figure window with unique **Tag** for export:

1. **State Machine Transitions** (`state_machine`) - State over time + transition markers
2. **3D Trajectory** (`trajectory_3d`) - Color-coded by time
3. **Altitude Tracking** (`altitude_tracking`) - Reference vs estimated with error
4. **Terrain Alpha** (`terrain_alpha`) - α tracking vs robot roll
5. **Terrain Beta** (`terrain_beta`) - β tracking vs robot pitch
6. **Robot Roll/Pitch/Yaw** (`robot_roll`, `robot_pitch`, `robot_yaw`) - 3 separate figures
7. **Control Inputs** (`control_surge`, ..., `control_yaw`) - 6 figures
8. **Normal Parallelism** (`normal_parallelism`) - Alignment angles over time
9. **Innovation** (`ekf_innovation`) - EKF innovation per sensor + norm
10. **SBES Measurements** (`sbes_measurements`) - 4 sensors measured vs predicted
11. **EKF Position XYZ** (`ekf_pos_xyz`) - Position vs ground truth
12. **EKF Position Angles** (`ekf_pos_angles`) - Orientation vs ground truth
13. **EKF Position Velocities** (`ekf_pos_velocities`) - Velocity vs ground truth
14. **EKF Position Errors** (`ekf_pos_errors`) - All estimation errors
15. **Sensor Failures** (`sensor_failures`) - Failure timeline

### Exporting Figures as PNG

All figures have a `Tag` property for programmatic export:

```matlab
% Export single figure by tag
fig = findobj('Tag', 'altitude_tracking');
exportgraphics(fig, 'altitude_tracking.png', 'Resolution', 300);

% Export all figures in one loop
figs = findall(0, 'Type', 'figure');
for i = 1:length(figs)
    if ~isempty(figs(i).Tag)
        exportgraphics(figs(i), [figs(i).Tag '.png'], 'Resolution', 300);
    end
end
```

## 🔬 EKF SBES Consistency Analysis

Use `analyze_ekf_sbes(run_name)` for detailed EKF evaluation:

### NEES Analysis (Normalized Estimation Error Squared)
- Checks if covariance matches actual estimation errors
- Should follow chi-squared distribution with 3 DOF
- 95% of values should fall within bounds

### NIS Analysis (Normalized Innovation Squared)
- Validates measurement model consistency
- Should follow chi-squared distribution with 4 DOF (sensors)

### Generated Plots (with Tags):
- `ekf_sbes_altitude` - Altitude estimation
- `ekf_sbes_altitude_error` - Estimation error
- `ekf_sbes_alpha` / `ekf_sbes_beta` - Terrain angles
- `ekf_sbes_nees` - NEES over time with bounds
- `ekf_sbes_nis` - NIS over time with bounds
- `ekf_sbes_innovation` - Per-sensor innovation
- `ekf_sbes_innovation_histogram` - Distribution check
- `ekf_sbes_innovation_autocorr` - Whiteness check
- `ekf_sbes_covariance` - Covariance evolution
- `ekf_sbes_tracking_error` - |φ-α| and |θ-β|

## 🔧 Usage Examples

### Save Simulation Data
```matlab
% At end of main_6DOF_3D.m, call:
sim_data = collect_simulation_data(time, N, Ts, Tf, h_ref, ...
    x_true, x_est, x_pred, x_loc, eta_gt, nu_gt, wRr_gt, ...
    ni, S, P, P0, Q, R, ...
    z_meas, z_pred, sensor_fail, n_mes, n_est, n_pre, ...
    rob_rot, clean_rot, R_noise, sigma_ahrs, ...
    u, u_dot, pid, integral_err, p_err, i_err, Kp, Ki, Kd, Kt, ...
    prob, wRr, wRt, goal, ...
    state, state_numeric, state_names, ...
    max_planes, step_length, angle_range, rate_of_change, pp_init_w, n0);

save_simulation_data(sim_data);
```

### Load and Analyze
```matlab
% Load specific run
sim_data = load_simulation_data('run_20251204_160543');

% Compute metrics
metrics = compute_performance_metrics(sim_data);

% Display formatted results
display_performance_metrics(metrics, true);

% Or use all-in-one:
analyze_single_run('run_20251204_160543');
```

### Batch Analysis
```matlab
% Analyze all runs with statistical summary
stats = batch_statistical_analysis();

% Check correlation between altitude error and control effort
r = stats.correlation.matrix(1, 10);
fprintf('Correlation: %.3f\n', r);

% 95% confidence interval for altitude error
ci = stats.confidence.altitude_rms.ci_95;
fprintf('Altitude RMS: [%.3f, %.3f] m (95%% CI)\n', ci(1), ci(2));
```

### Compare Configurations
```matlab
% Run with config A, save as 'config_A'
% Run with config B, save as 'config_B'

stats = batch_statistical_analysis({'config_A', 'config_B'});
% Compare means, check for significant differences
```

## 🔄 Backward Compatibility

The system automatically handles old data formats:
- Missing fields are filled with NaN or default values
- Old single-file format converted to new multi-file format
- `data_version` field tracks format version

```matlab
% Old data still loads correctly
old_data = load_simulation_data('run_20251023_143022');
% Missing EKF position data will be zeros/NaN
```

## 💡 Tips

1. **Use descriptive names** when saving: `terrain_steep_30deg` is better than `run_1`
2. **Check metadata.txt** for quick parameter overview without loading .mat files
3. **Save plots** using `analyze_single_run('name', 'save_plots', true)`
4. **For thesis**: Use `batch_statistical_analysis` with all your experimental runs

## 📚 Related Documentation

- `doc/ARCHITECTURE.md` - System design overview
- `doc/EKF_ALGORITHM.md` - EKF SBES estimation details
- `doc/STATE_MACHINE.md` - State machine design
- `doc/CONTROL_SYSTEM.md` - PID controller design
- `doc/SENSORS.md` - SBES sensor model

---

**Last Updated:** December 2025
