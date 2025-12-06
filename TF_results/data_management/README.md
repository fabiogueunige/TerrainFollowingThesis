# Data Management System

## Overview

This data management system allows you to save, load, and analyze simulation data for statistical analysis and performance evaluation.

**NEW:** Enhanced with comprehensive performance metrics following **RESULTS_GUIDE_2.pdf** specifications (Section 2.4).

## 🚀 Quick Start

### Single Run Analysis (All Results + All Plots)
```matlab
analyze_single_run();                   % Analyze most recent run
analyze_single_run('run_20251201_105813');  % Analyze specific run
```

### Batch Statistical Analysis (Variance, Covariance, Confidence Intervals)
```matlab
stats = batch_statistical_analysis();   % Analyze all runs
% Access results:
stats.summary       % Mean, std, variance, min, max
stats.covariance    % Covariance matrix between metrics
stats.correlation   % Pearson correlation coefficients
stats.confidence    % 95% and 99% confidence intervals
stats.distribution  % Skewness, kurtosis
```

### Test Complete Workflow
```matlab
test_complete_workflow;  % Run verification tests
```

## 📁 Directory Structure

After running simulations with data saving enabled, the following structure is created:

```
matlab_3D/
├── results/
│   ├── run_20251023_143022/
│   │   ├── ekf_states.mat
│   │   ├── ekf_covariance.mat
│   │   ├── sensor_data.mat
│   │   ├── control_data.mat
│   │   ├── trajectory.mat
│   │   ├── parameters.mat
│   │   └── metadata.txt
│   ├── run_20251023_150130/
│   │   └── ...
│   └── run_custom_name/
│       └── ...
└── data_management/
    ├── save_simulation_data.m
    ├── collect_simulation_data.m
    ├── load_simulation_data.m
    └── analyze_statistics.m
```

## 📊 Saved Data Categories

### 1. **EKF States** (`ekf_states.mat`)
- `x_true` - True terrain states [h, alpha, beta]
- `x_est` - EKF estimated states
- `x_pred` - EKF predicted states
- `h_ref` - Reference altitude profile
- `time` - Time vector

### 2. **EKF Covariance** (`ekf_covariance.mat`)
- `ni` - Innovation (measurement residual)
- `S` - Innovation covariance
- `P_final` - Final state covariance
- `P0` - Initial covariance

### 3. **Sensor Data** (`sensor_data.mat`)
- `z_meas` - SBES range measurements (4 sensors)
- `z_pred` - Predicted measurements
- `n_mes` - Measured terrain normals
- `n_est` - Estimated terrain normals
- `n_pre` - Predicted terrain normals
- `rob_rot` - Robot orientation (with noise)
- `clean_rot` - Robot orientation (clean)
- `R` - Measurement noise covariance

### 4. **Control Data** (`control_data.mat`)
- `pid` - PID controller output
- `u` - Robot velocities [u, v, w, p, q, r]
- `u_dot` - Robot accelerations
- `goal` - Desired setpoints
- `integral_err`, `p_err`, `i_err` - Controller error terms

### 5. **Trajectory** (`trajectory.mat`)
- `prob` - Robot position in world frame
- `wRr` - Robot rotation matrices
- `wRt` - Terrain rotation matrices
- `state` - State machine history

### 6. **Parameters** (`parameters.mat`)
- Simulation parameters (Ts, Tf, N)
- EKF parameters (Q, R, P0)
- Controller gains (Kp, Ki, Kd, Kt)
- Initial conditions (x0, speed0, etc.)
- **Terrain parameters** (max_planes, step_length, angle_range, rate_of_change, pp_init_w, n0)

### 7. **Metadata** (`metadata.txt`)
Human-readable summary including:
- Run timestamp
- Simulation parameters
- Final performance metrics
- Initial conditions
- **Terrain generation parameters**

## 🚀 Usage

### Running Simulation with Data Saving

1. **Run the simulation:**
   ```matlab
   main_6DOF_3D
   ```

2. **At the end, you'll be prompted:**
   ```
   Save data? (Y/N):
   ```

3. **Choose naming option:**
   - Option 1: Auto-generate timestamp name
   - Option 2: Custom name (e.g., `test_high_altitude`)

4. **Data is saved automatically!**

### Loading Saved Data

**List all available runs:**
```matlab
sim_data = load_simulation_data('');
% Shows numbered list, prompts for selection
```

**Load specific run:**
```matlab
sim_data = load_simulation_data('run_20251023_143022');
```

**Access data:**
```matlab
% EKF states
altitude_error = sim_data.h_ref - sim_data.x_est(1,:);

% Control signals
surge_velocity = sim_data.u(1,:);

% Sensor measurements
sbes_ranges = sim_data.z_meas;

% Terrain parameters
buffer_size = sim_data.max_planes;           % 500
plane_distance = sim_data.step_length;       % 4 m
angle_limits = sim_data.angle_range;         % [-π/3, π/3]
terrain_variation = sim_data.rate_of_change; % 3
```

### Statistical Analysis

**Analyze all runs:**

```matlab
stats = analyze_statistics();
```

**Analyze specific runs:**

```matlab
run_list = {'run1', 'run2', 'run3'};
stats = analyze_statistics(run_list);
```

### 🆕 Performance Metrics (RESULTS_GUIDE_2.pdf)

**Compute comprehensive metrics for single run:**

```matlab
% Load simulation data
sim_data = load_simulation_data('run_20251201_120426');

% Compute all metrics
metrics = compute_performance_metrics(sim_data);

% Display formatted results
display_performance_metrics(metrics, true);  % true = verbose mode
```

**Performance Metrics Computed:**

1. **RMS Altitude Error** - Altitude-holding precision
2. **Mean Angle Tracking Error** - |φ-α| and |θ-β| tracking
3. **Sensor Failure Rate** - % time with sensor_fail > 0
4. **State Transition Frequency** - Transitions per minute
5. **Control Effort** - Time-averaged ||u||₂
6. **Maximum Innovation** - ||ν||∞ (EKF consistency)
7. **Normal Parallelism** - ∠(n_est, n_mes) consistency
8. **Robot-Terrain Alignment** - ∠(z_robot, n_est)

**Quick performance check:**

```matlab
% Summary display only
display_performance_metrics(metrics, false);
```

**Batch analysis example:**

```matlab
% Run example script
example_performance_analysis
```

**Access statistics:**
```matlab
% Altitude tracking
mean_error = stats.altitude_tracking.mean_error;
rms_error = stats.altitude_tracking.mean_rms;

% Control effort
surge_effort = stats.control_effort.surge_mean;

% Per-run data
run1_altitude = stats.runs{1}.altitude;
```

## 📈 Performance Metrics

The analysis system computes:

### Tracking Performance
- **Altitude Error:** Mean, RMS, Max, Std
- **Angle Error:** Alpha and Beta tracking errors
- **Innovation:** Consistency check for EKF

### Control Effort
- **RMS velocities:** For all 6 DOFs
- **Control signal statistics**

### Sensor Performance
- **Measurement residuals**
- **Normal vector accuracy**

## 💡 Examples

### Example 1: Compare Two Controller Configurations

```matlab
% Run simulation with controller A
% ... modify gains ...
main_6DOF_3D
% Save as "run_controller_A"

% Run simulation with controller B
% ... modify gains ...
main_6DOF_3D
% Save as "run_controller_B"

% Compare
stats = analyze_statistics({'run_controller_A', 'run_controller_B'});
```

### Example 2: Monte Carlo Analysis

```matlab
% Run multiple simulations
for i = 1:10
    main_6DOF_3D
    % Save with auto-generated names
end

% Analyze all
stats = analyze_statistics();

% Results show mean ± std across all runs
```

### Example 3: Load and Re-plot

```matlab
% Load old simulation
sim_data = load_simulation_data('run_20251023_143022');

% Custom plot
figure;
plot(sim_data.time, sim_data.x_est(1,:));
hold on;
plot(sim_data.time, sim_data.h_ref);
xlabel('Time [s]');
ylabel('Altitude [m]');
legend('Estimated', 'Reference');
title('Altitude Tracking - Reloaded Data');
```

## 🔧 Advanced Usage

### Custom Metrics

Add custom metrics to `compute_run_metrics()` in `analyze_statistics.m`:

```matlab
% Example: Add settling time metric
settling_idx = find(abs(h_error) < 0.1, 1, 'first');
run_stats.altitude.settling_time = sim_data.time(settling_idx);
```

### Batch Export

Export statistics to CSV:

```matlab
stats = analyze_statistics();
writetable(struct2table(stats.altitude_tracking), 'altitude_stats.csv');
```

### Plot Multiple Runs

```matlab
runs = {'run1', 'run2', 'run3'};
figure; hold on;
for i = 1:length(runs)
    data = load_simulation_data(runs{i});
    plot(data.time, data.x_est(1,:), 'DisplayName', runs{i});
end
legend; xlabel('Time [s]'); ylabel('Altitude [m]');
```

## 📝 Notes

- **Storage:** Each run takes ~10-50 MB depending on simulation length
- **Metadata:** Always check `metadata.txt` for run parameters
- **Backup:** Store `results/` folder for long-term archival
- **Naming:** Use descriptive names for easier identification

## 🆘 Troubleshooting

**Issue:** "No results directory found"
- **Solution:** Run at least one simulation with saving enabled

**Issue:** "Run directory not found"
- **Solution:** Check run name spelling, use `load_simulation_data('')` to list available runs

**Issue:** Out of memory when analyzing many runs
- **Solution:** Analyze runs in batches or reduce `N` in simulation

## 📚 Related Documentation

- See `doc/ARCHITECTURE.md` for overall system design
- See `doc/EKF_ALGORITHM.md` for state estimation details
- See `doc/CONTROL_SYSTEM.md` for controller design

---

**Last Updated:** October 23, 2025
