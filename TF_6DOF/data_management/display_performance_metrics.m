%% DISPLAY_PERFORMANCE_METRICS - Display comprehensive performance metrics
%
% Formats and displays all performance metrics computed by
% compute_performance_metrics in a structured, readable format.
%
% SYNTAX:
%   display_performance_metrics(metrics)
%   display_performance_metrics(metrics, verbose)
%
% INPUTS:
%   metrics - Structure from compute_performance_metrics
%   verbose - (Optional) true for detailed output, false for summary
%             Default: true
%
% OUTPUTS:
%   None (prints to console)
%
% EXAMPLE:
%   sim_data = load_simulation_data('run_20251201_120426');
%   metrics = compute_performance_metrics(sim_data);
%   display_performance_metrics(metrics);
%
% See also: compute_performance_metrics, load_simulation_data

function display_performance_metrics(metrics, verbose)
    
    if nargin < 2
        verbose = true;
    end
    
    fprintf('\n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('              TERRAIN FOLLOWING PERFORMANCE METRICS            \n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('\n');
    
    %% 1. Altitude Tracking Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 1. ALTITUDE TRACKING PERFORMANCE                            │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'altitude') && ~isempty(fieldnames(metrics.altitude))
        fprintf('  RMS Error:               %8.4f m\n', get_metric(metrics.altitude, 'rms_error', NaN));
        fprintf('  MAE (Mean Abs Error):    %8.4f m\n', get_metric(metrics.altitude, 'mae', NaN));
        fprintf('  Maximum Error:           %8.4f m\n', get_metric(metrics.altitude, 'max_error', NaN));
        fprintf('  Standard Deviation:      %8.4f m\n', get_metric(metrics.altitude, 'std_error', NaN));
        fprintf('  Final Error:             %8.4f m\n', get_metric(metrics.altitude, 'final_error', NaN));
        
        if verbose
            fprintf('\n  Transient Analysis:\n');
            fprintf('    Transient RMS (0-20%%): %8.4f m\n', get_metric(metrics.altitude, 'transient_rms', NaN));
            fprintf('    Steady-State RMS:      %8.4f m\n', get_metric(metrics.altitude, 'steady_state_rms', NaN));
            fprintf('    Settling Time:         %8.2f s\n', get_metric(metrics.altitude, 'settling_time', NaN));
        end
        
        % Quality assessment
        rms_err = get_metric(metrics.altitude, 'rms_error', NaN);
        if rms_err < 0.2
            fprintf('                           ✓ Excellent (< 0.2 m)\n');
        elseif rms_err < 0.5
            fprintf('                           ✓ Good (< 0.5 m)\n');
        elseif rms_err < 1.0
            fprintf('                           ⚠ Acceptable (< 1.0 m)\n');
        else
            fprintf('                           ✗ Poor (≥ 1.0 m)\n');
        end
    else
        fprintf('  (Altitude data not available)\n');
    end
    fprintf('\n');
    
    %% 2. Angle Tracking Performance (Robot vs Terrain)
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 2. ANGLE TRACKING (Robot vs Terrain)                        │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'angle_tracking') && ~isempty(fieldnames(metrics.angle_tracking))
        fprintf('  Mean |φ - α| Error:      %8.4f° (± %.4f°)\n', ...
                rad2deg(get_metric(metrics.angle_tracking, 'phi_alpha_mean', 0)), ...
                rad2deg(get_metric(metrics.angle_tracking, 'phi_alpha_std', 0)));
        fprintf('  Mean |θ - β| Error:      %8.4f° (± %.4f°)\n', ...
                rad2deg(get_metric(metrics.angle_tracking, 'theta_beta_mean', 0)), ...
                rad2deg(get_metric(metrics.angle_tracking, 'theta_beta_std', 0)));
        fprintf('  Combined Mean Error:     %8.4f°\n', ...
                rad2deg(get_metric(metrics.angle_tracking, 'combined_mean', 0)));
        fprintf('  Combined RMS Error:      %8.4f°\n', ...
                rad2deg(get_metric(metrics.angle_tracking, 'combined_rms', 0)));
        
        if verbose
            fprintf('\n  Max Errors:\n');
            fprintf('    |φ - α| Max:           %8.4f°\n', ...
                    rad2deg(get_metric(metrics.angle_tracking, 'phi_alpha_max', 0)));
            fprintf('    |θ - β| Max:           %8.4f°\n', ...
                    rad2deg(get_metric(metrics.angle_tracking, 'theta_beta_max', 0)));
        end
    else
        fprintf('  (Angle tracking data not available)\n');
    end
    fprintf('\n');
    
    %% 3. EKF SBES Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 3. EKF SBES (Terrain Estimation)                            │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'ekf_sbes') && ~isempty(fieldnames(metrics.ekf_sbes))
        fprintf('  Altitude Estimation:\n');
        fprintf('    RMSE:                  %8.4f m\n', get_metric(metrics.ekf_sbes, 'h_rmse', NaN));
        fprintf('    MAE:                   %8.4f m\n', get_metric(metrics.ekf_sbes, 'h_mae', NaN));
        fprintf('    Max Error:             %8.4f m\n', get_metric(metrics.ekf_sbes, 'h_max', NaN));
        
        fprintf('\n  Alpha (α) Estimation:\n');
        fprintf('    RMSE:                  %8.4f° (%.6f rad)\n', ...
                rad2deg(get_metric(metrics.ekf_sbes, 'alpha_rmse', 0)), ...
                get_metric(metrics.ekf_sbes, 'alpha_rmse', 0));
        fprintf('    MAE:                   %8.4f°\n', rad2deg(get_metric(metrics.ekf_sbes, 'alpha_mae', 0)));
        
        fprintf('\n  Beta (β) Estimation:\n');
        fprintf('    RMSE:                  %8.4f° (%.6f rad)\n', ...
                rad2deg(get_metric(metrics.ekf_sbes, 'beta_rmse', 0)), ...
                get_metric(metrics.ekf_sbes, 'beta_rmse', 0));
        fprintf('    MAE:                   %8.4f°\n', rad2deg(get_metric(metrics.ekf_sbes, 'beta_mae', 0)));
        
        fprintf('\n  Innovation Statistics:\n');
        fprintf('    Mean ||ν||:            %8.4f\n', get_metric(metrics.ekf_sbes, 'innovation_mean', NaN));
        fprintf('    Max ||ν||:             %8.4f\n', get_metric(metrics.ekf_sbes, 'innovation_max', NaN));
        fprintf('    Std ||ν||:             %8.4f\n', get_metric(metrics.ekf_sbes, 'innovation_std', NaN));
        
        if isfield(metrics.ekf_sbes, 'consistency_ratio')
            ratio = metrics.ekf_sbes.consistency_ratio;
            fprintf('    Consistency Ratio:     %8.4f ', ratio);
            if ratio > 0.5 && ratio < 2.0
                fprintf('(✓ Filter consistent)\n');
            else
                fprintf('(⚠ Check filter tuning)\n');
            end
        end
    else
        fprintf('  (EKF SBES data not available)\n');
    end
    fprintf('\n');
    
    %% 4. Sensor Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 4. SENSOR PERFORMANCE                                       │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'sensors') && ~isempty(fieldnames(metrics.sensors))
        failure_rate = get_metric(metrics.sensors, 'failure_rate', 0);
        fprintf('  Sensor Failure Rate:     %8.2f%% of time\n', failure_rate);
        fprintf('  Total Failures:          %8d\n', get_metric(metrics.sensors, 'total_failures', 0));
        fprintf('  Max Simultaneous Fails:  %8d\n', get_metric(metrics.sensors, 'max_simultaneous_failures', 0));
        fprintf('  Mean Time Between Fails: %8.2f s\n', get_metric(metrics.sensors, 'mean_time_between_failures', 0));
        
        % Per-sensor statistics
        if verbose
            fprintf('\n  Per-Sensor Failure Rates:\n');
            for s = 1:4
                field_name = sprintf('sensor_%d_failure_rate', s);
                if isfield(metrics.sensors, field_name)
                    fprintf('    Sensor %d:              %8.2f%%\n', s, metrics.sensors.(field_name));
                end
            end
        end
        
        % Quality assessment
        if failure_rate < 5
            fprintf('                           ✓ Excellent (< 5%%)\n');
        elseif failure_rate < 10
            fprintf('                           ✓ Good (5-10%%)\n');
        elseif failure_rate < 20
            fprintf('                           ⚠ Acceptable (10-20%%)\n');
        else
            fprintf('                           ✗ High (> 20%%) - Check terrain\n');
        end
    else
        fprintf('  (Sensor data not available)\n');
    end
    fprintf('\n');
    
    %% 5. State Machine Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 5. STATE MACHINE PERFORMANCE                                │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'state_machine') && ~isempty(fieldnames(metrics.state_machine))
        fprintf('  Total Transitions:       %8d\n', get_metric(metrics.state_machine, 'total_transitions', 0));
        fprintf('  Transitions/Minute:      %8.2f\n', get_metric(metrics.state_machine, 'transitions_per_minute', 0));
        fprintf('  Mission Duration:        %8.2f min\n', get_metric(metrics.state_machine, 'duration_min', 0));
        fprintf('\n  Following Mode:\n');
        fprintf('    Time in Following:     %8.2f s (%.1f%%)\n', ...
                get_metric(metrics.state_machine, 'following_time', 0), ...
                get_metric(metrics.state_machine, 'following_percentage', 0));
        fprintf('    Recovery Time:         %8.2f s (%.1f%%)\n', ...
                get_metric(metrics.state_machine, 'recovery_time', 0), ...
                get_metric(metrics.state_machine, 'recovery_percentage', 0));
        
        if verbose && isfield(metrics.state_machine, 'state_names') && isfield(metrics.state_machine, 'occupancy')
            fprintf('\n  State Occupancy:\n');
            state_names = metrics.state_machine.state_names;
            occupancy = metrics.state_machine.occupancy;
            time_in_state = get_metric(metrics.state_machine, 'time_in_state', zeros(size(occupancy)));
            for i = 1:length(state_names)
                if occupancy(i) > 0.01  % Show only states with > 0.01%
                    fprintf('    %-20s: %6.2f%% (%6.2f s)\n', state_names{i}, occupancy(i), time_in_state(i));
                end
            end
        end
        
        % Quality assessment
        trans_per_min = get_metric(metrics.state_machine, 'transitions_per_minute', 0);
        if trans_per_min < 2
            fprintf('                           ✓ Stable mission (< 2/min)\n');
        elseif trans_per_min < 5
            fprintf('                           ⚠ Moderate activity (2-5/min)\n');
        else
            fprintf('                           ⚠ High activity (> 5/min)\n');
        end
    else
        fprintf('  (State machine data not available)\n');
    end
    fprintf('\n');
    
    %% 6. Control Effort
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 6. CONTROL EFFORT                                           │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'control') && ~isempty(fieldnames(metrics.control))
        fprintf('  Total Effort:\n');
        fprintf('    RMS:                   %8.4f\n', get_metric(metrics.control, 'total_effort_rms', NaN));
        fprintf('    Mean:                  %8.4f\n', get_metric(metrics.control, 'total_effort_mean', NaN));
        fprintf('    Max:                   %8.4f\n', get_metric(metrics.control, 'total_effort_max', NaN));
        fprintf('    Energy Proxy:          %8.4f\n', get_metric(metrics.control, 'energy', NaN));
        
        if verbose
            fprintf('\n  Per-Axis RMS (Linear):\n');
            fprintf('    Surge (u):             %8.4f m/s\n', get_metric(metrics.control, 'surge_rms', NaN));
            fprintf('    Sway (v):              %8.4f m/s\n', get_metric(metrics.control, 'sway_rms', NaN));
            fprintf('    Heave (w):             %8.4f m/s\n', get_metric(metrics.control, 'heave_rms', NaN));
            
            fprintf('\n  Per-Axis RMS (Angular):\n');
            fprintf('    Roll (p):              %8.4f rad/s\n', get_metric(metrics.control, 'roll_rms', NaN));
            fprintf('    Pitch (q):             %8.4f rad/s\n', get_metric(metrics.control, 'pitch_rms', NaN));
            fprintf('    Yaw (r):               %8.4f rad/s\n', get_metric(metrics.control, 'yaw_rms', NaN));
            
            fprintf('\n  Per-Axis Max:\n');
            fprintf('    Surge Max:             %8.4f m/s\n', get_metric(metrics.control, 'surge_max', NaN));
            fprintf('    Heave Max:             %8.4f m/s\n', get_metric(metrics.control, 'heave_max', NaN));
        end
    else
        fprintf('  (Control data not available)\n');
    end
    fprintf('\n');
    
    %% 7. Geometry - Normal Vector Analysis
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 7. GEOMETRY ANALYSIS                                        │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'geometry') && ~isempty(fieldnames(metrics.geometry))
        fprintf('  Normal Parallelism ∠(n_est, n_mes):\n');
        fprintf('    Mean Angle:            %8.4f°\n', get_metric(metrics.geometry, 'normal_parallelism_mean', NaN));
        fprintf('    Std Deviation:         %8.4f°\n', get_metric(metrics.geometry, 'normal_parallelism_std', NaN));
        fprintf('    Maximum:               %8.4f°\n', get_metric(metrics.geometry, 'normal_parallelism_max', NaN));
        fprintf('    Below 5° Threshold:    %8.2f%%\n', get_metric(metrics.geometry, 'normal_parallelism_below_5deg', NaN));
        
        par_mean = get_metric(metrics.geometry, 'normal_parallelism_mean', NaN);
        if ~isnan(par_mean)
            if par_mean < 5
                fprintf('                           ✓ Excellent consistency\n');
            elseif par_mean < 10
                fprintf('                           ✓ Good consistency\n');
            else
                fprintf('                           ⚠ Check EKF/sensor alignment\n');
            end
        end
        
        fprintf('\n  Robot-Terrain Alignment ∠(z_robot, n_est):\n');
        fprintf('    Mean Angle:            %8.4f°\n', get_metric(metrics.geometry, 'robot_alignment_mean', NaN));
        fprintf('    Std Deviation:         %8.4f°\n', get_metric(metrics.geometry, 'robot_alignment_std', NaN));
        fprintf('    Maximum:               %8.4f°\n', get_metric(metrics.geometry, 'robot_alignment_max', NaN));
        
        align_mean = get_metric(metrics.geometry, 'robot_alignment_mean', NaN);
        if ~isnan(align_mean)
            if align_mean < 10
                fprintf('                           ✓ Well aligned\n');
            elseif align_mean < 20
                fprintf('                           ⚠ Moderate alignment\n');
            else
                fprintf('                           ⚠ Poor alignment - check control\n');
            end
        end
    else
        fprintf('  (Geometry data not available)\n');
    end
    fprintf('\n');
    
    %% 8. EKF Position Filter Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 8. EKF POSITION FILTER (vs Ground Truth)                    │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'ekf_position') && ~isempty(fieldnames(metrics.ekf_position)) && ...
       ~isnan(get_metric(metrics.ekf_position, 'pos_rmse_total', NaN))
        
        fprintf('  Position Estimation RMSE:\n');
        fprintf('    X (North):             %8.4f m\n', get_metric(metrics.ekf_position, 'pos_rmse_x', NaN));
        fprintf('    Y (East):              %8.4f m\n', get_metric(metrics.ekf_position, 'pos_rmse_y', NaN));
        fprintf('    Z (Down):              %8.4f m\n', get_metric(metrics.ekf_position, 'pos_rmse_z', NaN));
        fprintf('    Total:                 %8.4f m\n', get_metric(metrics.ekf_position, 'pos_rmse_total', NaN));
        fprintf('    Max Error:             %8.4f m\n', get_metric(metrics.ekf_position, 'pos_max_err', NaN));
        
        fprintf('\n  Orientation Estimation RMSE:\n');
        fprintf('    Roll:                  %8.4f°\n', get_metric(metrics.ekf_position, 'ang_rmse_roll', NaN));
        fprintf('    Pitch:                 %8.4f°\n', get_metric(metrics.ekf_position, 'ang_rmse_pitch', NaN));
        fprintf('    Yaw:                   %8.4f°\n', get_metric(metrics.ekf_position, 'ang_rmse_yaw', NaN));
        fprintf('    Max Error:             %8.4f°\n', get_metric(metrics.ekf_position, 'ang_max_err', NaN));
        
        if verbose && isfield(metrics.ekf_position, 'vel_rmse_surge')
            fprintf('\n  Linear Velocity RMSE:\n');
            fprintf('    Surge:                 %8.4f m/s\n', get_metric(metrics.ekf_position, 'vel_rmse_surge', NaN));
            fprintf('    Sway:                  %8.4f m/s\n', get_metric(metrics.ekf_position, 'vel_rmse_sway', NaN));
            fprintf('    Heave:                 %8.4f m/s\n', get_metric(metrics.ekf_position, 'vel_rmse_heave', NaN));
            
            fprintf('\n  Angular Rate RMSE:\n');
            fprintf('    p (roll rate):         %8.4f°/s\n', get_metric(metrics.ekf_position, 'rate_rmse_p', NaN));
            fprintf('    q (pitch rate):        %8.4f°/s\n', get_metric(metrics.ekf_position, 'rate_rmse_q', NaN));
            fprintf('    r (yaw rate):          %8.4f°/s\n', get_metric(metrics.ekf_position, 'rate_rmse_r', NaN));
        end
        
        if isfield(metrics.ekf_position, 'bias_x_final')
            fprintf('\n  Final Gyro Bias Estimates:\n');
            fprintf('    Bias X:                %8.4f°/s\n', get_metric(metrics.ekf_position, 'bias_x_final', NaN));
            fprintf('    Bias Y:                %8.4f°/s\n', get_metric(metrics.ekf_position, 'bias_y_final', NaN));
            fprintf('    Bias Z:                %8.4f°/s\n', get_metric(metrics.ekf_position, 'bias_z_final', NaN));
        end
        
        % Quality assessment
        pos_total = get_metric(metrics.ekf_position, 'pos_rmse_total', NaN);
        if pos_total < 0.1
            fprintf('\n                           ✓ Excellent position accuracy\n');
        elseif pos_total < 0.5
            fprintf('\n                           ✓ Good position accuracy\n');
        else
            fprintf('\n                           ⚠ Check position filter tuning\n');
        end
    else
        fprintf('  (EKF Position Filter data not available or no ground truth)\n');
    end
    fprintf('\n');
    
    %% Summary Assessment
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('                      OVERALL ASSESSMENT                       \n');
    fprintf('═══════════════════════════════════════════════════════════════\n\n');
    
    if isfield(metrics, 'overall')
        % Display grade
        fprintf('  OVERALL SCORE:           %5.1f / 100\n', metrics.overall.score);
        fprintf('  GRADE:                   %s\n', metrics.overall.grade);
        fprintf('\n');
        
        % Score breakdown
        fprintf('  Score Breakdown:\n');
        fprintf('    Altitude Tracking:     25 pts (RMS < 0.2m = 25)\n');
        fprintf('    Following Time:        25 pts (100%% = 25)\n');
        fprintf('    Sensor Reliability:    20 pts (< 5%% failures = 20)\n');
        fprintf('    Angle Tracking:        15 pts (< 3° combined = 15)\n');
        fprintf('    Control Efficiency:    15 pts (low effort = 15)\n');
    end
    
    fprintf('\n═══════════════════════════════════════════════════════════════\n\n');
end

%% Helper function to safely get metric value
function value = get_metric(s, field, default)
    if isfield(s, field) && ~isempty(s.(field))
        value = s.(field);
    else
        value = default;
    end
end
