%% DISPLAY_PERFORMANCE_METRICS - Display comprehensive performance metrics
%
% Formats and displays all performance metrics computed by
% compute_performance_metrics in a structured, readable format following
% RESULTS_GUIDE_2.pdf specifications.
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
    fprintf('           PERFORMANCE METRICS (RESULTS_GUIDE_2.pdf)          \n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('\n');
    
    %% 1. Altitude Tracking Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 1. ALTITUDE TRACKING PERFORMANCE                            │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'altitude_tracking')
        fprintf('  RMS Altitude Error:      %8.4f m\n', metrics.altitude_tracking);
    end
    
    if isfield(metrics, 'altitude_errors')
        fprintf('  MAE (Mean Abs Error):    %8.4f m\n', metrics.altitude_errors.mae);
        fprintf('  RMSE:                    %8.4f m\n', metrics.altitude_errors.rmse);
        fprintf('  Maximum Error:           %8.4f m\n', metrics.altitude_errors.max);
        fprintf('  Standard Deviation:      %8.4f m\n', metrics.altitude_errors.std);
    end
    fprintf('\n');
    
    %% 2. Angle Tracking Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 2. ANGLE TRACKING PERFORMANCE                               │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'angle_tracking')
        fprintf('  Mean |φ - α| Error:      %8.4f° (± %.4f°)\n', ...
                rad2deg(metrics.angle_tracking.phi_alpha_mean), ...
                rad2deg(metrics.angle_tracking.phi_alpha_std));
        fprintf('  Mean |θ - β| Error:      %8.4f° (± %.4f°)\n', ...
                rad2deg(metrics.angle_tracking.theta_beta_mean), ...
                rad2deg(metrics.angle_tracking.theta_beta_std));
        fprintf('  Combined Mean Error:     %8.4f°\n', ...
                rad2deg(metrics.angle_tracking.combined_mean));
    end
    
    if verbose && isfield(metrics, 'alpha_errors')
        fprintf('\n  Alpha (α) Estimation Errors:\n');
        fprintf('    MAE:                   %8.4f° (%.6f rad)\n', ...
                rad2deg(metrics.alpha_errors.mae), metrics.alpha_errors.mae);
        fprintf('    RMSE:                  %8.4f°\n', rad2deg(metrics.alpha_errors.rmse));
        fprintf('    Maximum:               %8.4f°\n', rad2deg(metrics.alpha_errors.max));
    end
    
    if verbose && isfield(metrics, 'beta_errors')
        fprintf('\n  Beta (β) Estimation Errors:\n');
        fprintf('    MAE:                   %8.4f° (%.6f rad)\n', ...
                rad2deg(metrics.beta_errors.mae), metrics.beta_errors.mae);
        fprintf('    RMSE:                  %8.4f°\n', rad2deg(metrics.beta_errors.rmse));
        fprintf('    Maximum:               %8.4f°\n', rad2deg(metrics.beta_errors.max));
    end
    fprintf('\n');
    
    %% 3. Sensor Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 3. SENSOR PERFORMANCE                                       │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'sensor_failure_rate')
        fprintf('  Sensor Failure Rate:     %8.2f%% of time\n', metrics.sensor_failure_rate);
        
        % Interpret failure rate
        if metrics.sensor_failure_rate < 5
            fprintf('                           ✓ Excellent (< 5%%)\n');
        elseif metrics.sensor_failure_rate < 10
            fprintf('                           ⚠ Good (5-10%%)\n');
        elseif metrics.sensor_failure_rate < 20
            fprintf('                           ⚠ Acceptable (10-20%%)\n');
        else
            fprintf('                           ✗ High (> 20%%) - Check terrain\n');
        end
    end
    fprintf('\n');
    
    %% 4. State Machine Performance
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 4. STATE MACHINE PERFORMANCE                                │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'state_transitions')
        fprintf('  Total Transitions:       %8d\n', metrics.state_transitions.total);
        fprintf('  Transitions/Minute:      %8.2f\n', metrics.state_transitions.per_minute);
        fprintf('  Mission Duration:        %8.2f min\n', metrics.state_transitions.duration_min);
        
        % Interpret transition frequency
        if metrics.state_transitions.per_minute < 2
            fprintf('                           ✓ Stable mission (< 2/min)\n');
        elseif metrics.state_transitions.per_minute < 5
            fprintf('                           ⚠ Moderate activity (2-5/min)\n');
        else
            fprintf('                           ⚠ High activity (> 5/min)\n');
        end
    end
    
    if verbose && isfield(metrics, 'state_occupancy')
        fprintf('\n  State Occupancy:\n');
        fields = fieldnames(metrics.state_occupancy);
        for i = 1:length(fields)
            state_name = fields{i};
            state_pct = metrics.state_occupancy.(state_name);
            fprintf('    %s: %6.2f%%\n', state_name, state_pct);
        end
    end
    fprintf('\n');
    
    %% 5. Control Effort
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 5. CONTROL EFFORT                                           │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'control_effort')
        fprintf('  Mean ||u|| (L2 norm):    %8.4f\n', metrics.control_effort.mean);
        fprintf('  Std Deviation:           %8.4f\n', metrics.control_effort.std);
        fprintf('  Maximum:                 %8.4f\n', metrics.control_effort.max);
        
        if verbose
            fprintf('\n  Per-Axis RMS:\n');
            fprintf('    Surge (u):             %8.4f m/s\n', metrics.control_effort.surge);
            fprintf('    Sway (v):              %8.4f m/s\n', metrics.control_effort.sway);
            fprintf('    Heave (w):             %8.4f m/s\n', metrics.control_effort.heave);
            fprintf('    Roll (p):              %8.4f rad/s\n', metrics.control_effort.roll);
            fprintf('    Pitch (q):             %8.4f rad/s\n', metrics.control_effort.pitch);
            fprintf('    Yaw (r):               %8.4f rad/s\n', metrics.control_effort.yaw);
        end
    end
    fprintf('\n');
    
    %% 6. EKF Innovation Statistics
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 6. EKF INNOVATION STATISTICS                                │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'innovation_max')
        fprintf('  Maximum ||ν||_∞:         %8.4f\n', metrics.innovation_max);
        fprintf('  Mean ||ν||:              %8.4f\n', metrics.innovation_mean);
        fprintf('  Std Deviation:           %8.4f\n', metrics.innovation_std);
    end
    fprintf('\n');
    
    %% 7. Normal Vector Analysis
    fprintf('┌─────────────────────────────────────────────────────────────┐\n');
    fprintf('│ 7. NORMAL VECTOR ANALYSIS                                   │\n');
    fprintf('└─────────────────────────────────────────────────────────────┘\n');
    
    if isfield(metrics, 'normal_parallelism') && ~isempty(fieldnames(metrics.normal_parallelism))
        fprintf('  Parallelism ∠(n_est, n_mes):\n');
        fprintf('    Mean Angle:            %8.4f°\n', metrics.normal_parallelism.mean);
        fprintf('    Std Deviation:         %8.4f°\n', metrics.normal_parallelism.std);
        fprintf('    Maximum:               %8.4f°\n', metrics.normal_parallelism.max);
        fprintf('    Below 5° Threshold:    %8.2f%%\n', metrics.normal_parallelism.below_5deg);
        
        if metrics.normal_parallelism.mean < 5
            fprintf('                           ✓ Excellent consistency\n');
        elseif metrics.normal_parallelism.mean < 10
            fprintf('                           ⚠ Good consistency\n');
        else
            fprintf('                           ⚠ Check EKF/sensor alignment\n');
        end
    end
    
    if isfield(metrics, 'robot_alignment') && ~isempty(fieldnames(metrics.robot_alignment))
        fprintf('\n  Robot-Terrain Alignment ∠(z_robot, n_est):\n');
        fprintf('    Mean Angle:            %8.4f°\n', metrics.robot_alignment.mean);
        fprintf('    Std Deviation:         %8.4f°\n', metrics.robot_alignment.std);
        fprintf('    Maximum:               %8.4f°\n', metrics.robot_alignment.max);
        
        if metrics.robot_alignment.mean < 10
            fprintf('                           ✓ Well aligned\n');
        elseif metrics.robot_alignment.mean < 20
            fprintf('                           ⚠ Moderate alignment\n');
        else
            fprintf('                           ⚠ Poor alignment - check control\n');
        end
    end
    fprintf('\n');
    
    %% Summary Assessment
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('                         SUMMARY ASSESSMENT                    \n');
    fprintf('═══════════════════════════════════════════════════════════════\n');
    
    % Overall performance rating
    score = 0;
    max_score = 0;
    
    % Altitude tracking (20 points)
    if isfield(metrics, 'altitude_tracking')
        max_score = max_score + 20;
        if metrics.altitude_tracking < 0.3
            score = score + 20;
            fprintf('  ✓ Altitude Tracking:     Excellent (< 0.3 m)\n');
        elseif metrics.altitude_tracking < 0.5
            score = score + 15;
            fprintf('  ✓ Altitude Tracking:     Good (< 0.5 m)\n');
        elseif metrics.altitude_tracking < 1.0
            score = score + 10;
            fprintf('  ⚠ Altitude Tracking:     Acceptable (< 1.0 m)\n');
        else
            fprintf('  ✗ Altitude Tracking:     Poor (≥ 1.0 m)\n');
        end
    end
    
    % Sensor reliability (20 points)
    if isfield(metrics, 'sensor_failure_rate')
        max_score = max_score + 20;
        if metrics.sensor_failure_rate < 5
            score = score + 20;
            fprintf('  ✓ Sensor Reliability:    Excellent (< 5%% failure)\n');
        elseif metrics.sensor_failure_rate < 10
            score = score + 15;
            fprintf('  ✓ Sensor Reliability:    Good (< 10%% failure)\n');
        elseif metrics.sensor_failure_rate < 20
            score = score + 10;
            fprintf('  ⚠ Sensor Reliability:    Acceptable (< 20%% failure)\n');
        else
            fprintf('  ✗ Sensor Reliability:    Poor (≥ 20%% failure)\n');
        end
    end
    
    % State machine stability (20 points)
    if isfield(metrics, 'state_transitions')
        max_score = max_score + 20;
        if metrics.state_transitions.per_minute < 2
            score = score + 20;
            fprintf('  ✓ Mission Stability:     Excellent (< 2 trans/min)\n');
        elseif metrics.state_transitions.per_minute < 5
            score = score + 15;
            fprintf('  ✓ Mission Stability:     Good (< 5 trans/min)\n');
        else
            score = score + 10;
            fprintf('  ⚠ Mission Stability:     Moderate (≥ 5 trans/min)\n');
        end
    end
    
    % Normal consistency (20 points)
    if isfield(metrics, 'normal_parallelism') && isfield(metrics.normal_parallelism, 'mean')
        max_score = max_score + 20;
        if metrics.normal_parallelism.mean < 5
            score = score + 20;
            fprintf('  ✓ Normal Consistency:    Excellent (< 5°)\n');
        elseif metrics.normal_parallelism.mean < 10
            score = score + 15;
            fprintf('  ✓ Normal Consistency:    Good (< 10°)\n');
        else
            score = score + 10;
            fprintf('  ⚠ Normal Consistency:    Moderate (≥ 10°)\n');
        end
    end
    
    % Angle tracking (20 points)
    if isfield(metrics, 'angle_tracking')
        max_score = max_score + 20;
        combined_deg = rad2deg(metrics.angle_tracking.combined_mean);
        if combined_deg < 3
            score = score + 20;
            fprintf('  ✓ Angle Tracking:        Excellent (< 3°)\n');
        elseif combined_deg < 5
            score = score + 15;
            fprintf('  ✓ Angle Tracking:        Good (< 5°)\n');
        elseif combined_deg < 10
            score = score + 10;
            fprintf('  ⚠ Angle Tracking:        Acceptable (< 10°)\n');
        else
            fprintf('  ✗ Angle Tracking:        Poor (≥ 10°)\n');
        end
    end
    
    % Overall score
    if max_score > 0
        overall_pct = (score / max_score) * 100;
        fprintf('\n');
        fprintf('  Overall Performance:     %5.1f/100 ', overall_pct);
        
        if overall_pct >= 90
            fprintf('(A - Excellent)\n');
        elseif overall_pct >= 80
            fprintf('(B - Good)\n');
        elseif overall_pct >= 70
            fprintf('(C - Acceptable)\n');
        elseif overall_pct >= 60
            fprintf('(D - Marginal)\n');
        else
            fprintf('(F - Needs Improvement)\n');
        end
    end
    
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('\n');
    
end
