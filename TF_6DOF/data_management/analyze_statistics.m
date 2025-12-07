%% ANALYZE_STATISTICS - Comprehensive statistical analysis on simulation runs
%
% Analyzes multiple simulation runs using compute_performance_metrics to
% gather ALL relevant metrics for terrain following statistical analysis.
%
% SYNTAX:
%   stats = analyze_statistics(run_names)
%   stats = analyze_statistics()  % Analyzes all runs
%
% INPUTS:
%   run_names - Cell array of run names to analyze (optional)
%               If empty, analyzes all available runs
%
% OUTPUTS:
%   stats - Structure containing statistical metrics:
%           .altitude_tracking  - Altitude tracking statistics
%           .angle_tracking     - Terrain angle tracking (robot vs terrain)
%           .ekf_sbes           - EKF SBES estimation statistics
%           .ekf_position       - EKF Position filter statistics
%           .control_effort     - Control signal statistics (6-DOF)
%           .sensors            - Sensor failure statistics
%           .state_machine      - State occupancy statistics
%           .geometry           - Normal parallelism statistics
%           .overall            - Overall scores distribution
%           .runs               - Per-run metrics
%
% METRICS COMPUTED:
%   1. Altitude Tracking:
%      - RMS, MAE, max error, settling time
%   2. Angle Tracking:
%      - |φ-α| (roll vs terrain alpha)
%      - |θ-β| (pitch vs terrain beta)
%   3. EKF SBES Estimation:
%      - α, β estimation errors
%      - Innovation statistics
%   4. EKF Position Filter:
%      - Position RMSE (XYZ)
%      - Orientation RMSE (roll, pitch, yaw)
%      - Velocity RMSE
%   5. Control Effort:
%      - Per-axis RMS
%      - Total energy
%   6. Sensors:
%      - Failure rate
%      - Per-sensor failures
%   7. State Machine:
%      - Following percentage
%      - Transitions per minute
%   8. Geometry:
%      - Normal parallelism
%      - Robot alignment
%
% BACKWARD COMPATIBILITY:
%   Uses load_simulation_data with backward compatibility for old data formats.
%
% EXAMPLE:
%   % Analyze all runs
%   stats = analyze_statistics();
%   
%   % Analyze specific runs
%   stats = analyze_statistics({'run_20251201_105813', 'run_20251204_002618'});
%   
%   % Access results
%   fprintf('Mean altitude error: %.4f m\n', stats.altitude_tracking.mean_error);
%   fprintf('Mean following time: %.1f%%\n', stats.state_machine.following_mean);
%
% See also: load_simulation_data, compute_performance_metrics, batch_statistical_analysis

function stats = analyze_statistics(run_names)
    %% Get List of Runs to Analyze
    base_dir = 'results';
    
    if ~exist(base_dir, 'dir')
        error('No results directory found.');
    end
    
    if nargin < 1 || isempty(run_names)
        % Get all available runs
        runs = dir(fullfile(base_dir, 'run_*'));
        run_names = {runs.name};
    end
    
    num_runs = length(run_names);
    
    if num_runs == 0
        error('No runs to analyze');
    end
    
    fprintf('\n╔═══════════════════════════════════════════════════════════╗\n');
    fprintf('║            COMPREHENSIVE STATISTICAL ANALYSIS             ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════╝\n\n');
    fprintf('Analyzing %d simulation runs...\n\n', num_runs);
    
    %% Initialize Statistics Structure
    stats = struct();
    stats.num_runs = num_runs;
    stats.run_names = run_names;
    stats.runs = cell(num_runs, 1);
    stats.metrics_list = cell(num_runs, 1);
    
    %% Analyze Each Run using compute_performance_metrics
    for i = 1:num_runs
        fprintf('[%2d/%2d] Processing: %s ... ', i, num_runs, run_names{i});
        
        try
            % Load data with backward compatibility
            sim_data = load_simulation_data(run_names{i});
            
            % Compute comprehensive metrics using the updated function
            metrics = compute_performance_metrics(sim_data);
            
            stats.runs{i} = metrics;
            stats.metrics_list{i} = metrics;
            
            fprintf('✓\n');
            
        catch ME
            warning('Failed to analyze run %s: %s', run_names{i}, ME.message);
            stats.runs{i} = [];
            stats.metrics_list{i} = [];
        end
    end
    
    %% Filter Successful Runs
    valid_mask = cellfun(@(x) ~isempty(x), stats.runs);
    valid_metrics = stats.runs(valid_mask);
    valid_names = run_names(valid_mask);
    num_valid = sum(valid_mask);
    
    if num_valid == 0
        error('No valid runs to analyze');
    end
    
    fprintf('\nSuccessfully analyzed: %d/%d runs\n\n', num_valid, num_runs);
    stats.valid_runs = valid_names;
    stats.num_valid = num_valid;
    
    %% ========================================================================
    %% 1. Altitude Tracking Statistics
    %% ========================================================================
    fprintf('Aggregating altitude tracking statistics...\n');
    
    alt_rms = safe_extract_array(valid_metrics, 'altitude', 'rms_error');
    alt_mae = safe_extract_array(valid_metrics, 'altitude', 'mae');
    alt_max = safe_extract_array(valid_metrics, 'altitude', 'max_error');
    settling = safe_extract_array(valid_metrics, 'altitude', 'settling_time');
    
    stats.altitude_tracking = aggregate_stats(alt_rms, 'rms_error');
    stats.altitude_tracking = merge_structs(stats.altitude_tracking, aggregate_stats(alt_mae, 'mae'));
    stats.altitude_tracking = merge_structs(stats.altitude_tracking, aggregate_stats(alt_max, 'max_error'));
    stats.altitude_tracking = merge_structs(stats.altitude_tracking, aggregate_stats(settling, 'settling_time'));
    
    %% ========================================================================
    %% 2. Angle Tracking Statistics (Robot vs Terrain)
    %% ========================================================================
    fprintf('Aggregating angle tracking statistics...\n');
    
    phi_alpha = safe_extract_array(valid_metrics, 'angle_tracking', 'phi_alpha_mean');
    theta_beta = safe_extract_array(valid_metrics, 'angle_tracking', 'theta_beta_mean');
    combined = safe_extract_array(valid_metrics, 'angle_tracking', 'combined_mean');
    
    stats.angle_tracking = aggregate_stats(rad2deg(phi_alpha), 'phi_alpha');
    stats.angle_tracking = merge_structs(stats.angle_tracking, aggregate_stats(rad2deg(theta_beta), 'theta_beta'));
    stats.angle_tracking = merge_structs(stats.angle_tracking, aggregate_stats(rad2deg(combined), 'combined'));
    
    %% ========================================================================
    %% 3. EKF SBES Statistics
    %% ========================================================================
    fprintf('Aggregating EKF SBES statistics...\n');
    
    alpha_rmse = safe_extract_array(valid_metrics, 'ekf_sbes', 'alpha_rmse');
    beta_rmse = safe_extract_array(valid_metrics, 'ekf_sbes', 'beta_rmse');
    h_rmse = safe_extract_array(valid_metrics, 'ekf_sbes', 'h_rmse');
    innov_mean = safe_extract_array(valid_metrics, 'ekf_sbes', 'innovation_mean');
    innov_max = safe_extract_array(valid_metrics, 'ekf_sbes', 'innovation_max');
    
    stats.ekf_sbes = aggregate_stats(rad2deg(alpha_rmse), 'alpha_rmse_deg');
    stats.ekf_sbes = merge_structs(stats.ekf_sbes, aggregate_stats(rad2deg(beta_rmse), 'beta_rmse_deg'));
    stats.ekf_sbes = merge_structs(stats.ekf_sbes, aggregate_stats(h_rmse, 'h_rmse'));
    stats.ekf_sbes = merge_structs(stats.ekf_sbes, aggregate_stats(innov_mean, 'innovation_mean'));
    stats.ekf_sbes = merge_structs(stats.ekf_sbes, aggregate_stats(innov_max, 'innovation_max'));
    
    %% ========================================================================
    %% 4. EKF Position Filter Statistics
    %% ========================================================================
    fprintf('Aggregating EKF Position filter statistics...\n');
    
    pos_rmse_total = safe_extract_array(valid_metrics, 'ekf_position', 'pos_rmse_total');
    ang_rmse_roll = safe_extract_array(valid_metrics, 'ekf_position', 'ang_rmse_roll');
    ang_rmse_pitch = safe_extract_array(valid_metrics, 'ekf_position', 'ang_rmse_pitch');
    ang_rmse_yaw = safe_extract_array(valid_metrics, 'ekf_position', 'ang_rmse_yaw');
    
    stats.ekf_position = aggregate_stats(pos_rmse_total, 'pos_rmse_total');
    stats.ekf_position = merge_structs(stats.ekf_position, aggregate_stats(ang_rmse_roll, 'ang_rmse_roll'));
    stats.ekf_position = merge_structs(stats.ekf_position, aggregate_stats(ang_rmse_pitch, 'ang_rmse_pitch'));
    stats.ekf_position = merge_structs(stats.ekf_position, aggregate_stats(ang_rmse_yaw, 'ang_rmse_yaw'));
    
    %% ========================================================================
    %% 5. Control Effort Statistics
    %% ========================================================================
    fprintf('Aggregating control effort statistics...\n');
    
    surge_rms = safe_extract_array(valid_metrics, 'control', 'surge_rms');
    sway_rms = safe_extract_array(valid_metrics, 'control', 'sway_rms');
    heave_rms = safe_extract_array(valid_metrics, 'control', 'heave_rms');
    roll_rms = safe_extract_array(valid_metrics, 'control', 'roll_rms');
    pitch_rms = safe_extract_array(valid_metrics, 'control', 'pitch_rms');
    yaw_rms = safe_extract_array(valid_metrics, 'control', 'yaw_rms');
    total_rms = safe_extract_array(valid_metrics, 'control', 'total_effort_rms');
    energy = safe_extract_array(valid_metrics, 'control', 'energy');
    
    stats.control_effort = aggregate_stats(surge_rms, 'surge_rms');
    stats.control_effort = merge_structs(stats.control_effort, aggregate_stats(sway_rms, 'sway_rms'));
    stats.control_effort = merge_structs(stats.control_effort, aggregate_stats(heave_rms, 'heave_rms'));
    stats.control_effort = merge_structs(stats.control_effort, aggregate_stats(rad2deg(roll_rms), 'roll_rms_deg'));
    stats.control_effort = merge_structs(stats.control_effort, aggregate_stats(rad2deg(pitch_rms), 'pitch_rms_deg'));
    stats.control_effort = merge_structs(stats.control_effort, aggregate_stats(rad2deg(yaw_rms), 'yaw_rms_deg'));
    stats.control_effort = merge_structs(stats.control_effort, aggregate_stats(total_rms, 'total_rms'));
    stats.control_effort = merge_structs(stats.control_effort, aggregate_stats(energy, 'energy'));
    
    %% ========================================================================
    %% 6. Sensor Statistics
    %% ========================================================================
    fprintf('Aggregating sensor statistics...\n');
    
    failure_rate = safe_extract_array(valid_metrics, 'sensors', 'failure_rate');
    max_failures = safe_extract_array(valid_metrics, 'sensors', 'max_simultaneous_failures');
    
    stats.sensors = aggregate_stats(failure_rate, 'failure_rate');
    stats.sensors = merge_structs(stats.sensors, aggregate_stats(max_failures, 'max_simultaneous'));
    
    %% ========================================================================
    %% 7. State Machine Statistics
    %% ========================================================================
    fprintf('Aggregating state machine statistics...\n');
    
    transitions = safe_extract_array(valid_metrics, 'state_machine', 'total_transitions');
    trans_per_min = safe_extract_array(valid_metrics, 'state_machine', 'transitions_per_minute');
    following_pct = safe_extract_array(valid_metrics, 'state_machine', 'following_percentage');
    recovery_pct = safe_extract_array(valid_metrics, 'state_machine', 'recovery_percentage');
    
    stats.state_machine = aggregate_stats(transitions, 'total_transitions');
    stats.state_machine = merge_structs(stats.state_machine, aggregate_stats(trans_per_min, 'transitions_per_minute'));
    stats.state_machine = merge_structs(stats.state_machine, aggregate_stats(following_pct, 'following_percentage'));
    stats.state_machine = merge_structs(stats.state_machine, aggregate_stats(recovery_pct, 'recovery_percentage'));
    
    %% ========================================================================
    %% 8. Geometry Statistics
    %% ========================================================================
    fprintf('Aggregating geometry statistics...\n');
    
    normal_par = safe_extract_array(valid_metrics, 'geometry', 'normal_parallelism_mean');
    robot_align = safe_extract_array(valid_metrics, 'geometry', 'robot_alignment_mean');
    
    stats.geometry = aggregate_stats(normal_par, 'normal_parallelism');
    stats.geometry = merge_structs(stats.geometry, aggregate_stats(robot_align, 'robot_alignment'));
    
    %% ========================================================================
    %% 9. Overall Score Statistics
    %% ========================================================================
    fprintf('Aggregating overall scores...\n');
    
    overall_scores = safe_extract_array(valid_metrics, 'overall', 'score');
    stats.overall = aggregate_stats(overall_scores, 'score');
    
    % Grade distribution
    grades = {'A - Excellent', 'B - Good', 'C - Acceptable', 'D - Marginal', 'F - Needs Improvement'};
    grade_counts = zeros(1, length(grades));
    for i = 1:length(valid_metrics)
        if isfield(valid_metrics{i}, 'overall') && isfield(valid_metrics{i}.overall, 'grade')
            grade = valid_metrics{i}.overall.grade;
            for g = 1:length(grades)
                if strcmp(grade, grades{g})
                    grade_counts(g) = grade_counts(g) + 1;
                    break;
                end
            end
        end
    end
    stats.overall.grade_distribution = grade_counts;
    stats.overall.grade_names = grades;
    
    %% ========================================================================
    %% Display Summary Report
    %% ========================================================================
    display_analysis_summary(stats);
    
    fprintf('\nStatistical analysis complete!\n\n');
end

%% ============================================================================
%% Helper Functions
%% ============================================================================

function arr = safe_extract_array(metrics_cell, category, field)
    % Safely extract field from array of metrics structures
    n = length(metrics_cell);
    arr = NaN(1, n);
    
    for i = 1:n
        m = metrics_cell{i};
        if isfield(m, category) && isfield(m.(category), field)
            arr(i) = m.(category).(field);
        end
    end
end

function agg = aggregate_stats(data, prefix)
    % Compute aggregate statistics for a data array
    valid = ~isnan(data);
    data_valid = data(valid);
    
    agg = struct();
    
    if isempty(data_valid)
        agg.([prefix '_mean']) = NaN;
        agg.([prefix '_std']) = NaN;
        agg.([prefix '_min']) = NaN;
        agg.([prefix '_max']) = NaN;
        agg.([prefix '_median']) = NaN;
        agg.([prefix '_all']) = data;
        return;
    end
    
    agg.([prefix '_mean']) = mean(data_valid);
    agg.([prefix '_std']) = std(data_valid);
    agg.([prefix '_min']) = min(data_valid);
    agg.([prefix '_max']) = max(data_valid);
    agg.([prefix '_median']) = median(data_valid);
    agg.([prefix '_all']) = data;
end

function merged = merge_structs(s1, s2)
    % Merge two structures
    merged = s1;
    fields = fieldnames(s2);
    for i = 1:length(fields)
        merged.(fields{i}) = s2.(fields{i});
    end
end

function display_analysis_summary(stats)
    % Display formatted analysis summary
    
    fprintf('\n╔═══════════════════════════════════════════════════════════╗\n');
    fprintf('║                    ANALYSIS RESULTS                       ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════╝\n\n');
    
    fprintf('Successful runs: %d/%d\n\n', stats.num_valid, stats.num_runs);
    
    %% Altitude Tracking
    fprintf('─── ALTITUDE TRACKING ───\n');
    if isfield(stats.altitude_tracking, 'rms_error_mean')
        fprintf('  RMS Error:      %.4f ± %.4f m\n', ...
            stats.altitude_tracking.rms_error_mean, stats.altitude_tracking.rms_error_std);
        fprintf('  MAE:            %.4f ± %.4f m\n', ...
            stats.altitude_tracking.mae_mean, stats.altitude_tracking.mae_std);
        fprintf('  Max Error:      %.4f m (worst case)\n', stats.altitude_tracking.max_error_max);
        fprintf('  Settling Time:  %.2f ± %.2f s\n\n', ...
            stats.altitude_tracking.settling_time_mean, stats.altitude_tracking.settling_time_std);
    else
        fprintf('  (No data available)\n\n');
    end
    
    %% Angle Tracking
    fprintf('─── ANGLE TRACKING (Robot vs Terrain) ───\n');
    if isfield(stats.angle_tracking, 'phi_alpha_mean')
        fprintf('  |φ - α| (Roll):   %.2f ± %.2f deg\n', ...
            stats.angle_tracking.phi_alpha_mean, stats.angle_tracking.phi_alpha_std);
        fprintf('  |θ - β| (Pitch):  %.2f ± %.2f deg\n', ...
            stats.angle_tracking.theta_beta_mean, stats.angle_tracking.theta_beta_std);
        fprintf('  Combined:         %.2f ± %.2f deg\n\n', ...
            stats.angle_tracking.combined_mean, stats.angle_tracking.combined_std);
    else
        fprintf('  (No data available)\n\n');
    end
    
    %% EKF SBES
    fprintf('─── EKF SBES ESTIMATION ───\n');
    if isfield(stats.ekf_sbes, 'alpha_rmse_deg_mean')
        fprintf('  α RMSE:         %.3f ± %.3f deg\n', ...
            stats.ekf_sbes.alpha_rmse_deg_mean, stats.ekf_sbes.alpha_rmse_deg_std);
        fprintf('  β RMSE:         %.3f ± %.3f deg\n', ...
            stats.ekf_sbes.beta_rmse_deg_mean, stats.ekf_sbes.beta_rmse_deg_std);
        fprintf('  h RMSE:         %.4f ± %.4f m\n', ...
            stats.ekf_sbes.h_rmse_mean, stats.ekf_sbes.h_rmse_std);
        fprintf('  Innovation Mean: %.4f\n\n', stats.ekf_sbes.innovation_mean_mean);
    else
        fprintf('  (No data available)\n\n');
    end
    
    %% EKF Position
    fprintf('─── EKF POSITION FILTER ───\n');
    if isfield(stats.ekf_position, 'pos_rmse_total_mean')
        fprintf('  Position RMSE:  %.4f ± %.4f m\n', ...
            stats.ekf_position.pos_rmse_total_mean, stats.ekf_position.pos_rmse_total_std);
        fprintf('  Roll RMSE:      %.3f ± %.3f deg\n', ...
            stats.ekf_position.ang_rmse_roll_mean, stats.ekf_position.ang_rmse_roll_std);
        fprintf('  Pitch RMSE:     %.3f ± %.3f deg\n', ...
            stats.ekf_position.ang_rmse_pitch_mean, stats.ekf_position.ang_rmse_pitch_std);
        fprintf('  Yaw RMSE:       %.3f ± %.3f deg\n\n', ...
            stats.ekf_position.ang_rmse_yaw_mean, stats.ekf_position.ang_rmse_yaw_std);
    else
        fprintf('  (No data available)\n\n');
    end
    
    %% Control Effort
    fprintf('─── CONTROL EFFORT ───\n');
    if isfield(stats.control_effort, 'surge_rms_mean')
        fprintf('  Surge RMS:      %.4f ± %.4f m/s\n', ...
            stats.control_effort.surge_rms_mean, stats.control_effort.surge_rms_std);
        fprintf('  Heave RMS:      %.4f ± %.4f m/s\n', ...
            stats.control_effort.heave_rms_mean, stats.control_effort.heave_rms_std);
        fprintf('  Total RMS:      %.4f ± %.4f\n', ...
            stats.control_effort.total_rms_mean, stats.control_effort.total_rms_std);
        fprintf('  Energy:         %.2f ± %.2f J·s\n\n', ...
            stats.control_effort.energy_mean, stats.control_effort.energy_std);
    else
        fprintf('  (No data available)\n\n');
    end
    
    %% Sensors
    fprintf('─── SENSOR RELIABILITY ───\n');
    if isfield(stats.sensors, 'failure_rate_mean')
        fprintf('  Failure Rate:   %.2f ± %.2f %%\n', ...
            stats.sensors.failure_rate_mean, stats.sensors.failure_rate_std);
        fprintf('  Max Simult.:    %.0f sensors\n\n', stats.sensors.max_simultaneous_max);
    else
        fprintf('  (No data available)\n\n');
    end
    
    %% State Machine
    fprintf('─── STATE MACHINE ───\n');
    if isfield(stats.state_machine, 'following_percentage_mean')
        fprintf('  Following:      %.1f ± %.1f %%\n', ...
            stats.state_machine.following_percentage_mean, stats.state_machine.following_percentage_std);
        fprintf('  Recovery:       %.1f ± %.1f %%\n', ...
            stats.state_machine.recovery_percentage_mean, stats.state_machine.recovery_percentage_std);
        fprintf('  Trans/min:      %.2f ± %.2f\n\n', ...
            stats.state_machine.transitions_per_minute_mean, stats.state_machine.transitions_per_minute_std);
    else
        fprintf('  (No data available)\n\n');
    end
    
    %% Overall
    fprintf('─── OVERALL PERFORMANCE ───\n');
    if isfield(stats.overall, 'score_mean')
        fprintf('  Score:          %.1f ± %.1f /100\n', ...
            stats.overall.score_mean, stats.overall.score_std);
        fprintf('  Range:          [%.1f, %.1f]\n', ...
            stats.overall.score_min, stats.overall.score_max);
        
        % Grade distribution
        fprintf('\n  Grade Distribution:\n');
        for g = 1:length(stats.overall.grade_names)
            if stats.overall.grade_distribution(g) > 0
                fprintf('    %s: %d (%.1f%%)\n', ...
                    stats.overall.grade_names{g}, ...
                    stats.overall.grade_distribution(g), ...
                    stats.overall.grade_distribution(g) / stats.num_valid * 100);
            end
        end
    else
        fprintf('  (No data available)\n');
    end
    
    fprintf('\n═══════════════════════════════════════════════════════════\n');
end
