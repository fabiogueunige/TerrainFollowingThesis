%% ANALYZE_SINGLE_RUN - Complete analysis of a single simulation run
%
% Performs comprehensive analysis by computing ALL performance metrics,
% displaying formatted results, and generating ALL plots from RESULTS_GUIDE_2.pdf
%
% SYNTAX:
%   analyze_single_run(run_name)
%   analyze_single_run()  % Analyzes most recent run
%
% INPUTS:
%   run_name - Name of run to analyze (e.g., 'run_20251201_105813')
%              If empty/omitted, analyzes the most recent run
%
% WHAT IT DOES:
%   1. Loads simulation data (trajectory, EKF, control, sensors)
%   2. Computes ALL 8 performance metrics from RESULTS_GUIDE_2.pdf:
%      - RMS altitude error
%      - Angle tracking errors |φ-α| and |θ-β|
%      - Sensor failure rate
%      - State transitions per minute
%      - Control effort (6-DOF RMS)
%      - Maximum innovation
%      - Normal parallelism ∠(n_est, n_mes)
%      - Robot alignment ∠(z_robot, n_est)
%   3. Displays formatted results with quality assessment
%   4. Generates ALL plots:
%      - 3D trajectory
%      - Altitude tracking
%      - Angle tracking (roll, pitch, yaw)
%      - EKF states
%      - Control signals
%      - Sensor measurements
%      - Innovation analysis
%      - Covariance evolution
%
% OUTPUTS:
%   None (displays results and plots in console/figures)
%
% EXAMPLES:
%   % Analyze most recent run
%   analyze_single_run();
%   
%   % Analyze specific run
%   analyze_single_run('run_20251201_105813');
%
% NOTES:
%   - This function combines compute_performance_metrics, 
%     display_performance_metrics, and plot_results
%   - Provides complete analysis in one command
%   - Suitable for quick assessment and detailed report generation
%
% See also: compute_performance_metrics, display_performance_metrics,
%           plot_results, batch_statistical_analysis

function analyze_single_run(run_name)
    %% Handle Input Arguments
    if nargin < 1 || isempty(run_name)
        % Find most recent run
        base_dir = 'results';
        runs = dir(fullfile(base_dir, 'run_*'));
        
        if isempty(runs)
            error('No simulation runs found in results/');
        end
        
        % Sort by date to get most recent
        [~, idx] = sort([runs.datenum], 'descend');
        run_name = runs(idx(1)).name;
        
        fprintf('\n=== SINGLE RUN ANALYSIS ===\n');
        fprintf('No run specified, analyzing most recent: %s\n', run_name);
    else
        fprintf('\n=== SINGLE RUN ANALYSIS ===\n');
        fprintf('Analyzing run: %s\n', run_name);
    end
    
    %% STEP 1: Load Simulation Data
    fprintf('\n[1/4] Loading simulation data...\n');
    
    try
        sim_data = load_simulation_data(run_name);
        fprintf('      ✓ Loaded successfully\n');
        fprintf('      - Simulation time: %.2f s\n', sim_data.t(end));
        fprintf('      - Time steps: %d\n', length(sim_data.t));
        fprintf('      - Data fields: %d\n', length(fieldnames(sim_data)));
    catch ME
        error('Failed to load simulation data: %s', ME.message);
    end
    
    %% STEP 2: Compute Performance Metrics
    fprintf('\n[2/4] Computing performance metrics...\n');
    
    try
        metrics = compute_performance_metrics(sim_data);
        fprintf('      ✓ Computed %d metrics successfully\n', ...
                count_metrics(metrics));
    catch ME
        warning('Failed to compute metrics: %s', ME.message);
        fprintf('      ⚠ Continuing without metrics...\n');
        metrics = [];
    end
    
    %% STEP 3: Display Performance Results
    fprintf('\n[3/4] Displaying performance results...\n\n');
    
    if ~isempty(metrics)
        try
            % Display with verbose formatting
            display_performance_metrics(metrics, 'verbose');
        catch ME
            warning('Failed to display metrics: %s', ME.message);
            fprintf('      ⚠ Metrics computed but display failed\n');
        end
    else
        fprintf('      ⚠ No metrics to display\n');
    end
    
    %% STEP 4: Generate All Plots
    fprintf('\n[4/4] Generating all plots...\n');
    
    try
        % Use plot_results to generate comprehensive plots
        plot_results(sim_data);
        fprintf('      ✓ All plots generated successfully\n');
        fprintf('      - 3D trajectory\n');
        fprintf('      - Altitude tracking\n');
        fprintf('      - Angle tracking (roll, pitch, yaw)\n');
        fprintf('      - EKF state estimates\n');
        fprintf('      - Control signals (6-DOF)\n');
        fprintf('      - Sensor measurements\n');
        fprintf('      - Innovation analysis\n');
        fprintf('      - Covariance evolution\n');
    catch ME
        warning('Failed to generate plots: %s', ME.message);
        fprintf('      ⚠ Plot generation encountered errors\n');
    end
    
    %% Summary
    fprintf('\n=== ANALYSIS COMPLETE ===\n');
    fprintf('Run: %s\n', run_name);
    
    if ~isempty(metrics)
        fprintf('Overall Performance: %s (%.1f/100)\n', ...
                metrics.overall.grade, metrics.overall.score);
    end
    
    fprintf('\nAll results and plots have been generated.\n');
    fprintf('Use window controls to navigate between figures.\n\n');
end

%% Helper Function: Count Total Metrics
function n = count_metrics(metrics)
    % Count how many metrics were computed
    n = 0;
    
    if isfield(metrics, 'altitude') && isfield(metrics.altitude, 'rms_error')
        n = n + 1;
    end
    
    if isfield(metrics, 'angle_tracking') && isfield(metrics.angle_tracking, 'phi_alpha_error')
        n = n + 1;
    end
    
    if isfield(metrics, 'angle_tracking') && isfield(metrics.angle_tracking, 'theta_beta_error')
        n = n + 1;
    end
    
    if isfield(metrics, 'sensors') && isfield(metrics.sensors, 'failure_rate')
        n = n + 1;
    end
    
    if isfield(metrics, 'state_machine') && isfield(metrics.state_machine, 'transitions_per_minute')
        n = n + 1;
    end
    
    if isfield(metrics, 'control') && isfield(metrics.control, 'total_effort_rms')
        n = n + 1;
    end
    
    if isfield(metrics, 'ekf') && isfield(metrics.ekf, 'max_innovation')
        n = n + 1;
    end
    
    if isfield(metrics, 'geometry') && isfield(metrics.geometry, 'normal_parallelism')
        n = n + 1;
    end
    
    if isfield(metrics, 'geometry') && isfield(metrics.geometry, 'robot_alignment')
        n = n + 1;
    end
end
