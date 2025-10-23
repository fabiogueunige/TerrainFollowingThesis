%% ANALYZE_STATISTICS - Perform statistical analysis on simulation runs
%
% Analyzes multiple simulation runs to compute statistics on tracking
% performance, control effort, and sensor accuracy.
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
%           .angle_tracking     - Terrain angle tracking statistics
%           .control_effort     - Control signal statistics
%           .sensor_performance - Sensor accuracy metrics
%           .runs               - Per-run results
%
% METRICS COMPUTED:
%   - Mean/RMS/Max altitude error
%   - Mean/RMS/Max angle errors (alpha, beta)
%   - Control signal statistics (mean, std, max)
%   - Innovation statistics (consistency check)
%   - State estimation error statistics
%
% EXAMPLE:
%   % Analyze all runs
%   stats = analyze_statistics();
%   
%   % Analyze specific runs
%   stats = analyze_statistics({'run1', 'run2', 'run3'});
%   
%   % Display results
%   fprintf('Mean altitude error: %.4f m\n', stats.altitude_tracking.mean_error);
%
% See also: load_simulation_data, save_simulation_data

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
    
    fprintf('\n=== STATISTICAL ANALYSIS ===\n');
    fprintf('Analyzing %d simulation runs...\n\n', num_runs);
    
    %% Initialize Statistics Structure
    stats = struct();
    stats.num_runs = num_runs;
    stats.run_names = run_names;
    stats.runs = cell(num_runs, 1);
    
    %% Analyze Each Run
    for i = 1:num_runs
        fprintf('Processing run %d/%d: %s\n', i, num_runs, run_names{i});
        
        try
            % Load data
            sim_data = load_simulation_data(run_names{i});
            
            % Compute metrics for this run
            run_stats = compute_run_metrics(sim_data);
            stats.runs{i} = run_stats;
            
        catch ME
            warning('Failed to analyze run %s: %s', run_names{i}, ME.message);
            stats.runs{i} = [];
        end
    end
    
    %% Aggregate Statistics Across All Runs
    fprintf('\nComputing aggregate statistics...\n');
    
    % Filter successful runs
    valid_runs = cellfun(@(x) ~isempty(x), stats.runs);
    valid_stats = stats.runs(valid_runs);
    
    if isempty(valid_stats)
        error('No valid runs to analyze');
    end
    
    %% Altitude Tracking Statistics
    alt_errors = cellfun(@(x) x.altitude.mean_error, valid_stats);
    alt_rms = cellfun(@(x) x.altitude.rms_error, valid_stats);
    alt_max = cellfun(@(x) x.altitude.max_error, valid_stats);
    
    stats.altitude_tracking.mean_error = mean(alt_errors);
    stats.altitude_tracking.std_error = std(alt_errors);
    stats.altitude_tracking.mean_rms = mean(alt_rms);
    stats.altitude_tracking.mean_max = mean(alt_max);
    stats.altitude_tracking.all_mean_errors = alt_errors;
    
    %% Angle Tracking Statistics
    alpha_errors = cellfun(@(x) x.alpha.mean_error, valid_stats);
    beta_errors = cellfun(@(x) x.beta.mean_error, valid_stats);
    
    stats.angle_tracking.alpha_mean = mean(alpha_errors);
    stats.angle_tracking.alpha_std = std(alpha_errors);
    stats.angle_tracking.beta_mean = mean(beta_errors);
    stats.angle_tracking.beta_std = std(beta_errors);
    
    %% Control Effort Statistics
    surge_effort = cellfun(@(x) x.control.surge_rms, valid_stats);
    heave_effort = cellfun(@(x) x.control.heave_rms, valid_stats);
    
    stats.control_effort.surge_mean = mean(surge_effort);
    stats.control_effort.surge_std = std(surge_effort);
    stats.control_effort.heave_mean = mean(heave_effort);
    stats.control_effort.heave_std = std(heave_effort);
    
    %% Display Summary
    fprintf('\n=== ANALYSIS RESULTS ===\n');
    fprintf('Successful runs: %d/%d\n\n', sum(valid_runs), num_runs);
    
    fprintf('--- ALTITUDE TRACKING ---\n');
    fprintf('Mean error: %.4f ± %.4f m\n', stats.altitude_tracking.mean_error, stats.altitude_tracking.std_error);
    fprintf('Mean RMS: %.4f m\n', stats.altitude_tracking.mean_rms);
    fprintf('Mean Max: %.4f m\n\n', stats.altitude_tracking.mean_max);
    
    fprintf('--- ANGLE TRACKING ---\n');
    fprintf('Alpha error: %.4f ± %.4f deg\n', rad2deg(stats.angle_tracking.alpha_mean), rad2deg(stats.angle_tracking.alpha_std));
    fprintf('Beta error: %.4f ± %.4f deg\n\n', rad2deg(stats.angle_tracking.beta_mean), rad2deg(stats.angle_tracking.beta_std));
    
    fprintf('--- CONTROL EFFORT ---\n');
    fprintf('Surge RMS: %.4f ± %.4f m/s\n', stats.control_effort.surge_mean, stats.control_effort.surge_std);
    fprintf('Heave RMS: %.4f ± %.4f m/s\n\n', stats.control_effort.heave_mean, stats.control_effort.heave_std);
    
    fprintf('Analysis complete!\n\n');
end

%% Helper Function: Compute Metrics for Single Run
function run_stats = compute_run_metrics(sim_data)
    % Altitude tracking
    h_error = sim_data.h_ref - sim_data.x_est(1,:);
    run_stats.altitude.mean_error = mean(abs(h_error));
    run_stats.altitude.rms_error = rms(h_error);
    run_stats.altitude.max_error = max(abs(h_error));
    run_stats.altitude.std_error = std(h_error);
    
    % Alpha tracking
    alpha_error = sim_data.x_true(2,:) - sim_data.x_est(2,:);
    run_stats.alpha.mean_error = mean(abs(alpha_error));
    run_stats.alpha.rms_error = rms(alpha_error);
    run_stats.alpha.max_error = max(abs(alpha_error));
    
    % Beta tracking
    beta_error = sim_data.x_true(3,:) - sim_data.x_est(3,:);
    run_stats.beta.mean_error = mean(abs(beta_error));
    run_stats.beta.rms_error = rms(beta_error);
    run_stats.beta.max_error = max(abs(beta_error));
    
    % Control effort
    run_stats.control.surge_rms = rms(sim_data.u(1,:));
    run_stats.control.sway_rms = rms(sim_data.u(2,:));
    run_stats.control.heave_rms = rms(sim_data.u(3,:));
    run_stats.control.roll_rms = rms(sim_data.u(4,:));
    run_stats.control.pitch_rms = rms(sim_data.u(5,:));
    run_stats.control.yaw_rms = rms(sim_data.u(6,:));
    
    % Innovation statistics (consistency check)
    run_stats.innovation.mean_norm = mean(sqrt(sum(sim_data.ni.^2, 1)));
    run_stats.innovation.max_norm = max(sqrt(sum(sim_data.ni.^2, 1)));
end
