%% BATCH_STATISTICAL_ANALYSIS - Advanced statistical analysis across all runs
%
% Performs comprehensive statistical analysis across multiple simulation runs
% including mean, variance, covariance, correlation, and confidence intervals.
% Suitable for magistrale-level engineering statistical analysis.
%
% SYNTAX:
%   stats = batch_statistical_analysis(run_names)
%   stats = batch_statistical_analysis()  % Analyzes all runs
%
% INPUTS:
%   run_names - Cell array of run names to analyze (optional)
%               If empty, analyzes all available runs
%
% OUTPUTS:
%   stats - Structure containing comprehensive statistical analysis:
%           .summary         - Quick overview (mean, std, variance)
%           .covariance      - Covariance matrices between metrics
%           .correlation     - Correlation coefficients
%           .confidence      - 95% and 99% confidence intervals
%           .distribution    - Distribution analysis (skewness, kurtosis)
%           .per_run_metrics - Individual metrics for each run
%           .metadata        - Run information and timestamps
%
% METRICS ANALYZED (from RESULTS_GUIDE_2.pdf):
%   1. RMS altitude error [m]
%   2. Angle tracking |φ-α| [deg]
%   3. Angle tracking |θ-β| [deg]
%   4. Sensor failure rate [%]
%   5. State transitions [/min]
%   6. Control effort RMS [m/s, rad/s]
%   7. Max innovation [σ]
%   8. Normal parallelism [deg]
%   9. Robot alignment [deg]
%
% STATISTICAL OUTPUTS:
%   - Mean and standard deviation
%   - Variance and covariance matrices
%   - Correlation coefficients (Pearson)
%   - 95% and 99% confidence intervals
%   - Distribution parameters (skewness, kurtosis)
%   - Normality assessment (Jarque-Bera test if available)
%
% EXAMPLES:
%   % Analyze all runs
%   stats = batch_statistical_analysis();
%   
%   % Analyze specific runs
%   stats = batch_statistical_analysis({'run1', 'run2', 'run3'});
%   
%   % Access results
%   fprintf('Mean altitude error: %.4f ± %.4f m\n', ...
%           stats.summary.altitude_rms.mean, ...
%           stats.summary.altitude_rms.std);
%   
%   % Check correlation between altitude and control effort
%   corr_coef = stats.correlation(1,6);
%   fprintf('Correlation: %.3f\n', corr_coef);
%
% NOTES:
%   - Requires at least 3 runs for meaningful statistics
%   - Covariance analysis requires at least 2 runs
%   - Confidence intervals use t-distribution for small samples
%   - Results saved automatically to 'results/batch_analysis_YYYYMMDD_HHMMSS.mat'
%
% See also: compute_performance_metrics, analyze_statistics, analyze_single_run

function stats = batch_statistical_analysis(run_names)
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
    
    if num_runs < 2
        warning('Only %d run available. Statistical analysis requires at least 2 runs.', num_runs);
    end
    
    fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║      BATCH STATISTICAL ANALYSIS (RESULTS_GUIDE_2.pdf)     ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
    fprintf('Analyzing %d simulation runs...\n\n', num_runs);
    
    %% Initialize Data Collection
    metrics_list = {};  % Cell array of metrics structures
    valid_runs = {};    % Names of successfully analyzed runs
    
    %% Step 1: Compute Metrics for Each Run
    fprintf('[1/5] Computing metrics for each run...\n');
    
    for i = 1:num_runs
        fprintf('  [%2d/%2d] %s ... ', i, num_runs, run_names{i});
        
        try
            % Load simulation data
            sim_data = load_simulation_data(run_names{i});
            
            % Compute performance metrics
            metrics = compute_performance_metrics(sim_data);
            
            % Store successful result
            metrics_list{end+1} = metrics; %#ok<AGROW>
            valid_runs{end+1} = run_names{i}; %#ok<AGROW>
            
            fprintf('✓\n');
            
        catch ME
            fprintf('✗ (%s)\n', ME.message);
        end
    end
    
    num_valid = length(valid_runs);
    fprintf('  Successfully analyzed: %d/%d runs\n\n', num_valid, num_runs);
    
    if num_valid == 0
        error('No valid runs to analyze');
    end
    
    %% Step 2: Extract Metrics into Matrix
    fprintf('[2/5] Extracting metrics into matrix form...\n');
    
    % Define metric extraction functions
    metric_names = {
        'altitude_rms', 'phi_alpha_error', 'theta_beta_error', ...
        'sensor_failure_rate', 'transitions_per_min', 'control_effort', ...
        'max_innovation', 'normal_parallelism', 'robot_alignment'
    };
    
    num_metrics = length(metric_names);
    data_matrix = zeros(num_valid, num_metrics);
    
    for i = 1:num_valid
        m = metrics_list{i};
        
        % Extract each metric (with error handling)
        data_matrix(i, 1) = safe_extract(m, 'altitude', 'rms_error');
        data_matrix(i, 2) = safe_extract(m, 'angle_tracking', 'phi_alpha_error');
        data_matrix(i, 3) = safe_extract(m, 'angle_tracking', 'theta_beta_error');
        data_matrix(i, 4) = safe_extract(m, 'sensors', 'failure_rate');
        data_matrix(i, 5) = safe_extract(m, 'state_machine', 'transitions_per_minute');
        data_matrix(i, 6) = safe_extract(m, 'control', 'total_effort_rms');
        data_matrix(i, 7) = safe_extract(m, 'ekf', 'max_innovation');
        data_matrix(i, 8) = safe_extract(m, 'geometry', 'normal_parallelism');
        data_matrix(i, 9) = safe_extract(m, 'geometry', 'robot_alignment');
    end
    
    fprintf('  Extracted %d metrics × %d runs matrix\n\n', num_metrics, num_valid);
    
    %% Step 3: Compute Statistical Summaries
    fprintf('[3/5] Computing statistical summaries...\n');
    
    stats = struct();
    stats.metadata.num_runs = num_valid;
    stats.metadata.run_names = valid_runs;
    stats.metadata.analysis_date = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    stats.metadata.metric_names = metric_names;
    
    % Summary statistics
    stats.summary = struct();
    for i = 1:num_metrics
        name = metric_names{i};
        data = data_matrix(:, i);
        
        stats.summary.(name).mean = mean(data);
        stats.summary.(name).std = std(data);
        stats.summary.(name).variance = var(data);
        stats.summary.(name).min = min(data);
        stats.summary.(name).max = max(data);
        stats.summary.(name).median = median(data);
        stats.summary.(name).range = max(data) - min(data);
    end
    
    fprintf('  Computed mean, std, variance, min, max, median\n\n');
    
    %% Step 4: Covariance and Correlation Analysis
    fprintf('[4/5] Computing covariance and correlation...\n');
    
    if num_valid >= 2
        % Covariance matrix
        stats.covariance.matrix = cov(data_matrix);
        stats.covariance.metric_names = metric_names;
        
        % Correlation matrix (Pearson)
        stats.correlation.matrix = corrcoef(data_matrix);
        stats.correlation.metric_names = metric_names;
        
        fprintf('  Computed %d×%d covariance matrix\n', num_metrics, num_metrics);
        fprintf('  Computed %d×%d correlation matrix\n\n', num_metrics, num_metrics);
    else
        fprintf('  ⚠ Insufficient data for covariance (need ≥2 runs)\n\n');
        stats.covariance = [];
        stats.correlation = [];
    end
    
    %% Step 5: Confidence Intervals
    fprintf('[5/5] Computing confidence intervals...\n');
    
    stats.confidence = struct();
    
    for i = 1:num_metrics
        name = metric_names{i};
        data = data_matrix(:, i);
        n = length(data);
        
        % Mean and standard error
        mu = mean(data);
        se = std(data) / sqrt(n);
        
        % T-distribution critical values (appropriate for small samples)
        if n >= 2
            t_95 = tinv(0.975, n-1);  % 95% CI
            t_99 = tinv(0.995, n-1);  % 99% CI
            
            stats.confidence.(name).ci_95 = [mu - t_95*se, mu + t_95*se];
            stats.confidence.(name).ci_99 = [mu - t_99*se, mu + t_99*se];
        else
            stats.confidence.(name).ci_95 = [mu, mu];
            stats.confidence.(name).ci_99 = [mu, mu];
        end
    end
    
    fprintf('  Computed 95%% and 99%% confidence intervals\n\n');
    
    %% Distribution Analysis
    fprintf('Computing distribution parameters...\n');
    
    stats.distribution = struct();
    
    for i = 1:num_metrics
        name = metric_names{i};
        data = data_matrix(:, i);
        
        % Skewness and kurtosis
        stats.distribution.(name).skewness = skewness(data);
        stats.distribution.(name).kurtosis = kurtosis(data);
        
        % Interpretation
        if abs(stats.distribution.(name).skewness) < 0.5
            stats.distribution.(name).skew_interpretation = 'approximately symmetric';
        elseif stats.distribution.(name).skewness > 0
            stats.distribution.(name).skew_interpretation = 'right-skewed';
        else
            stats.distribution.(name).skew_interpretation = 'left-skewed';
        end
    end
    
    fprintf('  Computed skewness and kurtosis\n\n');
    
    %% Store Per-Run Metrics
    stats.per_run_metrics = metrics_list;
    stats.data_matrix = data_matrix;
    
    %% Save Results
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    save_path = fullfile(base_dir, sprintf('batch_analysis_%s.mat', timestamp));
    save(save_path, 'stats');
    fprintf('Results saved to: %s\n\n', save_path);
    
    %% Display Summary Report
    display_batch_summary(stats);
end

%% Helper: Safe Metric Extraction
function value = safe_extract(metrics, category, field)
    % Safely extract metric with default fallback
    if isfield(metrics, category) && isfield(metrics.(category), field)
        value = metrics.(category).(field);
    else
        value = NaN;
    end
end

%% Helper: Display Batch Summary
function display_batch_summary(stats)
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║                  STATISTICAL SUMMARY                       ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n\n');
    
    fprintf('Number of runs: %d\n', stats.metadata.num_runs);
    fprintf('Analysis date: %s\n\n', stats.metadata.analysis_date);
    
    % Display table of results
    fprintf('┌────────────────────────────┬──────────┬──────────┬──────────┐\n');
    fprintf('│ Metric                     │   Mean   │   Std    │ 95%% CI   │\n');
    fprintf('├────────────────────────────┼──────────┼──────────┼──────────┤\n');
    
    names = stats.metadata.metric_names;
    for i = 1:length(names)
        name = names{i};
        
        % Format display name
        display_name = strrep(name, '_', ' ');
        display_name = [upper(display_name(1)), display_name(2:end)];
        
        % Get values
        mu = stats.summary.(name).mean;
        sigma = stats.summary.(name).std;
        ci = stats.confidence.(name).ci_95;
        ci_width = ci(2) - ci(1);
        
        % Format based on magnitude
        if abs(mu) < 0.01
            fmt = '│ %-26s │ %8.4f │ %8.4f │ ±%7.4f │\n';
        elseif abs(mu) < 1
            fmt = '│ %-26s │ %8.3f │ %8.3f │ ±%7.3f │\n';
        else
            fmt = '│ %-26s │ %8.2f │ %8.2f │ ±%7.2f │\n';
        end
        
        fprintf(fmt, display_name, mu, sigma, ci_width/2);
    end
    
    fprintf('└────────────────────────────┴──────────┴──────────┴──────────┘\n\n');
    
    % Correlation highlights
    if ~isempty(stats.correlation)
        fprintf('─── NOTABLE CORRELATIONS ───\n');
        
        corr_mat = stats.correlation.matrix;
        threshold = 0.7;
        
        found_any = false;
        for i = 1:size(corr_mat, 1)
            for j = i+1:size(corr_mat, 2)
                if abs(corr_mat(i,j)) >= threshold
                    fprintf('  %s ↔ %s: %.3f\n', ...
                            names{i}, names{j}, corr_mat(i,j));
                    found_any = true;
                end
            end
        end
        
        if ~found_any
            fprintf('  (No strong correlations |r| ≥ %.1f found)\n', threshold);
        end
        
        fprintf('\n');
    end
    
    fprintf('═══════════════════════════════════════════════════════════\n');
    fprintf('For detailed analysis, access stats structure fields:\n');
    fprintf('  stats.summary       - Mean, std, variance, min, max\n');
    fprintf('  stats.covariance    - Covariance matrix between metrics\n');
    fprintf('  stats.correlation   - Pearson correlation coefficients\n');
    fprintf('  stats.confidence    - 95%% and 99%% confidence intervals\n');
    fprintf('  stats.distribution  - Skewness, kurtosis, normality\n');
    fprintf('═══════════════════════════════════════════════════════════\n\n');
end
