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
% METRICS ANALYZED (Comprehensive Terrain Following):
%   1.  RMS altitude error [m]
%   2.  |φ-α| angle tracking [deg]
%   3.  |θ-β| angle tracking [deg]
%   4.  EKF SBES α RMSE [deg]
%   5.  EKF SBES β RMSE [deg]
%   6.  EKF Position RMSE [m]
%   7.  Sensor failure rate [%]
%   8.  State transitions [/min]
%   9.  Following percentage [%]
%   10. Control effort RMS
%   11. Normal parallelism [deg]
%   12. Robot alignment [deg]
%   13. Overall score [/100]
%
% STATISTICAL OUTPUTS:
%   - Mean and standard deviation
%   - Variance and covariance matrices
%   - Correlation coefficients (Pearson)
%   - 95% and 99% confidence intervals
%   - Distribution parameters (skewness, kurtosis)
%
% BACKWARD COMPATIBILITY:
%   Uses load_simulation_data and compute_performance_metrics with
%   backward compatibility for old data formats.
%
% EXAMPLES:
%   % Analyze all runs
%   stats = batch_statistical_analysis();
%   
%   % Analyze specific runs
%   stats = batch_statistical_analysis({'run_20251201_105813', 'run_20251204_002618'});
%   
%   % Access results
%   fprintf('Mean altitude error: %.4f ± %.4f m\n', ...
%           stats.summary.altitude_rms.mean, ...
%           stats.summary.altitude_rms.std);
%   
%   % Check correlation between altitude and control effort
%   corr_coef = stats.correlation.matrix(1,10);
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
    fprintf('║    BATCH STATISTICAL ANALYSIS - TERRAIN FOLLOWING 6DOF    ║\n');
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
            % Load simulation data with backward compatibility
            sim_data = load_simulation_data(run_names{i});
            
            % Compute performance metrics using comprehensive function
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
    
    % Define metric extraction - ALL important metrics
    metric_names = {
        'altitude_rms', 'phi_alpha_deg', 'theta_beta_deg', ...
        'ekf_sbes_alpha_rmse', 'ekf_sbes_beta_rmse', 'ekf_pos_rmse', ...
        'sensor_failure_rate', 'transitions_per_min', 'following_pct', ...
        'control_effort_rms', 'normal_parallelism', 'robot_alignment', ...
        'overall_score'
    };
    
    metric_units = {
        'm', 'deg', 'deg', ...
        'deg', 'deg', 'm', ...
        '%', '/min', '%', ...
        '-', 'deg', 'deg', ...
        '/100'
    };
    
    num_metrics = length(metric_names);
    data_matrix = zeros(num_valid, num_metrics);
    
    for i = 1:num_valid
        m = metrics_list{i};
        
        % Extract each metric (with error handling)
        data_matrix(i, 1) = safe_extract(m, 'altitude', 'rms_error');
        data_matrix(i, 2) = rad2deg(safe_extract(m, 'angle_tracking', 'phi_alpha_mean'));
        data_matrix(i, 3) = rad2deg(safe_extract(m, 'angle_tracking', 'theta_beta_mean'));
        data_matrix(i, 4) = rad2deg(safe_extract(m, 'ekf_sbes', 'alpha_rmse'));
        data_matrix(i, 5) = rad2deg(safe_extract(m, 'ekf_sbes', 'beta_rmse'));
        data_matrix(i, 6) = safe_extract(m, 'ekf_position', 'pos_rmse_total');
        data_matrix(i, 7) = safe_extract(m, 'sensors', 'failure_rate');
        data_matrix(i, 8) = safe_extract(m, 'state_machine', 'transitions_per_minute');
        data_matrix(i, 9) = safe_extract(m, 'state_machine', 'following_percentage');
        data_matrix(i, 10) = safe_extract(m, 'control', 'total_effort_rms');
        data_matrix(i, 11) = safe_extract(m, 'geometry', 'normal_parallelism_mean');
        data_matrix(i, 12) = safe_extract(m, 'geometry', 'robot_alignment_mean');
        data_matrix(i, 13) = safe_extract(m, 'overall', 'score');
    end
    
    fprintf('  Extracted %d metrics × %d runs matrix\n\n', num_metrics, num_valid);
    
    %% Step 3: Compute Statistical Summaries
    fprintf('[3/5] Computing statistical summaries...\n');
    
    stats = struct();
    stats.metadata.num_runs = num_valid;
    stats.metadata.run_names = valid_runs;
    stats.metadata.analysis_date = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    stats.metadata.metric_names = metric_names;
    stats.metadata.metric_units = metric_units;
    
    % Summary statistics
    stats.summary = struct();
    for i = 1:num_metrics
        name = metric_names{i};
        data = data_matrix(:, i);
        valid_data = data(~isnan(data));
        
        if isempty(valid_data)
            stats.summary.(name).mean = NaN;
            stats.summary.(name).std = NaN;
            stats.summary.(name).variance = NaN;
            stats.summary.(name).min = NaN;
            stats.summary.(name).max = NaN;
            stats.summary.(name).median = NaN;
            stats.summary.(name).range = NaN;
            stats.summary.(name).valid_count = 0;
        else
            stats.summary.(name).mean = mean(valid_data);
            stats.summary.(name).std = std(valid_data);
            stats.summary.(name).variance = var(valid_data);
            stats.summary.(name).min = min(valid_data);
            stats.summary.(name).max = max(valid_data);
            stats.summary.(name).median = median(valid_data);
            stats.summary.(name).range = max(valid_data) - min(valid_data);
            stats.summary.(name).valid_count = length(valid_data);
        end
    end
    
    fprintf('  Computed mean, std, variance, min, max, median\n\n');
    
    %% Step 4: Covariance and Correlation Analysis
    fprintf('[4/5] Computing covariance and correlation...\n');
    
    if num_valid >= 2
        % Remove columns with all NaN for covariance computation
        valid_cols = all(~isnan(data_matrix), 1);
        data_clean = data_matrix(:, valid_cols);
        clean_names = metric_names(valid_cols);
        
        if size(data_clean, 2) >= 2
            % Covariance matrix
            stats.covariance.matrix = cov(data_clean);
            stats.covariance.metric_names = clean_names;
            
            % Correlation matrix (Pearson)
            stats.correlation.matrix = corrcoef(data_clean);
            stats.correlation.metric_names = clean_names;
            
            fprintf('  Computed %d×%d covariance matrix\n', size(data_clean, 2), size(data_clean, 2));
            fprintf('  Computed %d×%d correlation matrix\n\n', size(data_clean, 2), size(data_clean, 2));
        else
            fprintf('  ⚠ Insufficient valid data for covariance\n\n');
            stats.covariance = [];
            stats.correlation = [];
        end
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
        valid_data = data(~isnan(data));
        n = length(valid_data);
        
        if n >= 2
            % Mean and standard error
            mu = mean(valid_data);
            se = std(valid_data) / sqrt(n);
            
            % T-distribution critical values (appropriate for small samples)
            t_95 = tinv(0.975, n-1);  % 95% CI
            t_99 = tinv(0.995, n-1);  % 99% CI
            
            stats.confidence.(name).ci_95 = [mu - t_95*se, mu + t_95*se];
            stats.confidence.(name).ci_99 = [mu - t_99*se, mu + t_99*se];
            stats.confidence.(name).se = se;
        elseif n == 1
            mu = valid_data(1);
            stats.confidence.(name).ci_95 = [mu, mu];
            stats.confidence.(name).ci_99 = [mu, mu];
            stats.confidence.(name).se = 0;
        else
            stats.confidence.(name).ci_95 = [NaN, NaN];
            stats.confidence.(name).ci_99 = [NaN, NaN];
            stats.confidence.(name).se = NaN;
        end
    end
    
    fprintf('  Computed 95%% and 99%% confidence intervals\n\n');
    
    %% Distribution Analysis
    fprintf('Computing distribution parameters...\n');
    
    stats.distribution = struct();
    
    for i = 1:num_metrics
        name = metric_names{i};
        data = data_matrix(:, i);
        valid_data = data(~isnan(data));
        
        if length(valid_data) >= 3
            % Skewness and kurtosis
            stats.distribution.(name).skewness = skewness(valid_data);
            stats.distribution.(name).kurtosis = kurtosis(valid_data);
            
            % Interpretation
            if abs(stats.distribution.(name).skewness) < 0.5
                stats.distribution.(name).skew_interpretation = 'approximately symmetric';
            elseif stats.distribution.(name).skewness > 0
                stats.distribution.(name).skew_interpretation = 'right-skewed';
            else
                stats.distribution.(name).skew_interpretation = 'left-skewed';
            end
            
            % Kurtosis interpretation (3 = normal)
            if abs(stats.distribution.(name).kurtosis - 3) < 1
                stats.distribution.(name).kurtosis_interpretation = 'approximately normal';
            elseif stats.distribution.(name).kurtosis > 3
                stats.distribution.(name).kurtosis_interpretation = 'heavy tails (leptokurtic)';
            else
                stats.distribution.(name).kurtosis_interpretation = 'light tails (platykurtic)';
            end
        else
            stats.distribution.(name).skewness = NaN;
            stats.distribution.(name).kurtosis = NaN;
            stats.distribution.(name).skew_interpretation = 'insufficient data';
            stats.distribution.(name).kurtosis_interpretation = 'insufficient data';
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
    
    %% Generate Plots
    fprintf('\nGenerating statistical plots...\n');
    generate_batch_plots(stats, data_matrix);
    fprintf('Plots generated successfully!\n\n');
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
    fprintf('┌────────────────────────────┬───────────┬───────────┬───────────┬─────────┐\n');
    fprintf('│ Metric                     │   Mean    │    Std    │  95%% CI   │  Unit   │\n');
    fprintf('├────────────────────────────┼───────────┼───────────┼───────────┼─────────┤\n');
    
    names = stats.metadata.metric_names;
    units = stats.metadata.metric_units;
    
    for i = 1:length(names)
        name = names{i};
        unit = units{i};
        
        % Format display name
        display_name = strrep(name, '_', ' ');
        display_name = [upper(display_name(1)), display_name(2:end)];
        if length(display_name) > 24
            display_name = display_name(1:24);
        end
        
        % Get values
        mu = stats.summary.(name).mean;
        sigma = stats.summary.(name).std;
        ci = stats.confidence.(name).ci_95;
        ci_width = (ci(2) - ci(1)) / 2;
        
        % Format based on magnitude
        if isnan(mu)
            fmt = '│ %-26s │    N/A    │    N/A    │    N/A    │ %-7s │\n';
            fprintf(fmt, display_name, unit);
        elseif abs(mu) < 0.01 || (abs(mu) > 0 && abs(mu) < 0.001)
            fmt = '│ %-26s │ %9.5f │ %9.5f │ ±%8.5f │ %-7s │\n';
            fprintf(fmt, display_name, mu, sigma, ci_width, unit);
        elseif abs(mu) < 1
            fmt = '│ %-26s │ %9.4f │ %9.4f │ ±%8.4f │ %-7s │\n';
            fprintf(fmt, display_name, mu, sigma, ci_width, unit);
        elseif abs(mu) < 100
            fmt = '│ %-26s │ %9.3f │ %9.3f │ ±%8.3f │ %-7s │\n';
            fprintf(fmt, display_name, mu, sigma, ci_width, unit);
        else
            fmt = '│ %-26s │ %9.2f │ %9.2f │ ±%8.2f │ %-7s │\n';
            fprintf(fmt, display_name, mu, sigma, ci_width, unit);
        end
    end
    
    fprintf('└────────────────────────────┴───────────┴───────────┴───────────┴─────────┘\n\n');
    
    % Correlation highlights
    if ~isempty(stats.correlation) && isfield(stats.correlation, 'matrix')
        fprintf('─── NOTABLE CORRELATIONS ───\n');
        
        corr_mat = stats.correlation.matrix;
        corr_names = stats.correlation.metric_names;
        threshold = 0.6;
        
        found_any = false;
        for i = 1:size(corr_mat, 1)
            for j = i+1:size(corr_mat, 2)
                if abs(corr_mat(i,j)) >= threshold && ~isnan(corr_mat(i,j))
                    strength = '';
                    if abs(corr_mat(i,j)) >= 0.8
                        strength = ' (strong)';
                    elseif abs(corr_mat(i,j)) >= 0.6
                        strength = ' (moderate)';
                    end
                    
                    sign_str = '';
                    if corr_mat(i,j) > 0
                        sign_str = '+';
                    else
                        sign_str = '-';
                    end
                    
                    fprintf('  %s%s %s ↔ %s: r = %+.3f%s\n', ...
                            sign_str, sign_str, corr_names{i}, corr_names{j}, ...
                            corr_mat(i,j), strength);
                    found_any = true;
                end
            end
        end
        
        if ~found_any
            fprintf('  (No correlations |r| ≥ %.1f found)\n', threshold);
        end
        
        fprintf('\n');
    end
    
    % Distribution insights
    fprintf('─── DISTRIBUTION INSIGHTS ───\n');
    for i = 1:length(names)
        name = names{i};
        if isfield(stats.distribution, name) && ...
           ~strcmp(stats.distribution.(name).skew_interpretation, 'insufficient data')
            
            display_name = strrep(name, '_', ' ');
            skew = stats.distribution.(name).skewness;
            kurt = stats.distribution.(name).kurtosis;
            
            if abs(skew) > 1.0 || abs(kurt - 3) > 2
                fprintf('  %s: skew=%.2f (%s), kurtosis=%.2f (%s)\n', ...
                    display_name, skew, stats.distribution.(name).skew_interpretation, ...
                    kurt, stats.distribution.(name).kurtosis_interpretation);
            end
        end
    end
    fprintf('\n');
    
    fprintf('═══════════════════════════════════════════════════════════════\n');
    fprintf('For detailed analysis, access stats structure fields:\n');
    fprintf('  stats.summary       - Mean, std, variance, min, max\n');
    fprintf('  stats.covariance    - Covariance matrix between metrics\n');
    fprintf('  stats.correlation   - Pearson correlation coefficients\n');
    fprintf('  stats.confidence    - 95%% and 99%% confidence intervals\n');
    fprintf('  stats.distribution  - Skewness, kurtosis, interpretation\n');
    fprintf('  stats.data_matrix   - Raw data matrix [runs × metrics]\n');
    fprintf('═══════════════════════════════════════════════════════════════\n\n');
end

%% Helper: Generate Batch Statistical Plots
function generate_batch_plots(stats, data_matrix)
    
    names = stats.metadata.metric_names;
    units = stats.metadata.metric_units;
    num_metrics = length(names);
    
    %% Figure 1: Bar chart of means with error bars
    fig1 = figure('Name', 'Batch Analysis: Metric Means', 'Tag', 'batch_metric_means', 'NumberTitle', 'off');
    
    means = zeros(1, num_metrics);
    stds = zeros(1, num_metrics);
    for i = 1:num_metrics
        means(i) = stats.summary.(names{i}).mean;
        stds(i) = stats.summary.(names{i}).std;
    end
    
    % Normalize for visualization (different units)
    max_vals = max(abs(data_matrix), [], 1);
    max_vals(max_vals == 0) = 1;
    normalized_means = means ./ max_vals;
    normalized_stds = stds ./ max_vals;
    
    bar(normalized_means);
    hold on;
    errorbar(1:num_metrics, normalized_means, normalized_stds, 'k.', 'LineWidth', 1.5);
    
    display_names = cellfun(@(x) strrep(x, '_', ' '), names, 'UniformOutput', false);
    xticks(1:num_metrics);
    xticklabels(display_names);
    xtickangle(45);
    ylabel('Normalized Value');
    title('Metric Means with Standard Deviation (Normalized)');
    grid on;
    hold off;
    
    %% Figure 2: Box plots of key metrics
    fig2 = figure('Name', 'Batch Analysis: Distribution Box Plots', 'Tag', 'batch_boxplots', 'NumberTitle', 'off');
    
    key_metrics = [1, 2, 3, 7, 9, 13];  % altitude, phi-alpha, theta-beta, failure, following, score
    subplot_data = data_matrix(:, key_metrics);
    key_names = names(key_metrics);
    key_units = units(key_metrics);
    
    for i = 1:length(key_metrics)
        subplot(2, 3, i);
        boxplot(subplot_data(:, i));
        display_name = strrep(key_names{i}, '_', ' ');
        title(display_name);
        ylabel(key_units{i});
        grid on;
    end
    sgtitle('Distribution of Key Metrics');
    
    %% Figure 3: Correlation heatmap
    if ~isempty(stats.correlation) && isfield(stats.correlation, 'matrix')
        fig3 = figure('Name', 'Batch Analysis: Correlation Matrix', 'Tag', 'batch_correlation', 'NumberTitle', 'off');
        
        corr_mat = stats.correlation.matrix;
        corr_names = stats.correlation.metric_names;
        
        imagesc(corr_mat);
        colorbar;
        colormap(bluewhitered_map());
        caxis([-1 1]);
        
        display_names = cellfun(@(x) strrep(x, '_', ' '), corr_names, 'UniformOutput', false);
        xticks(1:length(corr_names));
        xticklabels(display_names);
        xtickangle(45);
        yticks(1:length(corr_names));
        yticklabels(display_names);
        
        title('Metric Correlation Matrix');
        
        % Add correlation values as text
        for i = 1:size(corr_mat, 1)
            for j = 1:size(corr_mat, 2)
                if ~isnan(corr_mat(i,j))
                    text(j, i, sprintf('%.2f', corr_mat(i,j)), ...
                        'HorizontalAlignment', 'center', ...
                        'FontSize', 8);
                end
            end
        end
    end
    
    %% Figure 4: Overall score distribution
    fig4 = figure('Name', 'Batch Analysis: Overall Score Distribution', 'Tag', 'batch_score_dist', 'NumberTitle', 'off');
    
    scores = data_matrix(:, end);  % Overall score is last column
    valid_scores = scores(~isnan(scores));
    
    if ~isempty(valid_scores)
        subplot(1, 2, 1);
        histogram(valid_scores, 'BinWidth', 5, 'FaceColor', [0.3 0.6 0.9]);
        xlabel('Overall Score');
        ylabel('Count');
        title('Score Distribution');
        grid on;
        
        % Add grade boundaries
        hold on;
        xline(90, 'g--', 'A', 'LineWidth', 2);
        xline(80, 'b--', 'B', 'LineWidth', 2);
        xline(70, 'c--', 'C', 'LineWidth', 2);
        xline(60, 'm--', 'D', 'LineWidth', 2);
        hold off;
        
        subplot(1, 2, 2);
        % Score vs run index
        plot(1:length(scores), scores, 'b-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
        xlabel('Run Index');
        ylabel('Overall Score');
        title('Score Progression');
        grid on;
        yline(mean(valid_scores), 'r--', sprintf('Mean = %.1f', mean(valid_scores)), 'LineWidth', 2);
    end
    
    sgtitle('Overall Performance Score Analysis');
end

%% Helper: Blue-White-Red colormap for correlation
function cmap = bluewhitered_map()
    n = 256;
    cmap = zeros(n, 3);
    
    % Blue to white (first half)
    for i = 1:n/2
        t = (i-1) / (n/2);
        cmap(i, :) = [t, t, 1];  % Blue to white
    end
    
    % White to red (second half)
    for i = n/2+1:n
        t = (i - n/2 - 1) / (n/2);
        cmap(i, :) = [1, 1-t, 1-t];  % White to red
    end
end
