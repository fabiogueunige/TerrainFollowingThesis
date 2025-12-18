%% EXAMPLE_BATCH_ANALYSIS - Example script for batch statistical analysis
%
% This script demonstrates how to:
% 1. Load multiple simulation runs
% 2. Perform comparative analysis
% 3. Generate comparison plots
% 4. Export results to files
%
% USAGE:
%   Run this script after collecting multiple simulation runs
addpath(genpath('.'))
clear; clc; close all;

%% Configuration
% Specify runs to analyze (leave empty to analyze all)
runs_to_analyze = {}; % Empty = all runs, or specify: {'run1', 'run2', 'run3'}

% Output directory for plots
output_dir = 'analysis_results';
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fprintf('=== BATCH ANALYSIS SCRIPT ===\n\n');

%% 1. Perform Statistical Analysis
fprintf('Step 1: Computing statistics...\n');
if isempty(runs_to_analyze)
    stats = analyze_statistics();
else
    stats = analyze_statistics(runs_to_analyze);
end

%% 2. Generate Comparison Plots

% Plot 2.1: Altitude Error Comparison
fprintf('\nStep 2: Generating comparison plots...\n');
fprintf('  - Altitude error comparison\n');

figure('Name', 'Altitude Error Comparison', 'Position', [100, 100, 1200, 400]);

subplot(1,3,1);
bar(stats.altitude_tracking.all_mean_errors);
ylabel('Mean Absolute Error [m]');
xlabel('Run Number');
title('Altitude Tracking - Mean Error');
grid on;

subplot(1,3,2);
histogram(stats.altitude_tracking.all_mean_errors, 10);
xlabel('Mean Error [m]');
ylabel('Frequency');
title('Error Distribution');
grid on;

subplot(1,3,3);
boxplot(stats.altitude_tracking.all_mean_errors);
ylabel('Mean Error [m]');
title('Error Statistics');
grid on;

saveas(gcf, fullfile(output_dir, 'altitude_error_comparison.png'));

% Plot 2.2: Control Effort Comparison
fprintf('  - Control effort comparison\n');

figure('Name', 'Control Effort Comparison', 'Position', [150, 150, 800, 600]);

valid_runs = cellfun(@(x) ~isempty(x), stats.runs);
num_valid = sum(valid_runs);

surge_rms = zeros(num_valid, 1);
heave_rms = zeros(num_valid, 1);

for i = 1:num_valid
    surge_rms(i) = stats.runs{i}.control.surge_rms;
    heave_rms(i) = stats.runs{i}.control.heave_rms;
end

subplot(2,1,1);
bar(surge_rms);
ylabel('Surge RMS [m/s]');
xlabel('Run Number');
title('Control Effort - Surge');
grid on;

subplot(2,1,2);
bar(heave_rms);
ylabel('Heave RMS [m/s]');
xlabel('Run Number');
title('Control Effort - Heave');
grid on;

saveas(gcf, fullfile(output_dir, 'control_effort_comparison.png'));

% Plot 2.3: Tracking Performance Over Time (First 3 Runs)
fprintf('  - Time-series comparison (first 3 runs)\n');

figure('Name', 'Time Series Comparison', 'Position', [200, 200, 1200, 800]);

num_plot = min(3, num_valid);
colors = {'b', 'r', 'g'};

for i = 1:num_plot
    % Load full data for time-series plot
    sim_data = load_simulation_data(stats.run_names{i});
    
    subplot(3,1,1);
    hold on;
    h_error = sim_data.h_ref - sim_data.x_est(1,:);
    plot(sim_data.time, h_error, colors{i}, 'DisplayName', stats.run_names{i});
    ylabel('Altitude Error [m]');
    title('Altitude Tracking Error Over Time');
    grid on;
    
    subplot(3,1,2);
    hold on;
    plot(sim_data.time, sim_data.u(1,:), colors{i}, 'DisplayName', stats.run_names{i});
    ylabel('Surge Velocity [m/s]');
    title('Surge Control Signal');
    grid on;
    
    subplot(3,1,3);
    hold on;
    plot(sim_data.time, sim_data.u(3,:), colors{i}, 'DisplayName', stats.run_names{i});
    ylabel('Heave Velocity [m/s]');
    xlabel('Time [s]');
    title('Heave Control Signal');
    grid on;
end

subplot(3,1,1); legend('Location', 'best');
subplot(3,1,2); legend('Location', 'best');
subplot(3,1,3); legend('Location', 'best');

saveas(gcf, fullfile(output_dir, 'timeseries_comparison.png'));

%% 3. Export Statistics to CSV

fprintf('\nStep 3: Exporting statistics to CSV...\n');

% Create summary table
summary_table = table();
summary_table.RunName = stats.run_names';

% Add altitude metrics
alt_errors = cellfun(@(x) x.altitude.mean_error, stats.runs(valid_runs));
alt_rms = cellfun(@(x) x.altitude.rms_error, stats.runs(valid_runs));
alt_max = cellfun(@(x) x.altitude.max_error, stats.runs(valid_runs));

summary_table.Altitude_Mean_Error = alt_errors;
summary_table.Altitude_RMS_Error = alt_rms;
summary_table.Altitude_Max_Error = alt_max;

% Add angle metrics
alpha_errors = cellfun(@(x) rad2deg(x.alpha.mean_error), stats.runs(valid_runs));
beta_errors = cellfun(@(x) rad2deg(x.beta.mean_error), stats.runs(valid_runs));

summary_table.Alpha_Mean_Error_deg = alpha_errors;
summary_table.Beta_Mean_Error_deg = beta_errors;

% Add control metrics
summary_table.Surge_RMS = surge_rms;
summary_table.Heave_RMS = heave_rms;

% Save to CSV
csv_file = fullfile(output_dir, 'statistics_summary.csv');
writetable(summary_table, csv_file);
fprintf('  Saved: %s\n', csv_file);

%% 4. Generate Summary Report

fprintf('\nStep 4: Generating summary report...\n');

report_file = fullfile(output_dir, 'analysis_report.txt');
fid = fopen(report_file, 'w');

fprintf(fid, '=== BATCH ANALYSIS REPORT ===\n');
fprintf(fid, 'Generated: %s\n\n', datestr(now));

fprintf(fid, '--- OVERVIEW ---\n');
fprintf(fid, 'Total runs analyzed: %d\n', num_valid);
fprintf(fid, 'Successful runs: %d\n\n', num_valid);

fprintf(fid, '--- ALTITUDE TRACKING PERFORMANCE ---\n');
fprintf(fid, 'Mean error:      %.4f ± %.4f m\n', stats.altitude_tracking.mean_error, stats.altitude_tracking.std_error);
fprintf(fid, 'Mean RMS error:  %.4f m\n', stats.altitude_tracking.mean_rms);
fprintf(fid, 'Mean max error:  %.4f m\n\n', stats.altitude_tracking.mean_max);

fprintf(fid, '--- ANGLE TRACKING PERFORMANCE ---\n');
fprintf(fid, 'Alpha error:     %.4f ± %.4f deg\n', rad2deg(stats.angle_tracking.alpha_mean), rad2deg(stats.angle_tracking.alpha_std));
fprintf(fid, 'Beta error:      %.4f ± %.4f deg\n\n', rad2deg(stats.angle_tracking.beta_mean), rad2deg(stats.angle_tracking.beta_std));

fprintf(fid, '--- CONTROL EFFORT ---\n');
fprintf(fid, 'Surge RMS:       %.4f ± %.4f m/s\n', stats.control_effort.surge_mean, stats.control_effort.surge_std);
fprintf(fid, 'Heave RMS:       %.4f ± %.4f m/s\n\n', stats.control_effort.heave_mean, stats.control_effort.heave_std);

fprintf(fid, '--- INDIVIDUAL RUN RESULTS ---\n');
for i = 1:num_valid
    fprintf(fid, '\nRun: %s\n', stats.run_names{i});
    fprintf(fid, '  Altitude error: %.4f m (RMS: %.4f m)\n', ...
        stats.runs{i}.altitude.mean_error, stats.runs{i}.altitude.rms_error);
    fprintf(fid, '  Control effort: Surge=%.4f, Heave=%.4f m/s\n', ...
        stats.runs{i}.control.surge_rms, stats.runs{i}.control.heave_rms);
end

fprintf(fid, '\n=== END OF REPORT ===\n');
fclose(fid);

fprintf('  Saved: %s\n', report_file);

%% 5. Display Summary

fprintf('\n=== ANALYSIS COMPLETE ===\n');
fprintf('Results saved to: %s/\n', output_dir);
fprintf('\nFiles generated:\n');
fprintf('  - altitude_error_comparison.png\n');
fprintf('  - control_effort_comparison.png\n');
fprintf('  - timeseries_comparison.png\n');
fprintf('  - statistics_summary.csv\n');
fprintf('  - analysis_report.txt\n');

fprintf('\n=== SUMMARY STATISTICS ===\n');
fprintf('Altitude tracking: %.4f ± %.4f m (mean ± std)\n', ...
    stats.altitude_tracking.mean_error, stats.altitude_tracking.std_error);
fprintf('Control effort:    Surge=%.4f m/s, Heave=%.4f m/s\n', ...
    stats.control_effort.surge_mean, stats.control_effort.heave_mean);

fprintf('\nAnalysis complete!\n');
