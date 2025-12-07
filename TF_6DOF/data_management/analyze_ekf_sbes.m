function analyze_ekf_sbes(run_name)
% ANALYZE_EKF_SBES - Comprehensive EKF SBES Consistency Analysis
%
% Performs NEES (Normalized Estimation Error Squared) and Innovation analysis
% for the EKF SBES terrain estimator (altitude h, terrain angles alpha, beta).
%
% SYNTAX:
%   analyze_ekf_sbes()              % Analyze most recent run
%   analyze_ekf_sbes('run_name')    % Analyze specific run
%
% ANALYSIS PERFORMED:
%   1. NEES Analysis - Filter consistency check (should follow chi-squared)
%   2. Innovation Analysis - Measurement residuals (should be white noise)
%   3. Covariance Evolution - Check for divergence or collapse
%   4. Estimation Accuracy - RMSE vs ground truth
%   5. State Tracking Performance - h, alpha, beta tracking
%
% OUTPUTS:
%   - Console statistics report
%   - Individual figures saved as PNG (use exportgraphics)
%
% For a CONSISTENT estimator:
%   - NEES should be within chi-squared bounds (95% confidence)
%   - Innovation should be zero-mean white noise
%   - Covariance should not grow unbounded or collapse
%
% See also: compute_performance_metrics, analyze_single_run

    clc; close all;
    
    %% Plot Settings (consistent for thesis)
    lw = 1.5;           % LineWidth
    fs = 12;            % FontSize  
    fn = 'Times New Roman'; % FontName
    
    fprintf('=================================================================\n');
    fprintf('       EKF SBES CONSISTENCY ANALYSIS (NEES & Innovation)\n');
    fprintf('=================================================================\n\n');
    
    %% Load Data
    if nargin < 1 || isempty(run_name)
        % Find most recent run
        base_dir = 'results';
        runs = dir(fullfile(base_dir, 'run_*'));
        if isempty(runs)
            error('No simulation runs found in results/');
        end
        [~, idx] = sort([runs.datenum], 'descend');
        run_name = runs(idx(1)).name;
        fprintf('Analyzing most recent run: %s\n\n', run_name);
    else
        fprintf('Analyzing run: %s\n\n', run_name);
    end
    
    % Load simulation data
    sim_data = load_simulation_data(run_name);
    
    % Extract required variables
    time = sim_data.time;
    N = sim_data.N;
    Ts = sim_data.Ts;
    
    x_est = sim_data.x_est;      % EKF estimated states [h; alpha; beta]
    x_true = sim_data.x_true;    % True states [h; alpha; beta]
    ni = sim_data.ni;            % Innovation (measurement residual)
    
    % Covariance history P_sbes [3x3xN] for NEES analysis
    if isfield(sim_data, 'P_sbes') && ~isempty(sim_data.P_sbes) && ndims(sim_data.P_sbes) == 3
        P = sim_data.P_sbes;     % Full covariance history
        has_covariance = true;
        fprintf('Using P_sbes covariance history [3x3x%d] for NEES analysis.\n\n', size(P, 3));
    elseif isfield(sim_data, 'P_final') && ~isempty(sim_data.P_final)
        % Only final covariance available - limited NEES
        P = sim_data.P_final;
        has_covariance = false;  % Set false because we can't do full NEES without history
        fprintf('WARNING: Only P_final available. NEES analysis limited.\n');
        fprintf('         Re-run simulation with updated main_6DOF_3D.m for full NEES.\n\n');
    else
        has_covariance = false;
        fprintf('WARNING: Covariance data (P_sbes) not available. NEES analysis disabled.\n');
        fprintf('         Re-run simulation with updated main_6DOF_3D.m to enable NEES.\n\n');
    end
    
    % Innovation covariance S (if available)
    if isfield(sim_data, 'S') && ~isempty(sim_data.S)
        S = sim_data.S;
        has_S = true;
    else
        has_S = false;
    end
    
    % Robot angles for comparison
    rob_rot = sim_data.rob_rot;  % [roll; pitch; yaw] from EKF position
    
    % State dimension
    n_states = 3;  % [h, alpha, beta]
    n_meas = size(ni, 1);  % Number of SBES sensors (typically 4)
    
    % Confidence level for statistical tests
    alpha_conf = 0.05;  % 95% confidence
    
    %% ========================================================================
    %% 1. ESTIMATION ACCURACY (RMSE)
    %% ========================================================================
    fprintf('--- ESTIMATION ACCURACY (RMSE vs Ground Truth) ---\n');
    
    % Skip transient (first 10%)
    transient_end = round(0.1 * N);
    steady_idx = transient_end:N;
    
    % Compute errors
    h_error = x_est(1,:) - x_true(1,:);
    alpha_error = x_est(2,:) - x_true(2,:);
    beta_error = x_est(3,:) - x_true(3,:);
    
    % Handle angle wrapping
    alpha_error = atan2(sin(alpha_error), cos(alpha_error));
    beta_error = atan2(sin(beta_error), cos(beta_error));
    
    % RMSE (steady-state)
    rmse_h = sqrt(mean(h_error(steady_idx).^2));
    rmse_alpha = rad2deg(sqrt(mean(alpha_error(steady_idx).^2)));
    rmse_beta = rad2deg(sqrt(mean(beta_error(steady_idx).^2)));
    
    fprintf('Altitude (h):    RMSE = %.4f m\n', rmse_h);
    fprintf('Alpha (terrain): RMSE = %.4f deg\n', rmse_alpha);
    fprintf('Beta (terrain):  RMSE = %.4f deg\n', rmse_beta);
    fprintf('\n');
    
    %% ========================================================================
    %% 2. NEES ANALYSIS
    %% ========================================================================
    fprintf('--- NEES ANALYSIS (Filter Consistency) ---\n');
    
    if has_covariance
        nees = zeros(1, N);
        
        for k = 1:N
            % State estimation error
            state_error = [h_error(k); alpha_error(k); beta_error(k)];
            
            % Get covariance at this timestep
            if ndims(P) == 3
                P_k = P(:,:,k);
            else
                P_k = P;  % Constant covariance
            end
            
            % Ensure P is 3x3 for SBES states
            if size(P_k, 1) > 3
                P_k = P_k(1:3, 1:3);
            end
            
            % NEES: e' * P^-1 * e
            if rcond(P_k) > 1e-12
                nees(k) = state_error' * (P_k \ state_error);
            else
                nees(k) = NaN;
            end
        end
        
        % Chi-squared bounds (95% confidence)
        dof = n_states;
        chi2_lower = chi2inv(alpha_conf/2, dof);
        chi2_upper = chi2inv(1 - alpha_conf/2, dof);
        
        % Statistics (steady-state only)
        valid_nees = nees(steady_idx);
        valid_nees = valid_nees(~isnan(valid_nees) & valid_nees < 1e6);
        
        avg_nees = mean(valid_nees);
        in_bounds = sum(valid_nees >= chi2_lower & valid_nees <= chi2_upper);
        pct_in_bounds = 100 * in_bounds / length(valid_nees);
        
        fprintf('Degrees of Freedom: %d (h, alpha, beta)\n', dof);
        fprintf('95%% Chi-squared bounds: [%.2f, %.2f]\n', chi2_lower, chi2_upper);
        fprintf('Average NEES: %.4f (expected ~%.1f for consistent estimator)\n', avg_nees, dof);
        fprintf('Percentage within 95%% bounds: %.1f%%\n', pct_in_bounds);
        
        % Consistency verdict
        if avg_nees < chi2_lower
            fprintf('⚠ WARNING: NEES too LOW - Covariance OVERESTIMATED (filter pessimistic)\n');
        elseif avg_nees > chi2_upper
            fprintf('⚠ WARNING: NEES too HIGH - Covariance UNDERESTIMATED (filter overconfident)\n');
        else
            fprintf('✓ PASS: NEES within bounds - Filter is statistically consistent\n');
        end
    else
        nees = [];
        chi2_lower = NaN;
        chi2_upper = NaN;
        fprintf('NEES analysis skipped (no covariance data)\n');
    end
    fprintf('\n');
    
    %% ========================================================================
    %% 3. INNOVATION ANALYSIS
    %% ========================================================================
    fprintf('--- INNOVATION ANALYSIS (Measurement Residuals) ---\n');
    fprintf('For consistent filter: zero-mean, white noise, low autocorrelation\n\n');
    
    % Innovation statistics per sensor
    for s = 1:n_meas
        innov_s = ni(s, steady_idx);
        innov_s = innov_s(~isnan(innov_s) & ~isinf(innov_s));
        
        if length(innov_s) > 10
            mean_innov = mean(innov_s);
            std_innov = std(innov_s);
            
            % Autocorrelation at lag 1
            if length(innov_s) > 1
                autocorr_1 = corr(innov_s(1:end-1)', innov_s(2:end)');
            else
                autocorr_1 = NaN;
            end
            
            fprintf('Sensor %d: Mean=%.4f m, Std=%.4f m, Autocorr(1)=%.3f\n', ...
                s, mean_innov, std_innov, autocorr_1);
        else
            fprintf('Sensor %d: Insufficient valid data\n', s);
        end
    end
    
    % Combined innovation norm
    innov_norm = sqrt(sum(ni.^2, 1));
    innov_norm_ss = innov_norm(steady_idx);
    innov_norm_ss = innov_norm_ss(~isnan(innov_norm_ss));
    
    fprintf('\nCombined Innovation Norm:\n');
    fprintf('  Mean = %.4f m, Max = %.4f m, Std = %.4f m\n', ...
        mean(innov_norm_ss), max(innov_norm_ss), std(innov_norm_ss));
    
    %% Normalized Innovation Squared (NIS)
    if has_S
        fprintf('\n--- NIS ANALYSIS (Normalized Innovation Squared) ---\n');
        
        nis = zeros(1, N);
        for k = 1:N
            nu_k = ni(:, k);
            if ndims(S) == 3
                S_k = S(:,:,k);
            else
                S_k = S;
            end
            
            % Remove NaN measurements
            valid = ~isnan(nu_k) & ~isinf(nu_k);
            if sum(valid) > 0 && rcond(S_k(valid,valid)) > 1e-12
                nis(k) = nu_k(valid)' * (S_k(valid,valid) \ nu_k(valid));
            else
                nis(k) = NaN;
            end
        end
        
        % NIS should follow chi-squared with dof = number of measurements
        dof_nis = n_meas;
        chi2_lower_nis = chi2inv(alpha_conf/2, dof_nis);
        chi2_upper_nis = chi2inv(1 - alpha_conf/2, dof_nis);
        
        valid_nis = nis(steady_idx);
        valid_nis = valid_nis(~isnan(valid_nis) & valid_nis < 1e6);
        
        if ~isempty(valid_nis)
            avg_nis = mean(valid_nis);
            in_bounds_nis = sum(valid_nis >= chi2_lower_nis & valid_nis <= chi2_upper_nis);
            pct_in_bounds_nis = 100 * in_bounds_nis / length(valid_nis);
            
            fprintf('Degrees of Freedom: %d (SBES sensors)\n', dof_nis);
            fprintf('95%% Chi-squared bounds: [%.2f, %.2f]\n', chi2_lower_nis, chi2_upper_nis);
            fprintf('Average NIS: %.4f (expected ~%.1f)\n', avg_nis, dof_nis);
            fprintf('Percentage within 95%% bounds: %.1f%%\n', pct_in_bounds_nis);
        end
    else
        nis = [];
    end
    fprintf('\n');
    
    %% ========================================================================
    %% 4. COVARIANCE EVOLUTION
    %% ========================================================================
    fprintf('--- COVARIANCE EVOLUTION ---\n');
    
    if has_covariance && ndims(P) == 3
        trace_P_h = zeros(1, N);
        trace_P_angles = zeros(1, N);
        
        for k = 1:N
            trace_P_h(k) = P(1,1,k);  % Variance of h
            trace_P_angles(k) = P(2,2,k) + P(3,3,k);  % Variance of alpha + beta
        end
        
        fprintf('Altitude Variance:  Initial=%.6f, Final=%.6f, Min=%.6f, Max=%.6f\n', ...
            trace_P_h(1), trace_P_h(end), min(trace_P_h), max(trace_P_h));
        fprintf('Angles Variance:    Initial=%.6f, Final=%.6f, Min=%.6f, Max=%.6f\n', ...
            trace_P_angles(1), trace_P_angles(end), min(trace_P_angles), max(trace_P_angles));
        
        % Check for divergence or collapse
        if trace_P_h(end) > 10 * trace_P_h(1)
            fprintf('⚠ WARNING: Altitude covariance growing (potential divergence)\n');
        elseif trace_P_h(end) < 0.01 * trace_P_h(1)
            fprintf('⚠ WARNING: Altitude covariance collapsed (filter overconfident)\n');
        else
            fprintf('✓ Covariance evolution appears stable\n');
        end
    else
        trace_P_h = [];
        trace_P_angles = [];
        fprintf('Covariance evolution analysis skipped (no time-varying P)\n');
    end
    fprintf('\n');
    
    %% ========================================================================
    %% 5. TERRAIN ANGLE TRACKING
    %% ========================================================================
    fprintf('--- TERRAIN ANGLE TRACKING (Robot vs Terrain) ---\n');
    
    % Robot angles from EKF position filter
    phi = rob_rot(1,:);    % Robot roll
    theta = rob_rot(2,:);  % Robot pitch
    
    % Terrain angles from EKF SBES
    alpha_est = x_est(2,:);  % Terrain alpha (relates to roll)
    beta_est = x_est(3,:);   % Terrain beta (relates to pitch)
    
    % Tracking errors
    phi_alpha_error = abs(phi - alpha_est);
    theta_beta_error = abs(theta - beta_est);
    
    fprintf('|φ - α| (Roll vs Terrain):  Mean=%.2f deg, Max=%.2f deg\n', ...
        rad2deg(mean(phi_alpha_error(steady_idx))), rad2deg(max(phi_alpha_error(steady_idx))));
    fprintf('|θ - β| (Pitch vs Terrain): Mean=%.2f deg, Max=%.2f deg\n', ...
        rad2deg(mean(theta_beta_error(steady_idx))), rad2deg(max(theta_beta_error(steady_idx))));
    fprintf('\n');
    
    %% ========================================================================
    %% PLOTTING - Each figure separate with export tag
    %% ========================================================================
    fprintf('Generating plots...\n');
    
    % Create output directory for plots
    plots_dir = fullfile('results', run_name, 'ekf_sbes_analysis');
    if ~exist(plots_dir, 'dir')
        mkdir(plots_dir);
    end
    
    %% Figure 1: Altitude Estimation
    fig1 = figure('Name', 'EKF_SBES_Altitude', 'NumberTitle', 'off');
    fig1.Tag = 'ekf_sbes_altitude';
    plot(time, x_true(1,:), 'b', 'LineWidth', lw, 'DisplayName', 'True');
    hold on;
    plot(time, x_est(1,:), 'r--', 'LineWidth', lw, 'DisplayName', 'EKF Estimate');
    if isfield(sim_data, 'h_ref')
        plot(time, sim_data.h_ref, 'g:', 'LineWidth', lw, 'DisplayName', 'Reference');
    end
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Altitude h [m]', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Altitude Estimation', 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig1, fullfile(plots_dir, 'ekf_sbes_altitude.png'), 'Resolution', 300);
    
    %% Figure 2: Altitude Error
    fig2 = figure('Name', 'EKF_SBES_Altitude_Error', 'NumberTitle', 'off');
    fig2.Tag = 'ekf_sbes_altitude_error';
    plot(time, h_error, 'b', 'LineWidth', lw);
    hold on;
    yline(0, 'k--', 'LineWidth', 1);
    yline(rmse_h, 'r--', 'LineWidth', 1, 'DisplayName', sprintf('RMSE = %.4f m', rmse_h));
    yline(-rmse_h, 'r--', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Altitude Error [m]', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Altitude Estimation Error', 'FontSize', fs, 'FontName', fn);
    legend('Error', '', 'RMSE bounds', 'Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig2, fullfile(plots_dir, 'ekf_sbes_altitude_error.png'), 'Resolution', 300);
    
    %% Figure 3: Alpha Estimation
    fig3 = figure('Name', 'EKF_SBES_Alpha', 'NumberTitle', 'off');
    fig3.Tag = 'ekf_sbes_alpha';
    plot(time, rad2deg(x_true(2,:)), 'b', 'LineWidth', lw, 'DisplayName', 'True Terrain α');
    hold on;
    plot(time, rad2deg(x_est(2,:)), 'r--', 'LineWidth', lw, 'DisplayName', 'EKF Estimate');
    plot(time, rad2deg(phi), 'g:', 'LineWidth', lw, 'DisplayName', 'Robot Roll φ');
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Angle [deg]', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Terrain Angle Alpha vs Robot Roll', 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig3, fullfile(plots_dir, 'ekf_sbes_alpha.png'), 'Resolution', 300);
    
    %% Figure 4: Beta Estimation
    fig4 = figure('Name', 'EKF_SBES_Beta', 'NumberTitle', 'off');
    fig4.Tag = 'ekf_sbes_beta';
    plot(time, rad2deg(x_true(3,:)), 'b', 'LineWidth', lw, 'DisplayName', 'True Terrain β');
    hold on;
    plot(time, rad2deg(x_est(3,:)), 'r--', 'LineWidth', lw, 'DisplayName', 'EKF Estimate');
    plot(time, rad2deg(theta), 'g:', 'LineWidth', lw, 'DisplayName', 'Robot Pitch θ');
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Angle [deg]', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Terrain Angle Beta vs Robot Pitch', 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig4, fullfile(plots_dir, 'ekf_sbes_beta.png'), 'Resolution', 300);
    
    %% Figure 5: Alpha Error
    fig5 = figure('Name', 'EKF_SBES_Alpha_Error', 'NumberTitle', 'off');
    fig5.Tag = 'ekf_sbes_alpha_error';
    plot(time, rad2deg(alpha_error), 'b', 'LineWidth', lw);
    hold on;
    yline(0, 'k--', 'LineWidth', 1);
    yline(rmse_alpha, 'r--', 'LineWidth', 1);
    yline(-rmse_alpha, 'r--', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Alpha Error [deg]', 'FontSize', fs, 'FontName', fn);
    title(sprintf('EKF SBES: Alpha Estimation Error (RMSE = %.4f deg)', rmse_alpha), 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig5, fullfile(plots_dir, 'ekf_sbes_alpha_error.png'), 'Resolution', 300);
    
    %% Figure 6: Beta Error
    fig6 = figure('Name', 'EKF_SBES_Beta_Error', 'NumberTitle', 'off');
    fig6.Tag = 'ekf_sbes_beta_error';
    plot(time, rad2deg(beta_error), 'b', 'LineWidth', lw);
    hold on;
    yline(0, 'k--', 'LineWidth', 1);
    yline(rmse_beta, 'r--', 'LineWidth', 1);
    yline(-rmse_beta, 'r--', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Beta Error [deg]', 'FontSize', fs, 'FontName', fn);
    title(sprintf('EKF SBES: Beta Estimation Error (RMSE = %.4f deg)', rmse_beta), 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig6, fullfile(plots_dir, 'ekf_sbes_beta_error.png'), 'Resolution', 300);
    
    %% Figure 7: NEES
    if ~isempty(nees)
        fig7 = figure('Name', 'EKF_SBES_NEES', 'NumberTitle', 'off');
        fig7.Tag = 'ekf_sbes_nees';
        plot(time, nees, 'b', 'LineWidth', 1);
        hold on;
        yline(chi2_lower, 'r--', 'LineWidth', lw, 'DisplayName', '95% Lower');
        yline(chi2_upper, 'r--', 'LineWidth', lw, 'DisplayName', '95% Upper');
        yline(n_states, 'g-', 'LineWidth', lw, 'DisplayName', 'Expected');
        ylim([0, min(30, max(nees(steady_idx))*1.2)]);
        xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
        ylabel('NEES', 'FontSize', fs, 'FontName', fn);
        title('EKF SBES: Normalized Estimation Error Squared', 'FontSize', fs, 'FontName', fn);
        legend({'NEES', '95% Bounds', '', 'Expected'}, 'Location', 'best', 'FontSize', fs, 'FontName', fn);
        grid on;
        set(gca, 'FontSize', fs, 'FontName', fn);
        exportgraphics(fig7, fullfile(plots_dir, 'ekf_sbes_nees.png'), 'Resolution', 300);
    end
    
    %% Figure 8: Innovation per Sensor
    fig8 = figure('Name', 'EKF_SBES_Innovation', 'NumberTitle', 'off');
    fig8.Tag = 'ekf_sbes_innovation';
    colors = lines(n_meas);
    for s = 1:n_meas
        plot(time, ni(s,:), 'LineWidth', 1, 'Color', colors(s,:), ...
            'DisplayName', sprintf('Sensor %d', s));
        hold on;
    end
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Innovation [m]', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Measurement Innovation (Residuals)', 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig8, fullfile(plots_dir, 'ekf_sbes_innovation.png'), 'Resolution', 300);
    
    %% Figure 9: Innovation Norm
    fig9 = figure('Name', 'EKF_SBES_Innovation_Norm', 'NumberTitle', 'off');
    fig9.Tag = 'ekf_sbes_innovation_norm';
    plot(time, innov_norm, 'b', 'LineWidth', lw);
    hold on;
    yline(mean(innov_norm_ss), 'r--', 'LineWidth', lw, ...
        'DisplayName', sprintf('Mean = %.3f m', mean(innov_norm_ss)));
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('||ν|| [m]', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Innovation Norm', 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig9, fullfile(plots_dir, 'ekf_sbes_innovation_norm.png'), 'Resolution', 300);
    
    %% Figure 10: Innovation Histogram (combined)
    fig10 = figure('Name', 'EKF_SBES_Innovation_Histogram', 'NumberTitle', 'off');
    fig10.Tag = 'ekf_sbes_innovation_histogram';
    all_innov = ni(:, steady_idx);
    all_innov = all_innov(~isnan(all_innov) & ~isinf(all_innov));
    histogram(all_innov, 50, 'Normalization', 'pdf', 'FaceColor', [0.3 0.6 0.9]);
    hold on;
    % Gaussian fit
    x_fit = linspace(min(all_innov), max(all_innov), 100);
    y_fit = normpdf(x_fit, mean(all_innov), std(all_innov));
    plot(x_fit, y_fit, 'r-', 'LineWidth', lw, 'DisplayName', 'Gaussian Fit');
    xlabel('Innovation [m]', 'FontSize', fs, 'FontName', fn);
    ylabel('Probability Density', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Innovation Distribution', 'FontSize', fs, 'FontName', fn);
    legend({'Histogram', 'Gaussian Fit'}, 'Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig10, fullfile(plots_dir, 'ekf_sbes_innovation_histogram.png'), 'Resolution', 300);
    
    %% Figure 11: Innovation Autocorrelation
    fig11 = figure('Name', 'EKF_SBES_Innovation_Autocorr', 'NumberTitle', 'off');
    fig11.Tag = 'ekf_sbes_innovation_autocorr';
    % Use combined innovation norm for autocorrelation
    innov_centered = innov_norm(steady_idx) - mean(innov_norm(steady_idx));
    innov_centered = innov_centered(~isnan(innov_centered));
    [acf, lags] = xcorr(innov_centered, min(100, floor(length(innov_centered)/10)), 'normalized');
    stem(lags, acf, 'b', 'LineWidth', 1, 'MarkerSize', 3);
    hold on;
    conf_bound = 1.96 / sqrt(length(innov_centered));
    yline(conf_bound, 'r--', 'LineWidth', lw);
    yline(-conf_bound, 'r--', 'LineWidth', lw);
    xlabel('Lag', 'FontSize', fs, 'FontName', fn);
    ylabel('Autocorrelation', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Innovation Autocorrelation (should be ~0 for white noise)', 'FontSize', fs, 'FontName', fn);
    legend({'ACF', '95% Confidence'}, 'Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig11, fullfile(plots_dir, 'ekf_sbes_innovation_autocorr.png'), 'Resolution', 300);
    
    %% Figure 12: Covariance Evolution
    if ~isempty(trace_P_h)
        fig12 = figure('Name', 'EKF_SBES_Covariance', 'NumberTitle', 'off');
        fig12.Tag = 'ekf_sbes_covariance';
        semilogy(time, trace_P_h, 'b', 'LineWidth', lw, 'DisplayName', 'σ²_h (altitude)');
        hold on;
        semilogy(time, trace_P_angles, 'r', 'LineWidth', lw, 'DisplayName', 'σ²_α + σ²_β (angles)');
        xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
        ylabel('Variance [log scale]', 'FontSize', fs, 'FontName', fn);
        title('EKF SBES: Covariance Evolution', 'FontSize', fs, 'FontName', fn);
        legend('Location', 'best', 'FontSize', fs, 'FontName', fn);
        grid on;
        set(gca, 'FontSize', fs, 'FontName', fn);
        exportgraphics(fig12, fullfile(plots_dir, 'ekf_sbes_covariance.png'), 'Resolution', 300);
    end
    
    %% Figure 13: NIS (if available)
    if ~isempty(nis)
        fig13 = figure('Name', 'EKF_SBES_NIS', 'NumberTitle', 'off');
        fig13.Tag = 'ekf_sbes_nis';
        plot(time, nis, 'b', 'LineWidth', 1);
        hold on;
        yline(chi2_lower_nis, 'r--', 'LineWidth', lw);
        yline(chi2_upper_nis, 'r--', 'LineWidth', lw);
        yline(n_meas, 'g-', 'LineWidth', lw);
        ylim([0, min(50, max(valid_nis)*1.2)]);
        xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
        ylabel('NIS', 'FontSize', fs, 'FontName', fn);
        title('EKF SBES: Normalized Innovation Squared', 'FontSize', fs, 'FontName', fn);
        legend({'NIS', '95% Bounds', '', 'Expected'}, 'Location', 'best', 'FontSize', fs, 'FontName', fn);
        grid on;
        set(gca, 'FontSize', fs, 'FontName', fn);
        exportgraphics(fig13, fullfile(plots_dir, 'ekf_sbes_nis.png'), 'Resolution', 300);
    end
    
    %% Figure 14: Angle Tracking Error
    fig14 = figure('Name', 'EKF_SBES_Tracking_Error', 'NumberTitle', 'off');
    fig14.Tag = 'ekf_sbes_tracking_error';
    plot(time, rad2deg(phi_alpha_error), 'b', 'LineWidth', lw, 'DisplayName', '|φ - α|');
    hold on;
    plot(time, rad2deg(theta_beta_error), 'r', 'LineWidth', lw, 'DisplayName', '|θ - β|');
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Tracking Error [deg]', 'FontSize', fs, 'FontName', fn);
    title('EKF SBES: Robot-Terrain Angle Tracking Error', 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    exportgraphics(fig14, fullfile(plots_dir, 'ekf_sbes_tracking_error.png'), 'Resolution', 300);
    
    %% Summary
    fprintf('\n=================================================================\n');
    fprintf('                    ANALYSIS COMPLETE\n');
    fprintf('=================================================================\n');
    fprintf('Plots saved to: %s\n', plots_dir);
    fprintf('\nTo export all figures manually:\n');
    fprintf('  figs = findall(0, ''Type'', ''figure'');\n');
    fprintf('  for i = 1:length(figs)\n');
    fprintf('      exportgraphics(figs(i), [figs(i).Tag ''.png''], ''Resolution'', 300);\n');
    fprintf('  end\n');
    fprintf('=================================================================\n\n');
end
