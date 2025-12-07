function ekf_consistency_analysis()
    % EKF CONSISTENCY ANALYSIS
    % This script analyzes the statistical consistency of the EKF estimator
    % using NEES (Normalized Estimation Error Squared) and Innovation analysis.
    %
    % A consistent estimator should have:
    % - NEES values within chi-squared bounds
    % - Innovation (measurement residuals) that are white noise with zero mean
    % - Covariance that doesn't grow unbounded or collapse to zero
    
    clc; clear; close all;
    addpath('../for_controller');
    addpath('../rotations');
    addpath('../sensors');
    addpath('../model');
    addpath('../ekf_position');

    %% Configuration
    Ts = 0.001; 
    Tf = 50; 
    time = 0:Ts:Tf; 
    N = length(time);
    
    % Dimensions
    i_dim = 6; 
    d_dim = 3;
    ekf_dim = 15;

    fprintf('=================================================================\n');
    fprintf('       EKF CONSISTENCY ANALYSIS (NEES & Innovation)\n');
    fprintf('=================================================================\n\n');
    fprintf('NOTE: Position X,Y are NOT directly observed (no GPS).\n');
    fprintf('      They are estimated by integrating DVL velocity.\n');
    fprintf('      High autocorrelation on X,Y is EXPECTED (drift).\n');
    fprintf('      For terrain following, Z (depth) and angles matter most.\n\n');

    %% Initialization
    eta = zeros(i_dim,N); 
    eta(1:3,1) = [0; 0; 0]; 
    eta(4:6,1) = [0; 0; 0];
    
    pos_true = zeros(i_dim,N); 
    pos_true(:,1) = eta(:,1);
    rot = zeros(d_dim,d_dim,N);
    
    nu = zeros(i_dim, N); 
    u = zeros(i_dim, N);
    nu_dot = zeros(i_dim,N);
    
    % EKF Setup
    P_loc = zeros(ekf_dim,ekf_dim,N);
    [Q_loc, P_loc(:,:,1)] = stateLoc_init(ekf_dim);
    wRr = zeros(d_dim, d_dim, N);
    wRr(:,:,1) = rotz(eta(6,1)) * roty(eta(5,1)) * rotx(eta(4,1));
    x_loc = zeros(ekf_dim, N);
    rot(:,:,1) = wRr(:,:,1);
    
    % NEES storage (for position states 1:3)
    nees_pos = zeros(1, N);
    
    % Innovation storage (we'll compute it from estimation error)
    innovation = zeros(6, N);
    
    % Covariance trace storage
    trace_P_pos = zeros(1, N);
    trace_P_vel = zeros(1, N);
    trace_P_ang = zeros(1, N);
    
    % Terrain & Trajectory
    p_seafloor_NED = [-3; 0; 5];
    alpha = zeros(1,N); 
    beta = zeros(1,N);
    h = zeros(N,1);
    h_clean = zeros(N,1);
    
    for k = 1:N
        t = time(k);
        if t < 10, alpha(k)=0; elseif t<25, alpha(k)=deg2rad(20); elseif t<40, alpha(k)=deg2rad(-45); else, alpha(k)=deg2rad(15); end
        if t < 5, beta(k)=0; elseif t<15, beta(k)=deg2rad(-20); elseif t<30, beta(k)=deg2rad(50); else, beta(k)=deg2rad(20); end
    end
    
    n0 = [0; 0; 1];
    wRs = zeros(d_dim, d_dim, N);
    wRs(:,:,1) =  rotx(pi) * rotz(0) * roty(beta(1)) * rotx(alpha(1));
    w_n = wRs(:,:,1) * n0;
    h(1) = (w_n'*(eta(1:3,1) - p_seafloor_NED))/(norm(w_n));
    h_clean(1) = h(1);
    
    % Controller Parameters
    speed0 = [0.2; 0; 0; 0; 0; 0];
    u_star = 0.3; v_star = 0; h_star = 3;
    
    err = zeros(i_dim, N);
    pid = zeros(i_dim, N);
    i_err = zeros(i_dim, N);
    term_sum = zeros(i_dim, N);
    
    % Differentiated saturation limits
    max_pid_vec = [4.0; 4.0; 2.5; 2.5; 2.5; 4.0]; % [surge,sway,heave,roll,pitch,yaw]
    
    [Kp, Ki, Kd] = gainComputation(speed0, i_dim);
    
    %% Simulation Loop
    for k = 2:N
        %% Altitude contribution to errors
        h_err = (h_star - h(k-1)); 
        r_n = wRr(:,:,k-1)' * w_n;
        r_n = r_n / norm(r_n);
        h_contribution = h_err * r_n;
        
        err(1,k) = u_star - nu(1,k-1) + 0.5 * h_contribution(1);
        err(2,k) = v_star - nu(2,k-1) + 0.5 * h_contribution(2);
        err(3,k) = h_contribution(3);
        err(4,k) = alpha(k) - eta(4,k-1);
        err(5,k) = beta(k) - eta(5,k-1);
        err(6,k) = 0 - eta(6,k-1);
        
        for l = 1:i_dim
            if l == 1 || l == 2
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu_dot(l,k-1);
                d_err = 0;
            else
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu(l,k-1);
                d_err = Kd(l) * nu_dot(l,k-1);
            end
            term_sum(l,k) = i_err(l,k) - p_err - d_err;
            pid(l,k) = integrator(pid(l,k-1), term_sum(l,k), term_sum(l,k-1), Ts);
            pid(l,k) = max(min(pid(l,k), max_pid_vec(l)), -max_pid_vec(l));
        end
        
        %% Dynamic Model
        [nu_dot(:,k), u(:,k)] = dynamic_model(pid(:,k), pos_true(4:6,k-1), u(:,k-1), Ts, i_dim, nu_dot(:,k-1));
        
        %% Clean position (Ground Truth)
        T = transformationT(pos_true(4:6,k-1));
        pos_true(1:3,k) = pos_true(1:3,k-1) + rot(:,:,k-1)*u(1:3,k)*Ts;
        pos_true(4:6,k) = pos_true(4:6,k-1) + T * u(4:6,k) *Ts;
        rot(:,:,k) = rotz(pos_true(6,k)) * roty(pos_true(5,k)) * rotx(pos_true(4,k));
        eta_clean = [pos_true(1:3,k); pos_true(4:6,k)];
    
        %% EKF Localization
        [x_loc(:,k), P_loc(:,:,k), wRr(:,:,k)] = ekf_position(x_loc(:,k-1), pid(:,k), wRr(:,:,k-1), u(:,k), eta_clean, P_loc(:,:,k-1), Q_loc, Ts);
        eta(:,k) = x_loc(1:6,k);
        nu(:,k) = x_loc(7:12,k);
        
        %% Compute NEES for position (states 1:3)
        pos_error = eta(1:3,k) - pos_true(1:3,k);
        P_pos = P_loc(1:3, 1:3, k);
        
        % NEES: e' * P^-1 * e (should follow chi-squared distribution)
        if rcond(P_pos) > 1e-10
            nees_pos(k) = pos_error' * (P_pos \ pos_error);
        else
            nees_pos(k) = NaN;
        end
        
        %% Store Innovation (estimation error)
        innovation(1:3, k) = pos_error;
        innovation(4:6, k) = atan2(sin(eta(4:6,k) - pos_true(4:6,k)), cos(eta(4:6,k) - pos_true(4:6,k)));
        
        %% Store Covariance Trace
        trace_P_pos(k) = trace(P_loc(1:3, 1:3, k));
        trace_P_vel(k) = trace(P_loc(7:9, 7:9, k));
        trace_P_ang(k) = trace(P_loc(4:6, 4:6, k));
        
        %% Altitude
        wRs(:,:,k) =  rotx(pi) * rotz(0) * roty(beta(k)) * rotx(alpha(k));
        w_n = wRs(:,:,k) * n0;
        w_n_norm = norm(w_n);
        h(k) = abs((w_n'*(eta(1:3,k) - p_seafloor_NED))/w_n_norm);
        h_clean(k) = abs((w_n'*(pos_true(1:3,k) - p_seafloor_NED))/w_n_norm);
    end
    
    %% NEES Analysis
    % For a consistent estimator, NEES should follow chi-squared distribution
    % with degrees of freedom = dimension of state being tested (3 for position)
    dof = 3; % Position has 3 states
    alpha_conf = 0.05; % 95% confidence
    chi2_lower = chi2inv(alpha_conf/2, dof);
    chi2_upper = chi2inv(1 - alpha_conf/2, dof);
    
    % Average NEES (should be close to dof for consistent estimator)
    valid_nees = nees_pos(~isnan(nees_pos) & nees_pos < 1e6);
    avg_nees = mean(valid_nees(1000:end)); % Skip transient
    
    % Percentage within bounds
    in_bounds = sum(valid_nees(1000:end) >= chi2_lower & valid_nees(1000:end) <= chi2_upper);
    pct_in_bounds = 100 * in_bounds / length(valid_nees(1000:end));
    
    fprintf('--- NEES ANALYSIS (Position States) ---\n');
    fprintf('Degrees of Freedom: %d\n', dof);
    fprintf('95%% Chi-squared bounds: [%.2f, %.2f]\n', chi2_lower, chi2_upper);
    fprintf('Average NEES: %.4f (should be ~%.1f for consistent estimator)\n', avg_nees, dof);
    fprintf('Percentage within 95%% bounds: %.1f%%\n\n', pct_in_bounds);
    
    %% Innovation Analysis
    % Innovation should be white noise with zero mean
    fprintf('--- INNOVATION ANALYSIS (Estimation Error) ---\n');
    fprintf('NOTE: X,Y have high autocorr because they drift (no absolute position sensor)\n');
    labels = {'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
    units = {'m', 'm', 'm', 'rad', 'rad', 'rad'};
    observable = {'NO (drift)', 'NO (drift)', 'YES (PS)', 'YES (AHRS)', 'YES (AHRS)', 'YES (AHRS)'};
    
    for i = 1:6
        innov_i = innovation(i, 1000:end); % Skip transient
        mean_innov = mean(innov_i);
        std_innov = std(innov_i);
        
        % Autocorrelation at lag 1 (should be ~0 for white noise)
        if length(innov_i) > 1
            autocorr_1 = corr(innov_i(1:end-1)', innov_i(2:end)');
        else
            autocorr_1 = NaN;
        end
        
        fprintf('%s [%s]: Mean=%.2e %s, Std=%.2e %s, Autocorr(1)=%.3f\n', ...
            labels{i}, observable{i}, mean_innov, units{i}, std_innov, units{i}, autocorr_1);
    end
    
    %% NEES for Observable States Only (Z, Roll, Pitch, Yaw)
    fprintf('\n--- NEES ANALYSIS (Observable States: Z + Angles) ---\n');
    nees_obs = zeros(1, N);
    for k = 2:N
        obs_error = [eta(3,k) - pos_true(3,k); ...
                     atan2(sin(eta(4:6,k) - pos_true(4:6,k)), cos(eta(4:6,k) - pos_true(4:6,k)))];
        P_obs = [P_loc(3,3,k), P_loc(3,4:6,k); P_loc(4:6,3,k), P_loc(4:6,4:6,k)];
        if rcond(P_obs) > 1e-10
            nees_obs(k) = obs_error' * (P_obs \ obs_error);
        else
            nees_obs(k) = NaN;
        end
    end
    
    dof_obs = 4; % Z + 3 angles
    chi2_lower_obs = chi2inv(alpha_conf/2, dof_obs);
    chi2_upper_obs = chi2inv(1 - alpha_conf/2, dof_obs);
    valid_nees_obs = nees_obs(~isnan(nees_obs) & nees_obs < 1e6);
    avg_nees_obs = mean(valid_nees_obs(1000:end));
    in_bounds_obs = sum(valid_nees_obs(1000:end) >= chi2_lower_obs & valid_nees_obs(1000:end) <= chi2_upper_obs);
    pct_in_bounds_obs = 100 * in_bounds_obs / length(valid_nees_obs(1000:end));
    
    fprintf('Degrees of Freedom: %d (Z + Roll + Pitch + Yaw)\n', dof_obs);
    fprintf('95%% Chi-squared bounds: [%.2f, %.2f]\n', chi2_lower_obs, chi2_upper_obs);
    fprintf('Average NEES (observable): %.4f (should be ~%.1f)\n', avg_nees_obs, dof_obs);
    fprintf('Percentage within 95%% bounds: %.1f%%\n', pct_in_bounds_obs);
    
    %% Consistency Verdict
    fprintf('\n--- CONSISTENCY VERDICT ---\n');
    if avg_nees < chi2_lower
        fprintf('WARNING: NEES too low (%.2f < %.2f) - Covariance may be OVERESTIMATED\n', avg_nees, chi2_lower);
        fprintf('         The filter is pessimistic about its accuracy.\n');
    elseif avg_nees > chi2_upper
        fprintf('WARNING: NEES too high (%.2f > %.2f) - Covariance may be UNDERESTIMATED\n', avg_nees, chi2_upper);
        fprintf('         The filter is overconfident. This can lead to divergence!\n');
    else
        fprintf('PASS: NEES within bounds (%.2f in [%.2f, %.2f])\n', avg_nees, chi2_lower, chi2_upper);
        fprintf('      The EKF covariance estimate is statistically consistent.\n');
    end
    
    if pct_in_bounds < 90
        fprintf('WARNING: Only %.1f%% of NEES values within bounds (expected ~95%%)\n', pct_in_bounds);
    end
    
    %% Plotting
    lw = 1.5; fs = 12; fn = 'Times New Roman';
    
    % Plot 1: NEES over time
    figure('Name', 'NEES Analysis');
    plot(time, nees_pos, 'b', 'LineWidth', 1);
    hold on;
    yline(chi2_lower, 'r--', 'LineWidth', lw);
    yline(chi2_upper, 'r--', 'LineWidth', lw);
    yline(dof, 'g-', 'LineWidth', lw);
    ylim([0, min(20, max(nees_pos(1000:end))*1.2)]);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('NEES', 'FontSize', fs, 'FontName', fn);
    title('Normalized Estimation Error Squared (Position)', 'FontSize', fs, 'FontName', fn);
    legend({'NEES', '95% Lower Bound', '95% Upper Bound', 'Expected Value'}, ...
        'FontSize', fs, 'FontName', fn, 'Location', 'best');
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 2: Covariance Trace Evolution
    figure('Name', 'Covariance Evolution');
    semilogy(time, trace_P_pos, 'b', 'LineWidth', lw);
    hold on;
    semilogy(time, trace_P_vel, 'r', 'LineWidth', lw);
    semilogy(time, trace_P_ang, 'g', 'LineWidth', lw);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Trace(P) [log scale]', 'FontSize', fs, 'FontName', fn);
    title('Covariance Matrix Trace Evolution', 'FontSize', fs, 'FontName', fn);
    legend({'Position', 'Velocity', 'Angles'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best');
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 3: Innovation Histogram (Position Z)
    figure('Name', 'Innovation Distribution (Z)');
    histogram(innovation(3, 1000:end), 50, 'Normalization', 'pdf');
    hold on;
    % Overlay Gaussian fit
    x_fit = linspace(min(innovation(3, 1000:end)), max(innovation(3, 1000:end)), 100);
    y_fit = normpdf(x_fit, mean(innovation(3, 1000:end)), std(innovation(3, 1000:end)));
    plot(x_fit, y_fit, 'r-', 'LineWidth', lw);
    xlabel('Innovation Z [m]', 'FontSize', fs, 'FontName', fn);
    ylabel('Probability Density', 'FontSize', fs, 'FontName', fn);
    title('Innovation Distribution for Position Z', 'FontSize', fs, 'FontName', fn);
    legend({'Histogram', 'Gaussian Fit'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best');
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 4: Innovation Autocorrelation
    figure('Name', 'Innovation Autocorrelation');
    [acf, lags] = xcorr(innovation(3, 1000:end) - mean(innovation(3, 1000:end)), 100, 'normalized');
    stem(lags, acf, 'b', 'LineWidth', 1);
    hold on;
    % 95% confidence bounds for white noise
    conf_bound = 1.96 / sqrt(length(innovation(3, 1000:end)));
    yline(conf_bound, 'r--', 'LineWidth', 1);
    yline(-conf_bound, 'r--', 'LineWidth', 1);
    xlabel('Lag', 'FontSize', fs, 'FontName', fn);
    ylabel('Autocorrelation', 'FontSize', fs, 'FontName', fn);
    title('Innovation Autocorrelation (Z) - Should be Zero for White Noise', 'FontSize', fs, 'FontName', fn);
    legend({'ACF', '95% Confidence'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best');
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    fprintf('\n=================================================================\n');
    fprintf('Analysis Complete.\n');
end
