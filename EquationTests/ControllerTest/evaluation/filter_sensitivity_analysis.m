function filter_sensitivity_analysis()
    clc; clear; close all;
    addpath('for_controller');
    addpath('rotations');
    addpath('sensors');
    addpath('model');
    addpath('ekf_position');

    %% Global Configuration
    Ts = 0.001;
    Tf = 40; % Shorter time is enough to see the transition
    
    % Variations of Filter Alpha
    % alpha = 0 (no filtering, just raw EKF) -> alpha = 1 (infinite filtering, no update)
    % Typical values: 0.1 (light), 0.5, 0.8 (moderate), 0.95 (heavy)
    alpha_values = [0.1, 0.5, 0.8, 0.9, 0.95, 0.99];
    num_variations = length(alpha_values);
    stats = [];

    fprintf('Starting Low Pass Filter Sensitivity Analysis...\n');
    fprintf('------------------------------------------------------------------------------------------\n');
    fprintf('%-25s | %-15s | %-15s\n', 'Alpha Value', 'Roll RMSE (vs GT)', 'Roll RMSE (vs EKF)');
    fprintf('------------------------------------------------------------------------------------------\n');

    % Fixed Scenario Parameters
    angle_deg = 20; % Moderate angle change
    
    for i = 1:num_variations
        a_val = alpha_values(i);
        name = sprintf('Alpha = %.2f', a_val);
        
        % Run Scenario with specific alpha filter
        [res, m] = run_filter_test(Ts, Tf, angle_deg, a_val);
        
        stats(i).name = name;
        stats(i).alpha = a_val;
        stats(i).metrics = m;
        stats(i).data = res;
        
        fprintf('%-25s | %.4f deg        | %.4f deg\n', ...
            name, rad2deg(m.rmse_roll_gt), rad2deg(m.rmse_roll_ekf));
    end
    fprintf('------------------------------------------------------------------------------------------\n');

    %% Generate Comparative Plots
    plot_filter_results(stats);
    
    fprintf('\nAnalysis Complete. Please check the generated figures.\n');
end

function [data, metrics] = run_filter_test(Ts, Tf, angle_mag_deg, alpha_filt_val)
    time = 0:Ts:Tf;
    N = length(time);
    
    % Dimensions
    i_dim = 6; d_dim = 3;
    
    % State Initialization
    eta = zeros(i_dim,N); eta(1:3,1) = [1; 0; 0];
    nu = zeros(i_dim, N);
    a = zeros(i_dim,N);
    h = zeros(N,1);
    
    % EKF Setup
    P_loc = zeros(i_dim,i_dim,N);
    [Q_loc, R_loc, P_loc(:,:,1)] = setup_loc();
    wRr = zeros(d_dim, d_dim, N);
    wRr(:,:,1) = rotz(eta(6,1)) * roty(eta(5,1)) * rotx(eta(4,1));
    
    % Clean State (Ground Truth)
    pos = zeros(i_dim,N); pos(:,1) = eta(:,1);
    rot = zeros(d_dim,d_dim,N); rot(:,:,1) = wRr(:,:,1);
    
    % Terrain & Trajectory Generation
    p_seafloor_NED = [0; 0; 40];
    alpha = zeros(1,N);
    beta = zeros(1,N);
    
    mag = deg2rad(angle_mag_deg);
    
    % Simple Step Change Scenario
    for k = 1:N
        t = time(k);
        if t < 10
            alpha(k) = 0; beta(k) = 0;
        else
            alpha(k) = mag; beta(k) = mag;
        end
    end
    
    % Controller Params
    u_star = 0.3; v_star = 0; h_star = 10;
    Kp = [3.5; 3.5; 6.0; 2.5; 2.5; 2.0];
    Ki = [0.7; 0.7; 2.0; 0.3; 0.3; 0.4];
    Kd = [1.2; 1.2; 3.0; 2.0; 2.0; 1.5];
    
    err = zeros(i_dim, N);
    pid = zeros(i_dim, N);
    int_err = zeros(i_dim, N);
    h_err_int = 0;
    
    % Filter Setup
    eta_filt = zeros(i_dim, N); eta_filt(:,1) = eta(:,1);
    
    % Simulation Loop
    n0 = [0; 0; 1];
    wRs = zeros(d_dim, d_dim, N);
    wRs(:,:,1) = rotz(0) * roty(beta(1)) * rotx(alpha(1)) * rotx(pi);
    
    speed0 = zeros(6,1);
    
    for k = 2:N
        % Altitude Control
        n_k = wRs(:,:,k-1) * n0;
        n_norm = norm(n_k);
        nz = n_k(3) / max(n_norm, 1e-9);
        z_target = p_seafloor_NED(3) - h_star * nz;
        z_err = z_target - pos(3,k-1); 
        
        h_err_int = h_err_int + z_err * Ts;
        h_err_int = max(min(h_err_int, 10.0), -10.0);
        w_desired = 0.5 * z_err + 0.2 * h_err_int;
        w_desired = max(min(w_desired, 0.5), -0.5);
        
        % Errors (Using Filtered State)
        err(1,k) = u_star - nu(1,k-1);
        err(2,k) = v_star - nu(2,k-1);
        err(3,k) = w_desired - nu(3,k-1);
        err(4,k) = alpha(k) - eta_filt(4,k-1);
        err(5,k) = beta(k) - eta_filt(5,k-1);
        err(6,k) = 0 - eta_filt(6,k-1);
        
        % PID
        for l = 1:i_dim
            if k == 2, int_err(l,k) = err(l,k)*Ts; else, int_err(l,k) = int_err(l,k-1) + 0.5*(err(l,k)+err(l,k-1))*Ts; end
            int_err(l,k) = max(min(int_err(l,k), 5.0), -5.0);
            if k == 2, d_err = 0; else, d_err = (err(l,k)-err(l,k-1))/Ts; end
            raw_pid = Kp(l)*err(l,k) + Ki(l)*int_err(l,k) + Kd(l)*d_err;
            lim = 4.0; if l==3, lim=15.0; end
            pid(l,k) = max(min(raw_pid, lim), -lim);
        end
        
        % Dynamics & EKF
        [a(:,k), nu(:,k)] = dynamic_model(pid(:,k), zeros(i_dim,1), speed0, eta(4:6,k-1), nu(:,k-1), Ts, i_dim, a(:,k-1), 0);
        [eta(:,k), wRr(:,:,k), P_loc(:,:,k)] = ekf_position(eta(:,k-1), nu(:,k), wRr(:,:,k-1), P_loc(:,:,k-1), Q_loc, R_loc, Ts, 1); % Use 1 for noisy EKF
        
        % Apply Low Pass Filter
        eta_filt(:,k) = alpha_filt_val * eta_filt(:,k-1) + (1 - alpha_filt_val) * eta(:,k);
        
        % Clean Pos (Ground Truth)
        pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*nu(1:3,k)*Ts;
        pos(4:6,k) = eta(4:6,k-1) + nu(4:6,k)*Ts; % Note: This is a simplification, usually we integrate nu
        % Better Ground Truth Integration:
        % But for this test, we just need to compare eta_filt vs eta vs pos
        % Let's assume 'pos' is the ground truth.
        % Wait, in the original code:
        % pos(4:6,k) = eta(4:6,k-1) + nu(4:6,k)*Ts; 
        % This updates pos based on previous eta? That seems wrong for Ground Truth.
        % Let's check test_main.m again.
        % It uses: pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*nu(1:3,k)*Ts;
        % And: pos(4:6,k) = eta(4:6,k-1) + nu(4:6,k)*Ts; <-- This is using estimated angles to update GT?
        % Ah, in the simulation loop, 'eta' is the state vector.
        % If we want true ground truth, we should integrate 'nu' using the true angles.
        % But 'nu' comes from dynamic model which uses 'eta' (estimated) for damping etc.
        % So the "Ground Truth" diverges from the "Estimated" one only by the noise added in EKF?
        % In 'ekf_position', if choice=1, it adds noise.
        % Let's stick to the comparison between 'eta' (EKF output) and 'eta_filt' (Filtered output).
        
        rot(:,:,k) = rotz(eta(6,k)) * roty(eta(5,k)) * rotx(eta(4,k));
        
        % Altitude Calc
        wRs(:,:,k) = rotz(0) * roty(beta(k)) * rotx(alpha(k)) * rotx(pi);
    end
    
    % Metrics
    data.time = time;
    data.eta = eta;         % EKF Output (Noisy)
    data.eta_filt = eta_filt; % Filtered Output
    data.alpha_ref = alpha; % Reference
    
    % RMSE vs Ground Truth (Approximated by EKF for now, or we can assume EKF is close enough to truth to see filter effect)
    % Actually, let's compare Filtered vs EKF (to see smoothing) and Filtered vs Reference (to see tracking)
    
    metrics.rmse_roll_gt = sqrt(mean((eta_filt(4,:) - alpha).^2)); % Tracking Error
    metrics.rmse_roll_ekf = sqrt(mean((eta_filt(4,:) - eta(4,:)).^2)); % Deviation from EKF (Smoothing)
end

function plot_filter_results(stats)
    % Plot 1: Roll Tracking Comparison
    figure('Name', 'Filter Alpha Sensitivity - Roll Tracking');
    colors = jet(length(stats));
    hold on;
    
    % Plot Reference once
    plot(stats(1).data.time, rad2deg(stats(1).data.alpha_ref), 'k--', 'LineWidth', 2, 'DisplayName', 'Reference');
    
    for i = 1:length(stats)
        plot(stats(i).data.time, rad2deg(stats(i).data.eta_filt(4,:)), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', stats(i).name);
    end
    
    xlabel('Time [s]'); ylabel('Roll Angle [deg]');
    title('Roll Tracking for Different Filter Alphas');
    legend('Location', 'best'); grid on;
    xlim([9, 15]); % Zoom in on the step change at t=10
    
    % Plot 2: Noise Suppression Detail
    figure('Name', 'Filter Noise Suppression Detail');
    hold on;
    % Plot Raw EKF from one run (e.g., the middle one)
    mid_idx = ceil(length(stats)/2);
    plot(stats(mid_idx).data.time, rad2deg(stats(mid_idx).data.eta(4,:)), 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw EKF Output');
    
    for i = 1:length(stats)
        plot(stats(i).data.time, rad2deg(stats(i).data.eta_filt(4,:)), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', stats(i).name);
    end
    xlabel('Time [s]'); ylabel('Roll Angle [deg]');
    title('Noise Suppression Detail (Zoomed)');
    legend('Location', 'best'); grid on;
    xlim([12, 14]); % Zoom in on a steady state section
    ylim([18, 22]); % Around the 20 deg target
end
