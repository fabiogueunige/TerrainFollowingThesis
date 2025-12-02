function controller_evaluation()
    clc; clear; close all;
    addpath('for_controller');
    addpath('rotations');
    addpath('sensors');
    addpath('model');
    addpath('ekf_position');

    %% Global Configuration
    Ts = 0.001;
    Tf = 60; 
    
    % Variations of Scenario 4 (Two Soft Changes)
    % We will test different magnitudes for the pitch/roll changes
    angle_variations = [40, 45, 50, 55, 60]; % Degrees
    num_variations = length(angle_variations);
    stats = [];

    fprintf('Starting Controller Sensitivity Analysis (Scenario 4)...\n');
    fprintf('------------------------------------------------------------------------------------------\n');
    fprintf('%-25s | %-10s | %-10s | %-10s | %-10s\n', 'Variation', 'Alt RMSE', 'Roll RMSE', 'Pitch RMSE', 'Sway RMSE');
    fprintf('------------------------------------------------------------------------------------------\n');

    for i = 1:num_variations
        angle_deg = angle_variations(i);
        name = sprintf('Change +/- %.0f deg', angle_deg);
        
        % Run Scenario 4 with specific angle magnitude
        [res, m] = run_scenario_4_variant(Ts, Tf, angle_deg);
        
        stats(i).name = name;
        stats(i).metrics = m;
        stats(i).data = res;
        
        fprintf('%-25s | %.4f m   | %.4f deg  | %.4f deg   | %.4f m/s\n', ...
            name, m.rmse_h, rad2deg(m.rmse_roll), rad2deg(m.rmse_pitch), m.rmse_sway);
    end
    fprintf('------------------------------------------------------------------------------------------\n');

    %% Generate Comparative Plots
    plot_sensitivity(stats);
    
    fprintf('\nAnalysis Complete. Please check the generated figures.\n');
end

function [data, metrics] = run_scenario_4_variant(Ts, Tf, angle_mag_deg)
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
    
    % Clean State
    pos = zeros(i_dim,N); pos(:,1) = eta(:,1);
    rot = zeros(d_dim,d_dim,N); rot(:,:,1) = wRr(:,:,1);
    h_clean = zeros(1,N);
    
    % Terrain & Trajectory Generation
    p_seafloor_NED = [0; 0; 40];
    alpha = zeros(1,N);
    beta = zeros(1,N);
    
    mag = deg2rad(angle_mag_deg);
    
    for k = 1:N
        t = time(k);
        % Scenario 4 Logic: Two Soft Changes
        if t < 10
            alpha(k) = 0; beta(k) = 0;
        elseif t < 25
            alpha(k) = mag; beta(k) = mag;
        else
            alpha(k) = -mag; beta(k) = -mag;
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
    alpha_filt = 0.80;
    eta_filt = zeros(i_dim, N); eta_filt(:,1) = eta(:,1);
    h_filt = zeros(N,1); h_filt(1) = 3; 
    
    % Simulation Loop
    n0 = [0; 0; 1];
    wRs = zeros(d_dim, d_dim, N);
    wRs(:,:,1) = rotz(0) * roty(beta(1)) * rotx(alpha(1)) * rotx(pi);
    w_n = wRs(:,:,1) * n0;
    h(1) = abs((w_n'*(eta(1:3,1) - p_seafloor_NED))/(norm(w_n)));
    h_clean(1) = h(1);
    h_filt(1) = h(1);
    
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
        
        % Errors
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
        [eta(:,k), wRr(:,:,k), P_loc(:,:,k)] = ekf_position(eta(:,k-1), nu(:,k), wRr(:,:,k-1), P_loc(:,:,k-1), Q_loc, R_loc, Ts, 0);
        
        % Filter
        eta_filt(:,k) = alpha_filt * eta_filt(:,k-1) + (1 - alpha_filt) * eta(:,k);
        
        % Clean Pos
        pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*nu(1:3,k)*Ts;
        pos(4:6,k) = eta(4:6,k-1) + nu(4:6,k)*Ts;
        rot(:,:,k) = rotz(eta(6,k)) * roty(eta(5,k)) * rotx(eta(4,k));
        
        % Altitude Calc
        wRs(:,:,k) = rotz(0) * roty(beta(k)) * rotx(alpha(k)) * rotx(pi);
        w_n = wRs(:,:,k) * n0;
        h(k) = abs((w_n'*(eta(1:3,k) - p_seafloor_NED))/(norm(w_n)));
        h_clean(k) = abs((w_n'*(pos(1:3,k) - p_seafloor_NED))/norm(w_n));
        h_filt(k) = alpha_filt * h_filt(k-1) + (1 - alpha_filt) * h(k);
    end
    
    % Metrics
    data.time = time;
    data.h_clean = h_clean;
    data.h_target = h_star;
    data.alpha = alpha;
    data.beta = beta;
    data.roll = pos(4,:);
    data.pitch = pos(5,:);
    data.nu = nu;
    
    metrics.rmse_h = sqrt(mean((h_clean - h_star).^2));
    metrics.rmse_roll = sqrt(mean((pos(4,:) - alpha).^2));
    metrics.rmse_pitch = sqrt(mean((pos(5,:) - beta).^2));
    metrics.rmse_sway = sqrt(mean((nu(2,:) - v_star).^2));
end

function plot_sensitivity(stats)
    % Plot 1: Altitude Sensitivity
    figure('Name', 'Altitude Sensitivity Analysis');
    colors = parula(length(stats)); % Use a gradient colormap
    hold on;
    for i = 1:length(stats)
        plot(stats(i).data.time, stats(i).data.h_clean, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', stats(i).name);
    end
    yline(3, 'k--', 'Target', 'LineWidth', 2);
    xlabel('Time [s]'); ylabel('Altitude [m]');
    title('Altitude Stability vs Terrain Angle Magnitude');
    legend('Location', 'best'); grid on;
    
    % Plot 2: Pitch Tracking Sensitivity
    figure('Name', 'Pitch Tracking Sensitivity');
    hold on;
    for i = 1:length(stats)
        plot(stats(i).data.time, rad2deg(stats(i).data.pitch), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', stats(i).name);
    end
    % Plot one target for reference (the largest one)
    plot(stats(end).data.time, rad2deg(stats(end).data.beta), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Target (Max)');
    
    xlabel('Time [s]'); ylabel('Pitch [deg]');
    title('Pitch Tracking vs Angle Magnitude');
    legend('Location', 'best'); grid on;
end
