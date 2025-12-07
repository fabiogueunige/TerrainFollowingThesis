function controller_evaluation()
    clc; clear; close all;
    addpath('../for_controller');
    addpath('../rotations');
    addpath('../sensors');
    addpath('../model');
    addpath('../ekf_position');

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
    i_dim = 6; d_dim = 3; ekf_dim = 15;
    
    % State Initialization
    eta = zeros(i_dim,N); eta(1:3,1) = [0; 0; 0]; eta(4:6,1) = [0; 0; 0];
    nu = zeros(i_dim, N);
    u = zeros(i_dim, N);
    nu_dot = zeros(i_dim,N);
    h = zeros(N,1);
    plane_intersection_points = zeros(d_dim, N);
    
    % EKF Setup
    P_loc = zeros(ekf_dim,ekf_dim,N);
    [Q_loc, P_loc(:,:,1)] = stateLoc_init(ekf_dim);
    wRr = zeros(d_dim, d_dim, N);
    wRr(:,:,1) = rotz(eta(6,1)) * roty(eta(5,1)) * rotx(eta(4,1));
    x_loc = zeros(ekf_dim, N);
    
    % Clean State
    pos = zeros(i_dim,N); pos(:,1) = eta(:,1);
    rot = zeros(d_dim,d_dim,N); rot(:,:,1) = wRr(:,:,1);
    h_clean = zeros(1,N);
    
    % Terrain & Trajectory Generation
    p_seafloor_NED = [-3; 0; 8];
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
    
    n0 = [0; 0; 1];
    wRs = zeros(d_dim, d_dim, N);
    wRs(:,:,1) =  rotx(pi) * rotz(0) * roty(beta(1)) * rotx(alpha(1));
    w_n = wRs(:,:,1) * n0;
    h(1) = (w_n'*(eta(1:3,1) - p_seafloor_NED))/(norm(w_n));
    h_clean(1) = h(1);
    
    % Controller Params
    speed0 = [0.2; 0; 0; 0; 0; 0];
    u_star = 0.3; v_star = 0; h_star = 3;
    
    err = zeros(i_dim, N);
    pid = zeros(i_dim, N);
    i_err = zeros(i_dim, N);
    term_sum = zeros(i_dim, N);
    
    % Differentiated saturation limits
    max_pid_vec = [4.0; 4.0; 2.5; 2.5; 2.5; 4.0]; % [surge,sway,heave,roll,pitch,yaw]
    
    [Kp, Ki, Kd] = gainComputation(speed0, i_dim);
    
    for k = 2:N
        %% Altitude contribution to errors
        h_err = (h_star - h(k-1)); 
        r_n = wRr(:,:,k-1)' * w_n;  % Normal in robot frame
        r_n = r_n / norm(r_n);  % Normalize
        h_contribution = h_err * r_n;  % [surge; sway; heave] contribution
        
        %% Error computation with intelligent altitude control
        % Surge: maintain constant speed + altitude correction
        err(1,k) = u_star - nu(1,k-1) + 0.3 * h_contribution(1);
        
        % Sway: keep zero lateral velocity + altitude correction
        err(2,k) = v_star - nu(2,k-1) + 0.3 * h_contribution(2);
        
        % Heave: altitude correction is primary objective
        err(3,k) = h_contribution(3);
    
        err(4,k) = alpha(k) - eta(4,k-1);
        err(5,k) = beta(k) - eta(5,k-1);
        err(6,k) = 0 - eta(6,k-1);
        
        %% Derivative of velocities (for all axes)
        for l = 1:i_dim
            if l == 1 || l == 2
                % Horizontal axes: use delta form with anti-windup
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu_dot(l,k-1);           % delta term on velocity derivative
                d_err = 0;
            else
                % Heave (NED z-down): include derivative and measurement term per structure
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu(l,k-1);               % proportional on measured heave velocity
                d_err = Kd(l) * nu_dot(l,k-1);
            end
            term_sum(l,k) = i_err(l,k) - p_err - d_err;
            pid(l,k) = integrator(pid(l,k-1), term_sum(l,k), term_sum(l,k-1), Ts);
            pid(l,k) = max(min(pid(l,k), max_pid_vec(l)), -max_pid_vec(l));
        end
        
        %% Dynamic Model
        [nu_dot(:,k), u(:,k)] = dynamic_model(pid(:,k), pos(4:6,k-1), u(:,k-1), Ts, i_dim, nu_dot(:,k-1));
        
        %% Clean position
        T = transformationT(pos(4:6,k-1));
        pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*u(1:3,k)*Ts;
        pos(4:6,k) = pos(4:6,k-1) + T * u(4:6,k) *Ts;
        rot(:,:,k) = rotz(pos(6,k)) * roty(pos(5,k)) * rotx(pos(4,k));
        eta_clean = [pos(1:3,k); pos(4:6,k)];
    
        %% EKF Localization
        [x_loc(:,k), P_loc(:,:,k), wRr(:,:,k)] = ekf_position(x_loc(:,k-1), pid(:,k), wRr(:,:,k-1), u(:,k), eta_clean, P_loc(:,:,k-1), Q_loc, Ts);
        eta(:,k) = x_loc(1:6,k);
        nu(:,k) = x_loc(7:12,k);
        
        %% Altitude
        wRs(:,:,k) =  rotx(pi) * rotz(0) * roty(beta(k)) * rotx(alpha(k));
        w_n = wRs(:,:,k) * n0;
        w_n_norm = norm(w_n);
        h(k) = abs((w_n'*(eta(1:3,k) - p_seafloor_NED))/w_n_norm);
        h_clean(k) = abs((w_n'*(pos(1:3,k) - p_seafloor_NED))/w_n_norm);
    end
    
    % Metrics
    data.time = time;
    data.h_clean = h_clean;
    data.h_target = h_star;
    data.alpha = alpha;
    data.beta = beta;
    data.pos = pos;
    data.roll = pos(4,:);
    data.pitch = pos(5,:);
    data.nu = nu;
    
    % Calculate Terrain Z for visualization
    % Plane equation: n_x*(x-x0) + n_y*(y-y0) + n_z*(z-z0) = 0
    % z_terrain = z0 - (n_x*(x-x0) + n_y*(y-y0))/n_z
    z_terrain = zeros(1,N);
    for k=1:N
        % Re-calculate normal (it was local in loop, need it here or store it)
        % To save memory, just re-calc
        wRs_k = rotx(pi) * rotz(0) * roty(beta(k)) * rotx(alpha(k));
        n_k = wRs_k * [0;0;1];
        
        if abs(n_k(3)) > 1e-6
            z_terrain(k) = p_seafloor_NED(3) - (n_k(1)*(pos(1,k)-p_seafloor_NED(1)) + n_k(2)*(pos(2,k)-p_seafloor_NED(2))) / n_k(3);
        else
            z_terrain(k) = NaN; % Vertical slope?
        end
    end
    data.z_terrain = z_terrain;
    
    metrics.rmse_h = sqrt(mean((h_clean - h_star).^2));
    metrics.rmse_roll = sqrt(mean((pos(4,:) - alpha).^2));
    metrics.rmse_pitch = sqrt(mean((pos(5,:) - beta).^2));
    metrics.rmse_sway = sqrt(mean((nu(2,:) - v_star).^2));
end

function plot_sensitivity(stats)
    % Common plot settings
    lw = 1.5;       % LineWidth
    fs = 12;        % FontSize
    fn = 'Times New Roman'; % FontName
    colors = parula(length(stats)); % Use a gradient colormap

    % Plot 1: Altitude Sensitivity (Clean)
    figure('Name', 'Altitude Sensitivity Analysis');
    hold on;
    for i = 1:length(stats)
        plot(stats(i).data.time, stats(i).data.h_clean, 'Color', colors(i,:), 'LineWidth', lw, 'DisplayName', stats(i).name);
    end
    yline(3, 'k--', 'Target', 'LineWidth', 2, 'DisplayName', 'Target');
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn); 
    ylabel('Altitude [m]', 'FontSize', fs, 'FontName', fn);
    title('Altitude Stability vs Terrain Angle Magnitude', 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn); 
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 2: Pitch Tracking Sensitivity
    figure('Name', 'Pitch Tracking Sensitivity');
    hold on;
    for i = 1:length(stats)
        plot(stats(i).data.time, rad2deg(stats(i).data.pitch), 'Color', colors(i,:), 'LineWidth', lw, 'DisplayName', stats(i).name);
    end
    % Plot one target for reference (the largest one)
    plot(stats(end).data.time, rad2deg(stats(end).data.beta), 'r--', 'LineWidth', lw, 'DisplayName', 'Target (Max)');
    
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn); 
    ylabel('Pitch [deg]', 'FontSize', fs, 'FontName', fn);
    title('Pitch Tracking vs Angle Magnitude', 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn); 
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);

    % Plot 3: Terrain Tracking Profile (Max Variation)
    % This clearly shows the robot depth vs terrain depth
    figure('Name', 'Terrain Tracking Profile (Max Variation)');
    hold on;
    
    max_stat = stats(end);
    time = max_stat.data.time;
    z_robot = max_stat.data.pos(3,:);
    z_terrain = max_stat.data.z_terrain;
    
    % Plot Terrain (Ground Truth)
    plot(time, z_terrain, 'k-', 'LineWidth', 2, 'DisplayName', 'Terrain Depth');
    
    % Plot Robot Depth
    plot(time, z_robot, 'b-', 'LineWidth', 2, 'DisplayName', 'Robot Depth');
    
    % Plot Target Depth (approximate)
    % Target Depth = Terrain Depth - h_star (roughly, ignoring slope effects on vertical projection for visualization)
    % plot(time, z_terrain - 3, 'g--', 'LineWidth', 1, 'DisplayName', 'Target Depth (Approx)');

    set(gca, 'YDir', 'reverse'); % Depth convention (positive down)
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Depth [m]', 'FontSize', fs, 'FontName', fn);
    title(['Terrain Tracking Profile (' max_stat.name ')'], 'FontSize', fs, 'FontName', fn);
    legend('Location', 'best', 'FontSize', fs, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
end
