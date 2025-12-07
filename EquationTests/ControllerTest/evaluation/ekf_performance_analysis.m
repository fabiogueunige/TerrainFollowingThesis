function ekf_performance_analysis()
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

    fprintf('Starting EKF Performance Analysis (Raw Estimate vs Ground Truth)...\n');

    %% Initialization
    % Initial State
    eta = zeros(i_dim,N); 
    eta(1:3,1) = [0; 0; 0]; 
    eta(4:6,1) = [0; 0; 0];
    
    % Ground Truth State
    pos_true = zeros(i_dim,N); 
    pos_true(:,1) = eta(:,1);
    rot = zeros(d_dim,d_dim,N);
    
    % Velocities and Accelerations
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
    
    % Terrain & Trajectory
    p_seafloor_NED = [-3; 0; 5];
    alpha = zeros(1,N); 
    beta = zeros(1,N);
    h = zeros(N,1);
    h_clean = zeros(N,1);
    
    % Generate Terrain Profile (Complex profile from original file)
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
        r_n = wRr(:,:,k-1)' * w_n;  % Normal in robot frame
        r_n = r_n / norm(r_n);  % Normalize
        h_contribution = h_err * r_n;  % [surge; sway; heave] contribution
        
        %% Error computation with intelligent altitude control
        % Surge: maintain constant speed + altitude correction
        err(1,k) = u_star - nu(1,k-1) + 0.5 * h_contribution(1);
        
        % Sway: keep zero lateral velocity + altitude correction
        err(2,k) = v_star - nu(2,k-1) + 0.5 * h_contribution(2);
        
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
        
        %% Altitude
        wRs(:,:,k) =  rotx(pi) * rotz(0) * roty(beta(k)) * rotx(alpha(k));
        w_n = wRs(:,:,k) * n0;
        w_n_norm = norm(w_n);
        h(k) = abs((w_n'*(eta(1:3,k) - p_seafloor_NED))/w_n_norm);
        h_clean(k) = abs((w_n'*(pos_true(1:3,k) - p_seafloor_NED))/w_n_norm);
    end
    
    %% Metrics Calculation (RMSE)
    rmse = zeros(6,1);
    for i = 1:6
        if i <= 3
            % Position RMSE [m]
            rmse(i) = sqrt(mean((eta(i,:) - pos_true(i,:)).^2));
        else
            % Angle RMSE [deg] - Handle wrapping
            diff_ang = eta(i,:) - pos_true(i,:);
            diff_ang = atan2(sin(diff_ang), cos(diff_ang));
            rmse(i) = rad2deg(sqrt(mean(diff_ang.^2)));
        end
    end
    
    %% Display Results
    fprintf('\n--------------------------------------------------\n');
    fprintf('EKF ESTIMATION ACCURACY (RMSE vs Ground Truth)\n');
    fprintf('--------------------------------------------------\n');
    fprintf('Position X:   %.4f m\n', rmse(1));
    fprintf('Position Y:   %.4f m\n', rmse(2));
    fprintf('Position Z:   %.4f m\n', rmse(3));
    fprintf('Roll (Phi):   %.4f deg\n', rmse(4));
    fprintf('Pitch (Theta):%.4f deg\n', rmse(5));
    fprintf('Yaw (Psi):    %.4f deg\n', rmse(6));
    fprintf('--------------------------------------------------\n');

    %% Plotting
    % Common plot settings
    lw = 1.5;       % LineWidth
    fs = 12;        % FontSize
    fn = 'Times New Roman'; % FontName
    
    % Plot 1: Position X
    figure('Name', 'EKF Performance: Position X');
    plot(time, eta(1,:), 'b--', 'LineWidth', lw); hold on;
    plot(time, pos_true(1,:), 'r', 'LineWidth', lw);
    ylabel('X [m]', 'FontSize', fs, 'FontName', fn); 
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    legend({'EKF Estimate', 'Ground Truth'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best'); 
    grid on; title('Position X Estimation', 'FontSize', fs, 'FontName', fn);
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 2: Position Y
    figure('Name', 'EKF Performance: Position Y');
    plot(time, eta(2,:), 'b--', 'LineWidth', lw); hold on;
    plot(time, pos_true(2,:), 'r', 'LineWidth', lw);
    ylabel('Y [m]', 'FontSize', fs, 'FontName', fn); 
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    legend({'EKF Estimate', 'Ground Truth'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best'); 
    grid on; title('Position Y Estimation', 'FontSize', fs, 'FontName', fn);
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 3: Position Z
    figure('Name', 'EKF Performance: Position Z');
    plot(time, eta(3,:), 'b--', 'LineWidth', lw); hold on;
    plot(time, pos_true(3,:), 'r', 'LineWidth', lw);
    ylabel('Z [m]', 'FontSize', fs, 'FontName', fn); 
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    legend({'EKF Estimate', 'Ground Truth'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best'); 
    grid on; title('Position Z Estimation', 'FontSize', fs, 'FontName', fn);
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 4: Roll
    figure('Name', 'EKF Performance: Roll');
    plot(time, rad2deg(eta(4,:)), 'b--', 'LineWidth', lw); hold on;
    plot(time, rad2deg(pos_true(4,:)), 'r', 'LineWidth', lw);
    ylabel('Roll [deg]', 'FontSize', fs, 'FontName', fn); 
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    legend({'EKF Estimate', 'Ground Truth'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best'); 
    grid on; title('Roll Estimation', 'FontSize', fs, 'FontName', fn);
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 5: Pitch
    figure('Name', 'EKF Performance: Pitch');
    plot(time, rad2deg(eta(5,:)), 'b--', 'LineWidth', lw); hold on;
    plot(time, rad2deg(pos_true(5,:)), 'r', 'LineWidth', lw);
    ylabel('Pitch [deg]', 'FontSize', fs, 'FontName', fn); 
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    legend({'EKF Estimate', 'Ground Truth'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best'); 
    grid on; title('Pitch Estimation', 'FontSize', fs, 'FontName', fn);
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 6: Yaw
    figure('Name', 'EKF Performance: Yaw');
    plot(time, rad2deg(eta(6,:)), 'b--', 'LineWidth', lw); hold on;
    plot(time, rad2deg(pos_true(6,:)), 'r', 'LineWidth', lw);
    ylabel('Yaw [deg]', 'FontSize', fs, 'FontName', fn); 
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    legend({'EKF Estimate', 'Ground Truth'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best'); 
    grid on; title('Yaw Estimation', 'FontSize', fs, 'FontName', fn);
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 7: Estimation Errors (Single Figures)
    titles = {'Error X', 'Error Y', 'Error Z', 'Error Roll', 'Error Pitch', 'Error Yaw'};
    units = {'[m]', '[m]', '[m]', '[deg]', '[deg]', '[deg]'};
    
    for i = 1:6
        figure('Name', ['EKF Error: ' titles{i}]);
        if i <= 3
            err_sig = eta(i,:) - pos_true(i,:);
        else
            err_sig = rad2deg(atan2(sin(eta(i,:) - pos_true(i,:)), cos(eta(i,:) - pos_true(i,:))));
        end
        plot(time, err_sig, 'r', 'LineWidth', lw);
        yline(0, 'k--', 'LineWidth', 1);
        ylabel(units{i}, 'FontSize', fs, 'FontName', fn);
        xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
        title(titles{i}, 'FontSize', fs, 'FontName', fn);
        grid on;
        set(gca, 'FontSize', fs, 'FontName', fn);
    end
end
