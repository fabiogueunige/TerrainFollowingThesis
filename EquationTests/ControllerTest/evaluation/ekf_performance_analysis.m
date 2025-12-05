function ekf_performance_analysis()
    clc; clear; close all;
    addpath('for_controller');
    addpath('rotations');
    addpath('sensors');
    addpath('model');
    addpath('ekf_position');

    %% Configuration
    Ts = 0.001; 
    Tf = 50; 
    time = 0:Ts:Tf; 
    N = length(time);
    
    % Dimensions
    i_dim = 6; 
    d_dim = 3;

    fprintf('Starting EKF Performance Analysis (Raw Estimate vs Ground Truth)...\n');

    %% Initialization
    % Initial State
    eta = zeros(i_dim,N); 
    eta(1:3,1) = [2; 0; 0]; 
    eta(4:6,1) = [0; 0; 0];
    
    % Ground Truth State
    pos_true = zeros(i_dim,N); 
    pos_true(:,1) = eta(:,1);
    
    % Velocities and Accelerations
    nu = zeros(i_dim, N); 
    a = zeros(i_dim,N);
    speed0 = zeros(6,1);
    
    % EKF Setup
    P_loc = zeros(i_dim,i_dim,N);
    [Q_loc, R_loc, P_loc(:,:,1)] = setup_loc();
    wRr = zeros(d_dim, d_dim, N);
    wRr(:,:,1) = rotz(eta(6,1)) * roty(eta(5,1)) * rotx(eta(4,1));
    
    % Filter for Controller (Internal use only, not analyzed)
    alpha_filt = 0.80;
    eta_filt = zeros(i_dim, N); 
    eta_filt(:,1) = eta(:,1);
    
    % Terrain & Trajectory
    p_seafloor_NED = [-3; 0; 5];
    alpha = zeros(1,N); 
    beta = zeros(1,N);
    
    % Generate Terrain Profile
    for k = 1:N
        t = time(k);
        if t < 10, alpha(k)=0; elseif t<25, alpha(k)=deg2rad(20); elseif t<40, alpha(k)=deg2rad(-45); else, alpha(k)=deg2rad(15); end
        if t < 5, beta(k)=0; elseif t<15, beta(k)=deg2rad(-20); elseif t<30, beta(k)=deg2rad(50); else, beta(k)=deg2rad(20); end
    end
    
    % Controller Parameters
    u_star = 0.3; v_star = 0; h_star = 3;
    Kp = [3.5; 3.5; 6.0; 2.5; 2.5; 2.0];
    Ki = [0.7; 0.7; 2.0; 0.3; 0.3; 0.4];
    Kd = [1.2; 1.2; 3.0; 2.0; 2.0; 1.5];
    
    err = zeros(i_dim, N); 
    pid = zeros(i_dim, N); 
    int_err = zeros(i_dim, N);
    h_err_int = 0; 
    h_err_int_max = 10.0;
    
    %% Simulation Loop
    for k = 2:N
        % --- 1. Controller (Uses Filtered State for Stability) ---
        wRs = rotz(0) * roty(beta(k)) * rotx(alpha(k)) * rotx(pi);
        n0 = [0;0;1]; 
        n_k = wRs * n0;
        nz = n_k(3) / max(norm(n_k), 1e-9);
        z_target = p_seafloor_NED(3) - h_star * nz;
        
        % Altitude Error
        z_err = z_target - eta_filt(3,k-1); % Using filtered Z for control
        h_err_int = max(min(h_err_int + z_err*Ts, h_err_int_max), -h_err_int_max);
        w_des = 0.5*z_err + 0.2*h_err_int;
        w_des = max(min(w_des, 0.5), -0.5);
        
        % State Errors
        err(1,k) = u_star - nu(1,k-1);
        err(2,k) = v_star - nu(2,k-1);
        err(3,k) = w_des - nu(3,k-1);
        err(4,k) = alpha(k) - eta_filt(4,k-1);
        err(5,k) = beta(k) - eta_filt(5,k-1);
        err(6,k) = 0 - eta_filt(6,k-1);
        
        % PID Calculation
        for l=1:i_dim
            if k==2, int_err(l,k)=err(l,k)*Ts; else, int_err(l,k)=int_err(l,k-1)+0.5*(err(l,k)+err(l,k-1))*Ts; end
            int_err(l,k) = max(min(int_err(l,k), 5), -5);
            d_err = (err(l,k)-err(l,k-1))/Ts;
            u_pid = Kp(l)*err(l,k) + Ki(l)*int_err(l,k) + Kd(l)*d_err;
            lim = 4.0; if l==3, lim=15.0; end
            pid(l,k) = max(min(u_pid, lim), -lim);
        end
        
        % --- 2. Dynamics (Physics) ---
        % Note: We pass 'pos_true' angles to physics if we want true physics, 
        % but the original code passed 'eta'. To be consistent with previous tests, 
        % we'll use 'eta' but ideally it should be 'pos_true'. 
        % Let's use 'pos_true' for better Ground Truth generation.
        [a(:,k), nu(:,k)] = dynamic_model(pid(:,k), zeros(i_dim,1), speed0, pos_true(4:6,k-1), nu(:,k-1), Ts, i_dim, a(:,k-1), 1);
        
        % --- 3. EKF Estimation (Noisy) ---
        [eta(:,k), wRr(:,:,k), P_loc(:,:,k)] = ekf_position(eta(:,k-1), nu(:,k), wRr(:,:,k-1), P_loc(:,:,k-1), Q_loc, R_loc, Ts, 1);
        
        % --- 4. Filter Update (For Controller Next Step) ---
        eta_filt(:,k) = alpha_filt * eta_filt(:,k-1) + (1 - alpha_filt) * eta(:,k);
        
        % --- 5. Ground Truth Integration ---
        % Integrate Linear Position
        % R_true based on TRUE angles
        phi_t = pos_true(4,k-1); theta_t = pos_true(5,k-1); psi_t = pos_true(6,k-1);
        R_true = rotz(psi_t) * roty(theta_t) * rotx(phi_t);
        pos_true(1:3,k) = pos_true(1:3,k-1) + R_true * nu(1:3,k) * Ts;
        
        % Integrate Angular Position (Euler Rates)
        % T matrix for angular velocity -> Euler rate conversion
        s_ph = sin(phi_t); c_ph = cos(phi_t);
        t_th = tan(theta_t); c_th = cos(theta_t);
        
        T_mat = [1,  s_ph * t_th,  c_ph * t_th;
                 0,  c_ph,         -s_ph;
                 0,  s_ph / c_th,  c_ph / c_th];
                 
        pos_true(4:6,k) = pos_true(4:6,k-1) + T_mat * nu(4:6,k) * Ts;
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
    
    % Plot 1: Position Estimation
    figure('Name', 'EKF Performance: Position', 'Color', 'w');
    subplot(3,1,1); hold on;
    plot(time, pos_true(1,:), 'k', 'LineWidth', 1.5);
    plot(time, eta(1,:), 'b--');
    ylabel('X [m]'); legend('Ground Truth', 'EKF Estimate'); grid on; title('Position Estimation');
    
    subplot(3,1,2); hold on;
    plot(time, pos_true(2,:), 'k', 'LineWidth', 1.5);
    plot(time, eta(2,:), 'b--');
    ylabel('Y [m]'); grid on;
    
    subplot(3,1,3); hold on;
    plot(time, pos_true(3,:), 'k', 'LineWidth', 1.5);
    plot(time, eta(3,:), 'b--');
    ylabel('Z [m]'); xlabel('Time [s]'); grid on;
    
    % Plot 2: Orientation Estimation
    figure('Name', 'EKF Performance: Orientation', 'Color', 'w');
    subplot(3,1,1); hold on;
    plot(time, rad2deg(pos_true(4,:)), 'k', 'LineWidth', 1.5);
    plot(time, rad2deg(eta(4,:)), 'b--');
    ylabel('Roll [deg]'); legend('Ground Truth', 'EKF Estimate'); grid on; title('Orientation Estimation');
    
    subplot(3,1,2); hold on;
    plot(time, rad2deg(pos_true(5,:)), 'k', 'LineWidth', 1.5);
    plot(time, rad2deg(eta(5,:)), 'b--');
    ylabel('Pitch [deg]'); grid on;
    
    subplot(3,1,3); hold on;
    plot(time, rad2deg(pos_true(6,:)), 'k', 'LineWidth', 1.5);
    plot(time, rad2deg(eta(6,:)), 'b--');
    ylabel('Yaw [deg]'); xlabel('Time [s]'); grid on;
    
    % Plot 3: Estimation Errors
    figure('Name', 'EKF Estimation Errors', 'Color', 'w');
    titles = {'Error X [m]', 'Error Y [m]', 'Error Z [m]', 'Error Roll [deg]', 'Error Pitch [deg]', 'Error Yaw [deg]'};
    for i = 1:6
        subplot(2,3,i); hold on;
        if i <= 3
            err_sig = eta(i,:) - pos_true(i,:);
        else
            err_sig = rad2deg(atan2(sin(eta(i,:) - pos_true(i,:)), cos(eta(i,:) - pos_true(i,:))));
        end
        plot(time, err_sig, 'r');
        yline(0, 'k--');
        title(titles{i}); grid on;
        if i > 3, xlabel('Time [s]'); end
    end
end
