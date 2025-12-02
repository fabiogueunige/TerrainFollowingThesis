clc; clear; close all;
addpath('for_controller');
addpath('rotations');
addpath('sensors');
addpath('model');
addpath('ekf_position');

%% Filter Parameters
Ts = 0.001;       % Sampling time [s]
Tf = 50;          % Final time [s]
time = 0:Ts:Tf;   % Time vector
N = length(time); % Number of iterations

%% Dimensions
i_dim = 6;          % Number of inputs 
d_dim = 3;          % world space total dimensions

%%%%%%%%%%%%%%%%%%%%%%%%%% Robot / NED frame (z positive downward) %%%%%%%%%%%%%%%%%%%%%%%%%%
eta = zeros(i_dim,N);        % AUV position [x;y;z] (z down)
eta_dot = zeros(i_dim,N);    % AUV velocities
eta(1:3,1) = [2; 0; 0];     % Start position: above seafloor (z=10 < z_seafloor=20)
eta(4:6,1) = [0; 0; 0];      % Euler angles [phi;theta;psi] - start level
wRr = zeros(d_dim, d_dim, N);
nu = zeros(i_dim, N);        % Body velocities
a = zeros(i_dim,N);           % Real acceleration
h = zeros(N,1);
plane_intersection_points = zeros(d_dim, N);

%%%%%%%%%%%%%%%%%%% EKF Localization Setup %%%%%%%%%%%%%%%%%%%%%%%
P_loc = zeros(i_dim,i_dim,N);
[Q_loc, R_loc, P_loc(:,:,1)] = setup_loc();
wRr(:,:,1) = rotz(eta(6,1)) * roty(eta(5,1)) * rotx(eta(4,1));

%%%%%%%%%%%%%%%%%%%%%% AUV no noise simulation %%%%%%%%%%%%%%%%%%%%%%
pos = zeros(i_dim,N); 
pos(:,1) = eta(:,1);
rot = zeros(d_dim,d_dim,N);
rot(:,:,1) = wRr(:,:,1);
h_clean = zeros(1,N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Terrain (Z positive upword) %%%%%%%%%%%%%%%%%%%%%%
p_seafloor_NED = [-3; 0; 5];
alpha = zeros(1,N);
beta = zeros(1,N);

% Programmed roll and pitch setpoints (terrain angles)
for k = 1:N
    t = time(k);
    
    % Roll (alpha) profile - moderate angles for EKF
    if t < 10
        alpha(k) = 0;
    elseif t < 25
        alpha(k) = deg2rad(20);  % increased
    elseif t < 40
        alpha(k) = deg2rad(-45);  % increased
    else
        alpha(k) = deg2rad(+15);  
    end
    
    % Pitch (beta) profile - moderate angles for EKF
    if t < 5
        beta(k) = 0;
    elseif t < 15
        beta(k) = deg2rad(-20);  % increased
    elseif t < 30
        beta(k) = deg2rad(50);  % increased
    else
        beta(k) = deg2rad(20);  % increased
    end
end

n0 = [0; 0; 1];  % seafloor normal (upward)
wRs = zeros(d_dim, d_dim, N);
wRs(:,:,1) = rotz(0) * roty(beta(1)) * rotx(alpha(1)) * rotx(pi);
w_n = wRs(:,:,1) * n0;
h(1) = (w_n'*(eta(1:3,1) - p_seafloor_NED))/(norm(w_n));
h_clean(1) = h(1);

t_param = (w_n' * (p_seafloor_NED - eta(1:3,1))) / (w_n' * w_n);
plane_intersection_points(:,1) = eta(1:3,1) + t_param * w_n;

%%%%%%%%%%%%%%%%%%%%%%%%%% Controller Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%
controller = 'PID';
choice = 0;  % Nonlinear model without noise (test altitude control)
speed0 = [0; 0; 0; 0; 0; 0];
u_star = 0.3;
v_star = 0;
h_star = 3;

err = zeros(i_dim, N);
pid = zeros(i_dim, N);
int_err = zeros(i_dim, N);
% Outer altitude PI integrator
h_err_int = 0;
h_err_int_max = 10.0;
max_pid = 4.0;  % Base saturation
max_pid_W = 15.0; % Higher saturation for heave only

% PID Gains (tuned for stable nonlinear model with EKF noise)
Kp = zeros(i_dim, 1);
Ki = zeros(i_dim, 1);
Kd = zeros(i_dim, 1);

Kp(1) = 3.5;  Ki(1) = 0.7;  Kd(1) = 1.2;  % Surge
Kp(2) = 3.5;  Ki(2) = 0.7;  Kd(2) = 1.2;  % Sway
Kp(3) = 6.0;  Ki(3) = 2.0;  Kd(3) = 3.0;  % Heave (Damped)
Kp(4) = 2.5;  Ki(4) = 0.3;  Kd(4) = 2.0;  % Roll
Kp(5) = 2.5;  Ki(5) = 0.3;  Kd(5) = 2.0;  % Pitch
Kp(6) = 2.0;  Ki(6) = 0.4;  Kd(6) = 1.5;  % Yaw

%% Low-pass filter for EKF states
alpha_filt = 0.80;  % Filter coefficient (0.80 = moderate filtering with low sensor noise)
eta_filt = zeros(i_dim, N);
eta_filt(:,1) = eta(:,1);
h_filt = zeros(N,1);
h_filt(1) = h(1);

%% Loop Cycle
for k = 2:N
    if mod(k, 10000) == 0
        fprintf('t=%.1fs: z=%.2fm, h=%.2fm, nu(3)=%.3f, pid(3)=%.2f\n', ...
            time(k), eta_filt(3,k-1), h_filt(k-1), nu(3,k-1), pid(3,k-1));
    end
    
    %% Compute errors (using filtered states)
    err(1,k) = u_star - nu(1,k-1);         % Surge velocity error
    err(2,k) = v_star - nu(2,k-1);         % Sway velocity error
    
    % Heave: altitude control via desired depth z_target derived from plane normal
    % h = |n·(eta - p0)|/‖n‖, to get h_star we set target along vertical using normal's z component
    n_k = wRs(:,:,k-1) * n0;                 % plane normal at previous step
    n_norm = norm(n_k);
    nz = n_k(3) / max(n_norm, 1e-9);         % vertical component of unit normal
    z_target = p_seafloor_NED(3) - h_star * nz;  % target depth z to achieve altitude h_star
    z_err = z_target - pos(3,k-1);           % use clean depth for control
    % Altitude PI (outer)
    h_err_int = h_err_int + z_err * Ts;
    h_err_int = max(min(h_err_int, h_err_int_max), -h_err_int_max);
    k_h = 0.5;    % proportional on depth error
    k_hi = 0.2;   % integral on depth error
    w_desired = k_h * z_err + k_hi * h_err_int;    % outer loop PI
    w_desired = max(min(w_desired, 0.5), -0.5);    % tighter velocity limit
    err(3,k) = w_desired - nu(3,k-1);        % inner loop tracks vertical velocity
    
    err(4,k) = alpha(k) - eta_filt(4,k-1); % Roll angle error (filtered)
    err(5,k) = beta(k) - eta_filt(5,k-1);  % Pitch angle error (filtered)
    err(6,k) = 0 - eta_filt(6,k-1);        % Yaw angle error (filtered)
    
    %% PID Controller
    for l = 1:i_dim
        if k == 2
            int_err(l,k) = err(l,k) * Ts;
        else
            int_err(l,k) = int_err(l,k-1) + 0.5*(err(l,k) + err(l,k-1)) * Ts;
        end
        
        int_err(l,k) = max(min(int_err(l,k), 5.0), -5.0);
        
        if k == 2
            d_err = 0;
        else
            d_err = (err(l,k) - err(l,k-1)) / Ts;
        end
        
        raw_pid = Kp(l) * err(l,k) + Ki(l) * int_err(l,k) + Kd(l) * d_err;
        if l == 3
            pid(l,k) = max(min(raw_pid, max_pid_W), -max_pid_W);
        else
            pid(l,k) = max(min(raw_pid, max_pid), -max_pid);
        end
    end
    
    %% Dynamic Model
    [a(:,k), nu(:,k)] = dynamic_model(pid(:,k), zeros(i_dim,1), speed0, eta(4:6,k-1), nu(:,k-1), Ts, i_dim, a(:,k-1), choice);
    
    if any(~isfinite(a(:,k))) || any(~isfinite(nu(:,k)))
        error('Non finite values at k=%d', k);
    end

    %% EKF Localization
    [eta(:,k), wRr(:,:,k), P_loc(:,:,k)] = ekf_position(eta(:,k-1), nu(:,k), wRr(:,:,k-1), P_loc(:,:,k-1), Q_loc, R_loc, Ts, choice);
    
    if any(~isfinite(eta(:,k)))
        error('Non finite eta at k=%d', k);
    end
    
    %% Low-pass filter EKF states
    eta_filt(:,k) = alpha_filt * eta_filt(:,k-1) + (1 - alpha_filt) * eta(:,k);

    %% Clean position
    pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*nu(1:3,k)*Ts;
    pos(4:6,k) = eta(4:6,k-1) + nu(4:6,k)*Ts;
    rot(:,:,k) = rotz(eta(6,k)) * roty(eta(5,k)) * rotx(eta(4,k));

    %% Altitude
    wRs(:,:,k) = rotz(0) * roty(beta(k)) * rotx(alpha(k)) * rotx(pi);
    w_n = wRs(:,:,k) * n0;
    w_n_norm = norm(w_n);
    if w_n_norm < 1e-10
        error('Normal vector too small');
    end
    h(k) = abs((w_n'*(eta(1:3,k) - p_seafloor_NED))/w_n_norm);
    h_clean(k) = abs((w_n'*(pos(1:3,k) - p_seafloor_NED))/w_n_norm);
    
    %% Filter altitude
    h_filt(k) = alpha_filt * h_filt(k-1) + (1 - alpha_filt) * h(k);
    
    w_n_dot_w_n = w_n' * w_n;
    if abs(w_n_dot_w_n) < 1e-10
        error('Normal dot product too small');
    end
    t_param = (w_n' * (p_seafloor_NED - eta(1:3,k))) / w_n_dot_w_n;
    plane_intersection_points(:,k) = eta(1:3,k) + t_param * w_n;
end

%% Plots
% Colors for academic use
c_target = [1, 0, 0];       % Red
c_est = [0, 0, 1];          % Blue
c_clean = [0, 1, 1];        % Cyan (Azzurro acceso)

figure('Name', 'Velocities and Altitude');
subplot(3,1,1);
plot(time, nu(1,:), 'Color', c_est, 'LineWidth', 1.5); hold on;
yline(u_star, '--', 'Color', c_target, 'LineWidth', 1.5);
ylabel('Surge [m/s]'); title('Velocities'); legend('Actual', 'Target');
grid on;

subplot(3,1,2);
plot(time, nu(2,:), 'Color', c_est, 'LineWidth', 1.5); hold on;
yline(v_star, '--', 'Color', c_target, 'LineWidth', 1.5);
ylabel('Sway [m/s]'); legend('Actual', 'Target');
grid on;

subplot(3,1,3);
plot(time, h_clean, 'Color', c_clean, 'LineWidth', 2); hold on;
plot(time, h, '--', 'Color', c_est, 'LineWidth', 1.5);
yline(h_star, '--', 'Color', c_target, 'LineWidth', 1.5);
ylabel('Altitude [m]'); xlabel('Time [s]'); legend('Clean', 'EKF', 'Target');
grid on;

figure('Name', 'Euler Angles - Clean vs EKF');
subplot(3,1,1);
plot(time, rad2deg(pos(4,:)), 'Color', c_clean, 'LineWidth', 2); hold on;
plot(time, rad2deg(eta(4,:)), '--', 'Color', c_est, 'LineWidth', 1.5);
plot(time, rad2deg(alpha), '--', 'Color', c_target, 'LineWidth', 1.5);
ylabel('Roll [deg]'); title('Angles: Clean vs EKF'); legend('Clean', 'EKF', 'Target');
grid on;

subplot(3,1,2);
plot(time, rad2deg(pos(5,:)), 'Color', c_clean, 'LineWidth', 2); hold on;
plot(time, rad2deg(eta(5,:)), '--', 'Color', c_est, 'LineWidth', 1.5);
plot(time, rad2deg(beta), '--', 'Color', c_target, 'LineWidth', 1.5);
ylabel('Pitch [deg]'); legend('Clean', 'EKF', 'Target');
grid on;

subplot(3,1,3);
plot(time, rad2deg(pos(6,:)), 'Color', c_clean, 'LineWidth', 2); hold on;
plot(time, rad2deg(eta(6,:)), '--', 'Color', c_est, 'LineWidth', 1.5);
yline(0, '--', 'Color', c_target, 'LineWidth', 1.5);
ylabel('Yaw [deg]'); xlabel('Time [s]'); legend('Clean', 'EKF', 'Target');
grid on;

figure('Name', 'Position - Clean vs EKF');
subplot(3,1,1);
plot(time, pos(1,:), 'Color', c_clean, 'LineWidth', 2); hold on;
plot(time, eta(1,:), '--', 'Color', c_est, 'LineWidth', 1.5);
ylabel('X [m]'); title('Position: Clean vs EKF'); legend('Clean', 'EKF');
grid on;

subplot(3,1,2);
plot(time, pos(2,:), 'Color', c_clean, 'LineWidth', 2); hold on;
plot(time, eta(2,:), '--', 'Color', c_est, 'LineWidth', 1.5);
ylabel('Y [m]'); legend('Clean', 'EKF');
grid on;

subplot(3,1,3);
plot(time, pos(3,:), 'Color', c_clean, 'LineWidth', 2); hold on;
plot(time, eta(3,:), '--', 'Color', c_est, 'LineWidth', 1.5);
ylabel('Z [m]'); xlabel('Time [s]'); legend('Clean', 'EKF');
grid on;

fprintf('\n=== FINAL RESULTS ===\n');
fprintf('Surge: %.3f m/s (target %.3f)\n', nu(1,end), u_star);
fprintf('Sway:  %.3f m/s (target %.3f)\n', nu(2,end), v_star);
fprintf('Altitude: %.3f m (target %.3f)\n', h(end), h_star);
fprintf('Roll:  %.2f° (target %.2f°)\n', rad2deg(eta(4,end)), rad2deg(alpha(end)));
fprintf('Pitch: %.2f° (target %.2f°)\n', rad2deg(eta(5,end)), rad2deg(beta(end)));
fprintf('Yaw:   %.2f° (target 0°)\n', rad2deg(eta(6,end)));


