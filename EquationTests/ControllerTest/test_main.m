clc; clear; close all;
addpath('for_controller');
addpath('rotations');
addpath('sensors');
addpath('model');
addpath('ekf_position');
addpath('evaluation');

%% Filter Parameters
Ts = 0.001;       % Sampling time [s]
Tf = 100;          % Final time [s]
time = 0:Ts:Tf;   % Time vector
N = length(time); % Number of iterations

%% Dimensions
i_dim = 6;          % Number of inputs 
d_dim = 3;          % world space total dimensions
ekf_dim = 15;      % EKF state dimension

p = 1:3;
angles = 4:6;
vel = 7:9;
rates = 10:12;
gyro = 13:15;

%%%%%%%%%%%%%%%%%%%%%%%%%% Robot / NED frame (z positive downward) %%%%%%%%%%%%%%%%%%%%%%%%%%
eta = zeros(i_dim,N);        % AUV position [x;y;z] (z down)
eta_dot = zeros(i_dim,N);    % AUV velocities
eta(1:3,1) = [0; 0; 0];     % Start position: above seafloor (z=10 < z_seafloor=20)
eta(4:6,1) = [0; 0; 0];      % Euler angles [phi;theta;psi] - start level
wRr = zeros(d_dim, d_dim, N);
nu = zeros(i_dim, N);        % Body velocities
u = zeros(i_dim, N);         % Clean Body velocities
nu_dot = zeros(i_dim,N);           % Real acceleration
h = zeros(N,1);
plane_intersection_points = zeros(d_dim, N);

%%%%%%%%%%%%%%%%%%% EKF Localization Setup %%%%%%%%%%%%%%%%%%%%%%%
P_loc = zeros(ekf_dim,ekf_dim,N);
[Q_loc, P_loc(:,:,1)] = stateLoc_init(ekf_dim);
wRr(:,:,1) = rotz(eta(6,1)) * roty(eta(5,1)) * rotx(eta(4,1));
x_loc = zeros(ekf_dim, N);

%%%%%%%%%%%%%%%%%%%%%% AUV no noise simulation %%%%%%%%%%%%%%%%%%%%%%
pos = zeros(i_dim,N); 
pos(:,1) = eta(:,1);
rot = zeros(d_dim,d_dim,N);
rot(:,:,1) = wRr(:,:,1);
h_clean = zeros(1,N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Terrain (Z positive upword) %%%%%%%%%%%%%%%%%%%%%%
p_seafloor_NED = [-3; 0; 8];
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
        alpha(k) = deg2rad(20);  % increased
    else
        alpha(k) = deg2rad(20);  
    end
    
    % Pitch (beta) profile - moderate angles for EKF
    if t < 5
        beta(k) = 0;
    elseif t < 15
        beta(k) = deg2rad(0);  % increased
    elseif t < 30
        beta(k) = deg2rad(0);  % increased
    else
        beta(k) = deg2rad(0);  % increased
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
speed0 = [0.2; 0; 0; 0; 0; 0];
u_star = 0.3;
v_star = 0;
h_star = 3;

err = zeros(i_dim, N);
pid = zeros(i_dim, N);
i_err = zeros(i_dim, N);
term_sum = zeros(i_dim, N);

max_pid = 10.0;  % Base saturation

% PID Gains (tuned for stable nonlinear model with EKF noise)
% Kp = zeros(i_dim, 1);
% Ki = zeros(i_dim, 1);
% Kd = zeros(i_dim, 1);

[Kp, Ki, Kd] = gainComputation(speed0, i_dim);

%% Loop Cycle
for k = 2:N
    if mod(k, 10000) == 0
        fprintf('t=%.1fs: nu(3)=%.3f, pid(3)=%.2f\n', ...
            time(k), nu(3,k-1), pid(3,k-1));
    end

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
        % Gain scheduling every 50 steps
        if (mod(k,50)) == 0
            [Kp, Ki, Kd] = gainComputation(nu(:,k-1), 6);
        end
        % Restore delta implementation structure (per paper), with per-axis indexing fixes
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
        pid(l,k) = max(min(pid(l,k), max_pid), -max_pid);
    end
    
    %% Dynamic Model
    [nu_dot(:,k), u(:,k)] = dynamic_model(pid(:,k), pos(4:6,k-1), u(:,k-1), Ts, i_dim, nu_dot(:,k-1));
    
    if any(~isfinite(nu_dot(:,k))) || any(~isfinite(nu(:,k)))
        error('Non finite values at k=%d', k);
    end
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
    
    if any(~isfinite(eta(:,k)))
        error('Non finite eta at k=%d', k);
    end

    %% Altitude
    wRs(:,:,k) = rotz(0) * roty(beta(k)) * rotx(alpha(k)) * rotx(pi);
    w_n = wRs(:,:,k) * n0;
    w_n_norm = norm(w_n);
    if w_n_norm < 1e-10
        error('Normal vector too small');
    end
    h(k) = abs((w_n'*(eta(1:3,k) - p_seafloor_NED))/w_n_norm);
    h_clean(k) = abs((w_n'*(pos(1:3,k) - p_seafloor_NED))/w_n_norm);
    
    w_n_dot_w_n = w_n' * w_n;
    if abs(w_n_dot_w_n) < 1e-10
        error('Normal dot product too small');
    end
    t_param = (w_n' * (p_seafloor_NED - eta(1:3,k))) / w_n_dot_w_n;
    plane_intersection_points(:,k) = eta(1:3,k) + t_param * w_n;
end



% %% Plots
% % Colors for academic use
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


