clc; clear; close all;
addpath('for_controller');
addpath('rotations');
%% Filter Parameters
Ts = 0.001;       % Sampling time [s]
Tf = 50;          % Final time [s]
time = 0:Ts:Tf;   % Time vector
N = length(time); % Number of iterations
%% Dimensions
i_dim = 3;          % Number of inputs 
d_dim = 3;          % world space total dimensions

%% Robot / NED frame (z positive downward)
eta1 = zeros(d_dim,N);        % AUV position [x;y;z] (z down)
eta1(:,1) = [-20; 0; 10];     % Start position: above seafloor (z=10 < z_seafloor=20)
eta2 = [pi/10; pi/6; 0];      % Euler angles [phi;theta;psi]
% wRr = zeros(i_dim, i_dim);
nu1 = zeros(i_dim, N);        % Body velocities (assumed aligned with NED axes here)
nu1_dot = zeros(i_dim,N);     % Accelerations (derivative of velocity)
a = zeros(i_dim,N);         % Real acceleration command effect
h = zeros(N,1);
plane_intersection_points = zeros(d_dim, N);  % World frame intersection points with plane

wRr = rotz(eta2(3)) * roty(eta2(2)) * rotx(eta2(1));

%% Terrain (Z positive upword)
p_seafloor_NED = [-20; 0; 20];
alpha = pi/7;
beta = pi/9;
% wRs = zeros(i_dim, i_dim);
n0 = [0; 0; 1];  % seafloor normal (upward)

wRs = rotz(0) * roty(beta) * rotx(alpha) * rotx(pi);
w_n = wRs * n0;  % rotated seafloor normal
h(1) = (w_n'*(eta1(:,1) - p_seafloor_NED))/(norm(w_n));
% Calculate initial intersection point: project robot position onto plane
t_param = (w_n' * (p_seafloor_NED - eta1(:,1))) / (w_n' * w_n);
plane_intersection_points(:,1) = eta1(:,1) + t_param * w_n;

%% Controller choice
controller = 'A+D';  % Valid working modes: 'PID' or 'A+D'
speed0 = [0; 0; 0];
tau0 = zeros(i_dim,1);

%% Dynamic
% massa totale [kg]
m = 11.5; 
% added mass
tau_a = -[27.08; 25.952; 29.9081];
% Linear damping
tau_r = [-0.1213; -1.1732; -1.1130];
% Quadratic damping
tau_d = [-23.9000; -46.2700; -50.2780];   
mv = m - tau_a;
% Dissipative forces (con v0 = 0.1 solo nel surge)
dv = -tau_r - tau_d.*abs(speed0);
dv_lin = -tau_r - 2*tau_d.*abs(speed0);

%% Controller Parameters
u_star = 0.3;
v_star = 0;
h_star = 3;

err = zeros(i_dim, N);
pid = zeros(i_dim, N);          % PID for Dynamics
int_term = zeros(i_dim, N);     % integral error
term_sum = zeros(i_dim, N);     % integral error
max_pid = 1;
integral_max = 1;

% errori pid semplice
i_err = zeros(i_dim, N);            % integral error

%% tau0
for l = 1:i_dim
    tau0(l) = dv(l)*speed0(l);
end

%% Gain Computation
kp = zeros(i_dim, N);
ki = zeros(i_dim, N);
kd = zeros(i_dim, N);
Kt = zeros(i_dim, N);
[Kp(:,1), Ki(:,1), Kd(:,1), Kt(:,1)] = gainComputation(speed0);


%% Loop Cycle
for k = 3:N
    %% Altitude error and projection onto robot axes
    % Convention: h_err = (h_current - h_target)
    % - If h > h_star (too far): h_err > 0 → need to move TOWARD plane (along +w_n direction)
    % - If h < h_star (too close): h_err < 0 → need to move AWAY from plane (along -w_n direction)
    % Note: In NED, w_n points "away from plane" (upward), but has positive Z component (downward in world)
    %       so moving along +w_n actually means descending toward plane in NED frame
    h_err = -(h_star - h(k-1));  % = h(k-1) - h_star (current - target)
    
    % Normal vector in robot frame (how altitude h projects onto robot axes)
    % Transform world normal to robot frame
    r_n = wRr' * w_n;  % Normal in robot frame
    r_n = r_n / norm(r_n);  % Normalize
    
    % Project altitude error onto each robot axis
    % To correct h, we need velocity along the normal direction
    % Each axis contributes proportionally to its alignment with the normal
    % h_contribution is the velocity command in robot frame [surge; sway; heave]
    h_contribution = h_err * r_n;  % [surge; sway; heave] contribution
    
    %% Error computation with intelligent altitude control
    % Surge: maintain constant speed + altitude correction
    err(1,k) = u_star - nu1(1,k-1) + 0.3 * h_contribution(1);
    
    % Sway: keep zero lateral velocity + altitude correction
    err(2,k) = v_star - nu1(2,k-1) + 0.3 * h_contribution(2);
    
    % Heave: altitude correction is primary objective
    err(3,k) = -h_contribution(3);
    
    %% Derivative of velocities (for all axes)
    for l = 1:i_dim
        nu1_dot(l,k-1) = derivator(nu1_dot(l,k-2), nu1(l,k-1), nu1(l,k-2), Ts);
    end

    %% Controllers
    for l = 1:i_dim
        switch controller
            case 'PID'
                % Standard PID (derivative only meaningful on heave to damp vertical motion)
                int_state(l) = integrator(int_state(l), err(l,k), err(l,k-1), Ts);
                p_err = Kp(l) * err(l,k);
                i_err(l,k) = Ki(l) * int_state(l);
                d_err = 0;
                if l == 3  % heave axis
                    d_err = -Kd(l) * nu1_dot(l,k-1); % derivative on measurement
                end
                raw = p_err + i_err(l,k) + d_err;
                pid(l,k) = max(min(raw, max_pid), -max_pid);

            case 'A+D'
                % Gain scheduling every 50 steps
                if (mod(k,50)) == 0
                    [Kp, Ki, Kd, Kt] = gainComputation(nu1(:,k-1));
                end
                % Restore delta implementation structure (per paper), with per-axis indexing fixes
                if l == 1 || l == 2
                    % Horizontal axes: use delta form with anti-windup
                    i_err(l,k) = Ki(l) * err(l,k);
                    p_err = Kp(l) * nu1_dot(l,k-1);           % delta term on velocity derivative
                    term_sum(l,k) = i_err(l,k) - p_err - Kt(l) * pid(l,k-1);
                    int_term(l,k) = integrator(int_term(l,k-1), term_sum(l,k), term_sum(l,k-1), Ts);
                    err_sat = max(min(int_term(l,k), max_pid), -max_pid);
                    pid(l,k) = int_term(l,k) - err_sat;
                else
                    % Heave (NED z-down): include derivative and measurement term per structure
                    i_err(l,k) = Ki(l) * err(l,k);
                    p_err = Kp(l) * nu1(l,k-1);               % proportional on measured heave velocity
                    d_err = Kd(l) * nu1_dot(l,k-1);
                    % NED convention damping: negative derivative sign for heave
                    % d_err = -d_err;
                    term_sum(l,k) = i_err(l,k) - p_err - d_err - Kt(l) * pid(l,k-1);
                    int_term(l,k) = integrator(int_term(l,k-1), term_sum(l,k), term_sum(l,k-1), Ts);
                    err_sat = max(min(int_term(l,k), max_pid), -max_pid);
                    pid(l,k) = int_term(l,k) - err_sat;
                end
        end
    end
    %% Dynamic (simple decoupled model)
    % Surge & sway: standard sign. Heave (z): we want positive pid to reduce altitude (descend),
    % altitude decreases when z increases toward seafloor (z moves from z_init to z_target = z_seafloor_NED - h_star).
    for l = 1:i_dim
        a(l,k) = (pid(l,k) - dv(l)*nu1(l,k-1)) / mv(l);
        nu1(l,k) = integrator(nu1(l,k-1), a(l,k), a(l,k-1), Ts);
    end

    %% New point
    eta1(:,k) = eta1(:,k-1) + wRr*nu1(:,k)*Ts;   % Position integration (z increases downward)
    if ~isfinite(eta1(3,k)); error('Non finite'); end
    % Altitude update (above seafloor)
    h(k) = abs((w_n'*(eta1(:,k) - p_seafloor_NED))/(norm(w_n)));
    
    % Calculate intersection point: project current robot position onto plane
    % Plane equation: w_n' * (P - p_seafloor_NED) = 0
    % Line from robot perpendicular to plane: P = eta1(:,k) + t * w_n
    % Solve for t: w_n' * (eta1(:,k) + t*w_n - p_seafloor_NED) = 0
    t_param = (w_n' * (p_seafloor_NED - eta1(:,k))) / (w_n' * w_n);
    plane_intersection_points(:,k) = eta1(:,k) + t_param * w_n;
end

figure;
plot(time, nu1(1,:), 'LineWidth',1.1);
hold on;
plot(time, nu1(2,:), 'LineWidth',1.1);
plot(time, nu1(3,:), 'LineWidth',1.1);
xlabel('Time [s]'); ylabel('Velocity [m/s]');
title(['AUV Velocities (Controller = ',controller,')']);
legend('Surge (nu1)','Sway (v)','Heave (w)','Location','best'); grid on; hold off;

% Heave altitude reconstruction & diagnostics (from position, NED)
tol = 0.05; % 5 cm tolerance
settle_idx = find(abs(err(3,:)) < tol,1,'first');
fprintf('\nHeave final altitude: %.3f m (target %.3f m)\n', h(end), h_star);
fprintf('Heave final error: %.3f m\n', err(3,end));
if ~isempty(settle_idx)
    fprintf('Heave settled within %.2f m at t = %.2f s\n', tol, time(settle_idx));
else
    fprintf('Heave did NOT settle within %.2f m tolerance.\n', tol);
end
fprintf('Heave altitude range: [%.3f , %.3f] m\n', min(h), max(h));

figure; plot(time, h,'b','LineWidth',1.1); hold on; yline(h_star,'r--','h^*');
xlabel('Time [s]'); ylabel('Altitude h [m]'); title('Heave Altitude'); grid on; hold off;

figure; plot(time, err(3,:),'k','LineWidth',1.1);
xlabel('Time [s]'); ylabel('Altitude Error (h^* - h) [m]'); title('Heave Altitude Error'); grid on;

figure;
scatter3(eta1(1,:), eta1(2,:), -eta1(3,:), [], time);
colorbar; 
colormap(jet);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m] (up)');
title('Trajectory XYZ (Color = time)');
set(gca, 'ZDir', 'normal');  % Z positive upward
grid on;

%% 3D Visualization: Robot trajectory + Plane intersection points + Seafloor plane
figure('Position', [100, 100, 1200, 800]);
hold on; grid on; axis equal;

% Invert Z for intuitive visualization (Z up, robot above ground)
% Plot robot trajectory
plot3(eta1(1,:), eta1(2,:), -eta1(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Robot Trajectory');

% Plot intersection points on plane
scatter3(plane_intersection_points(1,:), plane_intersection_points(2,:), ...
         -plane_intersection_points(3,:), 30, time, 'filled', ...
         'DisplayName', 'Plane Intersection Points');

% Plot vertical lines connecting robot to plane (altitude h)
for k_sample = 1:max(1, floor(N/50)):N
    plot3([eta1(1,k_sample), plane_intersection_points(1,k_sample)], ...
          [eta1(2,k_sample), plane_intersection_points(2,k_sample)], ...
          [-eta1(3,k_sample), -plane_intersection_points(3,k_sample)], ...
          'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
end

% Draw seafloor plane surface
x_range = [min(plane_intersection_points(1,:))-2, max(plane_intersection_points(1,:))+2];
y_range = [min(plane_intersection_points(2,:))-2, max(plane_intersection_points(2,:))+2];
[X_plane, Y_plane] = meshgrid(linspace(x_range(1), x_range(2), 20), ...
                               linspace(y_range(1), y_range(2), 20));
% Plane equation: w_n(1)*x + w_n(2)*y + w_n(3)*z = w_n' * p_seafloor_NED
% Solve for z: z = (d - w_n(1)*x - w_n(2)*y) / w_n(3)
d_plane = w_n' * p_seafloor_NED;
Z_plane = (d_plane - w_n(1)*X_plane - w_n(2)*Y_plane) / w_n(3);
surf(X_plane, Y_plane, -Z_plane, 'FaceAlpha', 0.3, 'EdgeColor', 'none', ...
     'FaceColor', [0.8 0.8 0.6], 'DisplayName', 'Seafloor Plane');

% Draw plane normal vector from reference point (inverted for visualization)
quiver3(p_seafloor_NED(1), p_seafloor_NED(2), -p_seafloor_NED(3), ...
        w_n(1)*3, w_n(2)*3, -w_n(3)*3, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, ...
        'DisplayName', 'Plane Normal');

% Mark start and end points
plot3(eta1(1,1), eta1(2,1), -eta1(3,1), 'go', 'MarkerSize', 12, ...
      'MarkerFaceColor', 'g', 'LineWidth', 2, 'DisplayName', 'Start');
plot3(eta1(1,end), eta1(2,end), -eta1(3,end), 'rs', 'MarkerSize', 12, ...
      'MarkerFaceColor', 'r', 'LineWidth', 2, 'DisplayName', 'End');

colorbar;
colormap(jet);
xlabel('X [m]', 'FontSize', 12);
ylabel('Y [m]', 'FontSize', 12);
zlabel('Z [m] (up: standard convention)', 'FontSize', 12);
set(gca, 'ZDir', 'normal');  % Ensure Z points upward
title(sprintf('3D Visualization: Robot Trajectory and Plane Intersection\n(alpha=%.1f°, beta=%.1f°, h*=%.1fm)\nRobot ABOVE seafloor plane', ...
      rad2deg(alpha), rad2deg(beta), h_star), 'FontSize', 14);
legend('Location', 'best');
view(45, 30);
hold off;

% Surge and sway diagnostics
surge_err_series = u_star - nu1(1,:);
sway_err_series  = v_star - nu1(2,:);
fprintf('Surge final velocity: %.3f m/s (target %.3f)\n', nu1(1,end), u_star);
fprintf('Surge final error: %.3f m/s\n', surge_err_series(end));
fprintf('Sway final velocity:  %.3f m/s (target %.3f)\n', nu1(2,end), v_star);
fprintf('Sway final error:  %.3f m/s\n', sway_err_series(end));

figure;
plot(time, surge_err_series,'m','LineWidth',1.1); hold on;
plot(time, sway_err_series,'c','LineWidth',1.1);
legend('Surge error','Sway error','Location','best'); grid on;
xlabel('Time [s]'); ylabel('Velocity Error [m/s]'); title('Surge/Sway Errors'); hold off;

% Additional diagnostics: surge velocity tracking
figure;
subplot(2,1,1);
plot(time, nu1(1,:), 'b', 'LineWidth', 1.5); hold on;
yline(u_star, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Surge Velocity [m/s]');
title('Surge Velocity Tracking');
legend('Actual', 'Target', 'Location', 'best'); grid on;

subplot(2,1,2);
plot(time, h, 'b', 'LineWidth', 1.5); hold on;
yline(h_star, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Altitude h [m]');
title('Altitude Tracking');
legend('Actual', 'Target', 'Location', 'best'); grid on;

% Persist logs to MAT and CSV files
% logs.nu1 = nu1;              % velocities [3 x N]
% logs.a = a;              % accelerations [3 x N]
% logs.eta1 = eta1;            % positions [3 x N]
% logs.time = time;        % time vector [1 x N]
% logs.heave_alt = h;           % altitude [1 x N]
% logs.heave_err = -err(3,:);    % heave error [1 x N]
% logs.surge_err = surge_err_series;    % surge error [1 x N]
% logs.sway_err  = sway_err_series;     % sway error [1 x N]
% logs.plane_intersection_points = plane_intersection_points;  % intersection points [3 x N]
% logs.plane_normal = w_n;      % plane normal vector
% logs.plane_point = p_seafloor_NED;  % point on plane

% Create logs directory if it doesn't exist
% if ~exist('logs', 'dir')
%     mkdir('logs');
% end

% save('logs/controller_logs.mat','logs');

% CSV exports (one file per signal for quick inspection)
% writematrix([time.' nu1.'], 'logs/u_surgeswayheave.csv');
% writematrix([time.' a.'], 'logs/a_surgeswayheave.csv');
% writematrix([time.' h.'], 'logs/heave_alt.csv');
% writematrix([time.' -err(3,:).'], 'logs/heave_err.csv');
% writematrix([time.' surge_err_series.'], 'logs/surge_err.csv');
% writematrix([time.' sway_err_series.'], 'logs/sway_err.csv');
% writematrix([time.' plane_intersection_points.'], 'logs/plane_intersection_points.csv');

% fprintf('\n=== Data saved to logs/ directory ===\n');
% fprintf('Plane intersection points saved to:\n');
% fprintf('  - logs/controller_logs.mat (variable: logs.plane_intersection_points)\n');
% fprintf('  - logs/plane_intersection_points.csv\n');
% fprintf('Total intersection points: %d\n', N);

%% Gain computation
function [kp, ki, kd, kt] = gainComputation(speed0)
    wn = 0.4;
    damp = 0.6;
    p = 10;
    dim_i = 3;

    % Initialize gain vectors
    kp = zeros(dim_i, 1);
    ki = zeros(dim_i, 1);
    kd = zeros(dim_i, 1);
    Ti = zeros(dim_i, 1);   % Integral time constant
    Td = zeros(dim_i, 1);   % Derivative time constant
    kt = zeros(dim_i, 1);   % Anti-windup gain

    %% Model
    % massa totale [kg]
    m = 11.5; 
    % added mass
    tau_a = -[27.08; 25.952; 29.9081];
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130];
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780];   
    mv = m - tau_a;
    % Dissipative forces (con v0 = 0.1 solo nel surge)
    dv = -tau_r - tau_d.*abs(speed0);
    dv_lin = -tau_r - 2*tau_d.*abs(speed0);

    for l = 1:dim_i
        if l == 1 || l == 2
            % PI control for surge and sway (horizontal motion)
            % No derivative action needed for stable horizontal dynamics
            kp(l) = 2*damp*wn*mv(l) - dv_lin(l);
            ki(l) = wn^2 * mv(l);
            kd(l) = 0;
            Ti(l) = kp(l)/ki(l);
            kt(l) = 1/Ti(l);  % Anti-windup gain
        else
            % PID control for heave, roll, pitch, yaw
            % Derivative action helps with stability and faster response
            kp(l) = mv(l)*((wn^2) + 2*damp*p*wn);
            ki(l) = p*(wn^2)*mv(l);
            kd(l) = (p + 2*damp*wn)*mv(l) - dv_lin(l);
            Ti(l) = kp(l)/ki(l);
            Td(l) = kd(l)/kp(l);
            kt(l) = 2 / sqrt(Ti(l)*Td(l));  % Stronger anti-windup on heave
        end
    end
end

%% Local numerical helpers (fallback if external versions differ)
function y = integrator(prev_state, current_input, prev_input, Ts)
% Trapezoidal integration for smoother response
    y = prev_state + 0.5*(current_input + prev_input)*Ts;
end

function d = derivator(prev_deriv, current_signal, prev_signal, Ts)
% Backward difference with simple first-order filtering
    raw = (current_signal - prev_signal)/Ts;
    alpha = 0.2; % smoothing factor (0<alpha<=1)
    d = alpha*raw + (1-alpha)*prev_deriv;
end