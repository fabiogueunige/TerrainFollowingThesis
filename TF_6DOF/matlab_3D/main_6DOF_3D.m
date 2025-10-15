clc; clear; close all;
addpath('controller');
addpath('ekf_filter');
addpath('kf_filter');
addpath('math_function');
addpath('model');
addpath('noise');
addpath('rotation');
addpath('sensors');
addpath('state_machine');
addpath('visualization');
addpath('world_generator');

%% Global Variables Definition
IND_H = 1;      ALPHA = 2;      BETA = 3;  
global PHI; global THETA; global PSI;          
PHI = 1;        THETA = 2;      PSI = 3;  
global SURGE; global SWAY; global HEAVE;
SURGE = 1;  SWAY = 2;   HEAVE = 3;
global ROLL; global PITCH; global YAW;
ROLL = 4;   PITCH = 5;  YAW = 6;

%% Simulation Parameters
Ts = 0.001;         % Sampling time [s]
Tf = 40;            % Final time [s]
time = 0:Ts:Tf;     % Time vector
N = length(time);   % Number of iterations
global DEBUG;
DEBUG = false;

%% State Machine Definition
cmd.start = false;          % Start command
cmd.setpoint = false;       % Setpoint command
cmd.reset = false;          % Angles to 0
cmd.contact1 = false;       % Contact point 1
cmd.contact2 = false;       % Contact point 2
cmd.contact3 = false;       % Contact point 3
cmd.contact4 = false;       % Contact point 4
cmd.sensor_fail = 0;        % Number of failure
cmd.end = false;            % End command
cmd.emergency = false;      % Emergency command
state = strings(1, N);      % State vector
state(1) = 'Idle';          % Initial state

% Goal structure definition
i_goal.surge = 0;
i_goal.sway = 0;
i_goal.altitude = 0;
i_goal.roll = 0;
i_goal.pitch = 0;
i_goal.yaw = 0; 
goal(1:N) = i_goal;

%% Matrix Dimensions
n_dim = 3;          % Number of states
m_dim = 4;          % Number of measurements
i_dim = 6;          % Number of inputs 
d_dim = 3;          % world space total dimensions
s_dim = 4;          % number of echosonar
w_dim = 6;          % world dimension

%% Noise
[Q, R_tp, R_a] = noise_setup(n_dim, m_dim, d_dim);

%% Software Design
index_N = round(N/2);
global h_ref;
h_ref = zeros(1, N);
%%%%%%%%% CHANGE ALTITUDE GOAL %%%%%%%%%%%
% index_N = round(N/2);
% h_ref(1:index_N) = ;
% h_ref(index_N:end) = 4;
%%%%%%%%% NO CHANGE %%%%%%%%%%%%%%%%%%%%%%
h_ref(:) = 3; % real goal 1:3

%% Terrain Parameters
ext_m_p = 12; % Initial dynamic allocation
step_length = 0.8; % Distance between consecutive planes
n0 = [0, 0, 1]';
pp_init = [-4; 4; -5];

% terrain generation
plane = terrain_init(pp_init, ext_m_p, N, step_length);

% terrain variable estimation
wRt = zeros(d_dim, d_dim, N);
wRt_pre = zeros(d_dim, d_dim, N);
n_pre = zeros(d_dim,N);                 % Surface vector predicted
n_est = zeros(d_dim, N);                % surface vector estimated
n_mes = zeros(d_dim, N);                % surface vector measured

%% AUV Parameters 
prob = zeros(d_dim,N);              % AUV initial position

% robot velocities & acceleration
u = zeros(i_dim, N);                % Known inputs
u_dot = zeros(i_dim,N);             % AUV acceleration

% robot angles
rob_rot = zeros(d_dim, N);          % Robot angles roll, pitch, yaw
clean_rot = zeros(d_dim,N);         % NO Noise Rotation for real state

% robot rotations
wRr = zeros(d_dim, d_dim, N);       % Robot in world rotation
wRr_real = zeros(d_dim, d_dim);     % Real robot rotation

% echosonar part
s = zeros(d_dim,s_dim);             % Robot echosonar

%% EKF Parameters
x_pred = zeros(n_dim, N);                       % State predicted
x_est = zeros(n_dim, N);                        % Estimated state
x_true = zeros(n_dim, N);                       % True state
z_meas = zeros(m_dim, N);                       % Measurements
z_pred = zeros(m_dim, N);                       % Predicted output
R = repmat(R_tp, 1, 1, N);                      % Observation matrix
x0 = [10, plane(1).alpha, plane(1).beta]';      % True initial state 
x0_est = zeros(n_dim, 1);                       % Estimated initial state
ni = zeros(m_dim, N);                           % Innovation
S = zeros(m_dim, m_dim, N);                     % Covariance Innovation
     
%% Controller Parameters
pid = zeros(i_dim, N);              % PID for Dynamics
integral_err = zeros(i_dim, N);     % integral error
p_err = zeros(i_dim, N);            % proportional error
i_err = zeros(i_dim, N);            % integral error
t_sum = zeros(i_dim,N);

% initial controller
speed0 = [0.2; 0; 0; 0; 0; 0];
tau0 = tau0_values(speed0, i_dim);
[Kp, Ki, Kd, Kt] = gainComputation(speed0, i_dim);

%% EKF Start
% covariance 
P0 = diag([1, 0.08, 2]);
P0 = P0 * (1.1) + 5*rand;
P = P0;

% State
x_true(:,1) = x0;
x_est(:,1) = x0_est;

% Rotations
wRr(:,:,1) = eye(d_dim);
wRt(:,:,1) = eye(d_dim) * rotx(pi);
wRt_pre(:,:,1) = wRt(:,:,1);

cmd.start = true;

%% EKF Simulation && State Machine
for k = 2:N
    if k == 2 || mod(k, 1000) == 0
        disp(['Processing iteration \n', num2str(k)]);
    end
    %% State Machine %%
    % no change in commands during the State Machine
    state(k) = state_machine(state(k-1), cmd, k);
    goal(k) = goal_def(state(k), rob_rot(:,k-1), x_est(:,k-1), k);
    
    %% EKF: Input Control Computation
    if k >= 3
        % PID Values with Saturation
        [pid(:,k), integral_err(:,k), p_err(:,k), i_err(:,k), u_dot(:,k), t_sum(:,k)] = input_control(goal(k), x_est(:,k-1), rob_rot(:,k-1), pid(:,k-1), integral_err(:,k-1), ...
                         u(:,k-1), u(:,k-2), u_dot(:,k-1), t_sum(:,k-1), wRr(:,:,k-1), wRt(:,:,k-1), Ts, i_dim, p_err(:,k-1), i_err(:,k-1),  Kp, Ki, Kd, Kt);   
    end
    %% EKF: Dynamic and Kinematic Model
    % Dynamic model
    [u(:,k)] = dynamic_model(pid(:,k), tau0, speed0, rob_rot(:, k-1), u(:,k-1), Ts, i_dim, u_dot(:,k-1));
    % [] = kinematic_model();
    
    %% KF -> robot pos and rot update
    % noise of sensors
    
    

    % sensors measurament
    [clean_rot(:,k), wRr_real] = AHRS_measurement(clean_rot(:,k-1), u(:,k), Ts);
    %%%%%%%%%%%%%%% NO NOISE FOR AHRS %%%%%%%%%%%%%%%%%%%%%%%
    % rob_rot(:,k) = clean_rot(:,k);                        %
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    v_ahrs = mvnrnd(zeros(d_dim,1), R_a)';
    rob_rot(:,k) = clean_rot(:,k) + v_ahrs;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [dvl_speed, prob(:,k)] = DVL_measurament(prob(:,k-1), u(:,k), wRr_real, Ts);
    %%%%%%%%%%%%%%% NO NOISE FOR DVL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % I do not have a localization problem -> needed only for the sensors %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Kalman Filter for best estimation
    % [] = kf_estimation();

    % Robot rotation matrix computation
    wRr(:,:,k) = rotz(rob_rot(PSI,k))*roty(rob_rot(THETA,k))*rotx(rob_rot(PHI,k));

    %% Terrain Dynamic Update
    t_it = k + ext_m_p;
    [plane(t_it)] = terrain_generator(plane(t_it - 1), dvl_speed, k, step_length);
    
    %% EKF: Real Measurement
    [z_meas(:,k), hmes, n_mes(:,k), R(:,:,k), cmd] = SBES_measurament(plane, s_dim, prob(:,k), wRr_real, ...
                                            k, R(:,:,k), cmd, ext_m_p);
    
    %%%%%%%%%%%%%%% NO NOISE FOR SBES %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    v_sbes = mvnrnd(zeros(m_dim,1), R_tp)'; 
    z_meas(:,k) = z_meas(:,k) + v_sbes;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    %% EKF: True State update (based on state)
    % TRUE state estimation   !!! Va costruito il piano in base a inidici che si salva per i diversi sensori
    % x_true(:,k) = [hmes, plane.alpha, plane.beta]';

    %% EKF: Prediction
    % State prediction
    w = mvnrnd(zeros(n_dim,1), Q)'; % Process noise
    [x_pred(:,k), wRt_pre(:,:,k)] = f(x_est(:,k-1), u(:,k), Ts, wRr(:,:,k));
    %%%%%%%%%%%%%%% NO NOISE %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%
    x_pred(:,k) = x_pred(:,k) + w;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Dynamics Jacobian
    F = jacobian_f(x_est(:,k-1), u(:,k), Ts, n_dim, wRr(:,:,k)); 
    % Covariance prediction
    P_pred = F * P * F' + Q;

    %% Sensors Computation
    s = SBES_definition(wRr(:,:,k));

    %% Norm to the terrain
    n_pre(:,k) = wRt_pre(:,:,k)*n0;
    if (norm(n_pre(:,k)) ~= 1)
        n_pre(:,k) = vector_normalization(n_pre(:,k));
    end
   
    %% EKF: Gain and Prediction Update
    % Observation Jacobian
    H = jacobian_h(x_pred(:,k), s, m_dim, n_dim, s_dim, n_pre(:,k), n0);
    if any(isnan(H(:)))
        error('Esecuzione interrotta: H contiene valori NaN. Istante %0.f', k);
    end
    % Covariance Innovation
    S(:,:,k) = (H * P_pred * H' + R(:,:,k));
    % Kalman Gain
    K = P_pred * H' / S(:,:,k); % Kalman Gain
    if any(isnan(K(:)))
        error('Esecuzione interrotta: K contiene valori NaN. Istante %0.f', k);
    end
    
    % Measurament prediction
    z_pred(:,k) = h(x_pred(:,k), s, s_dim, m_dim, n_pre(:,k));      

    %% EKF: State Update
    % State Innovation
    ni(:,k) = (z_meas(:,k) - z_pred(:,k));
    % State Estimated
    x_est(:,k) = x_pred(:,k) + K * ni(:,k);
    % Covariance
    P = (eye(n_dim) - K * H) * P_pred;

    %% Rotation Update 
    % terrain rotation
    wRt(:,:,k) = rotz(0)*roty(x_est(BETA, k))*rotx(x_est(ALPHA,k))*rotx(pi);

    %% Check for Z direction
    n_est(:,k) = wRt(:,:,k)*n0;
    if (norm(n_est(:,k)) ~= 1)
        n_est(:,k) = vector_normalization(n_est(:,k));
    end
    [x_est(ALPHA,k), x_est(BETA, k)] = reference_correction(n_est(:,k),x_est(ALPHA,k), x_est(BETA,k));

    %% State Machine Check of Results
    cmd = goal_controller(cmd, x_est(:,k), rob_rot(:,k), goal(k), N, state(k), k, d_dim);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% States
ttl = {'altitude', 'alpha', 'beta'};
for i = 1:n_dim
    figure;
    if (i == 1)
        plot(time, h_ref(:), 'b', 'DisplayName', 'Desired')
        hold on;
        plot(time, x_true(i,:), 'r', 'DisplayName', 'True');
        plot(time, x_est(i,:), 'g', 'DisplayName', 'Estimated');
    else
        plot(time, rad2deg(rob_rot(i-1,:)), 'r', 'DisplayName', 'Rob angle');
        hold on;
        plot(time, rad2deg(x_true(i,:)), 'b', 'DisplayName', 'True Terrain');
        plot(time, rad2deg(x_est(i,:)), 'g', 'DisplayName', 'Estimated');
    end
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
    legend; grid on;
    title(ttl{i})
    hold off;
end

%% Robot angles
ttl = {'roll', 'pitch', 'yaw'};
for i = 1:d_dim
    figure;
    plot(time, rad2deg(clean_rot(i,:)), 'r', 'DisplayName', 'True');
    hold on;
    if i == 1
        plot(time, rad2deg([goal.roll]), 'b', 'DisplayName', 'Goal');
    end
    if i == 2
        plot(time, rad2deg([goal.pitch]), 'b', 'DisplayName', 'Goal');
    end
    plot(time, rad2deg(rob_rot(i,:)), 'g', 'DisplayName', 'Estimated');
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
    legend; grid on;
    title(ttl{i})
    hold off;
end

%% Inputs
ttl = {'u input', 'v input', 'w input', 'p input', 'q input', 'r input'};
for i = 1:i_dim
    figure;
    if i <= HEAVE
        plot(time, u(i,:), 'b', 'DisplayName', 'u');
    else
        plot(time, rad2deg(u(i,:)), 'b', 'DisplayName', ttl{i});
    end
    hold on
    xlabel('Time [s]'); ylabel('Space [m]');
    grid on;
    title(ttl{i})
    hold off;
end

%% Z axis i f parallel
% tutto riferito a world frame
% aa12 = acosd(abs(dot(n_est', n_mes')))'; % tra n estimato e n da misure
% aa13 = acosd(abs(dot(n_est', n')))'; % tra n estimato e n rob
% aa23 = acosd(abs(dot(n_mes', n3')))'; % tra n da misure e n rob
% 
% figure; hold on; grid on;
% plot(time, aa12, 'r', 'LineWidth', 1.5);
% plot(time, aa13, 'g', 'LineWidth', 1.5);
% plot(time, aa23, 'b', 'LineWidth', 1.5);
% 
% xlabel('Tempo [s]');
% ylabel('Angolo tra normali [°]');
% title('Parallelismo tra le normali dei piani nel tempo');
% legend('\theta_{12}','\theta_{13}','\theta_{23}');
% ylim([0 10]); % adatta in base ai tuoi dati
% hold off;
% grid off;

%% Points
figure;
scatter3(prob(1,:), prob(2,:), prob(3,:), [], time);
colorbar; 
colormap(jet);
xlabel('On X');
ylabel('On Z');
title('Trajectory XYZ (Color = time)');
grid on;
