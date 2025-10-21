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

%%  Global Variables Definition
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

%% Matrix Dimensions
n_dim = 3;          % Number of states
m_dim = 4;          % Number of measurements
i_dim = 6;          % Number of inputs 
d_dim = 3;          % world space total dimensions
s_dim = 4;          % number of echosonar
w_dim = 6;          % world dimension
t_dim = 1000;       % terrain sampling

%% State Machine Definition
cmd.start = false;              % Start command
cmd.setpoint = false;           % Setpoint command
cmd.reset = false;              % Angles to 0
cmd.contact = false(s_dim,1);   % Contact points
cmd.sensor_fail = 0;            % Number of failure
cmd.end = false;                % End command
cmd.emergency = false;          % Emergency command

% New fields for improved sensor diagnostics and recovery
cmd.pitch_sensors_lost = false;     % Sensors 1-2 (pitch pair) both lost
cmd.roll_sensors_lost = false;      % Sensors 3-4 (roll pair) both lost
cmd.diagonal_sensors_lost = false;  % Diagonal sensors lost (1-4 or 2-3)
cmd.sensor_fail_persistent = false; % Sensor failure persisted beyond grace period
cmd.recovery_timeout = false;       % Recovery maneuver timed out
cmd.recovery_progress = false;      % Recovery showing improvement

state = strings(1, N);          % State vector
state(1) = 'Idle';              % Initial state

% Goal structure definition
i_goal.surge = 0;
i_goal.sway = 0;
i_goal.altitude = 0;
i_goal.roll = 0;
i_goal.pitch = 0;
i_goal.yaw = 0; 
goal(1:N) = i_goal;

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

%% Terrain Parameters
max_planes = 500; % Circular buffer size
step_length = 4; % Distance between consecutive planes
n0 = [0, 0, 1]';
pp_init_w = [-8; -8; 15];

% terrain generation
[plane, t_idx] = terrain_init(pp_init_w, prob(:,1), max_planes, step_length, n0);

% terrain variable estimation
wRt = zeros(d_dim, d_dim, N);
wRt_pre = zeros(d_dim, d_dim, N);
n_pre = zeros(d_dim,N);                 % Surface vector predicted
n_est = zeros(d_dim, N);                % surface vector estimated
n_mes = zeros(d_dim, N);                % surface vector measured

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
speed0 = [0.2; 0.1; 0; 0; 0; 0];
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
    % sensors measurament
    [clean_rot(:,k), wRr_real] = AHRS_measurement(clean_rot(:,k-1), u(:,k), Ts);
    %%%%%%%%%%%%%%% NO NOISE FOR AHRS %%%%%%%%%%%%%%%%%%%%%%%
    % rob_rot(:,k) = clean_rot(:,k);                        %
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    v_ahrs = mvnrnd(zeros(d_dim,1), R_a)';
    rob_rot(:,k) = clean_rot(:,k) + v_ahrs;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [dvl_speed_w, prob(:,k)] = DVL_measurament(prob(:,k-1), u(:,k), wRr_real, Ts);
    %%%%%%%%%%%%%%% NO NOISE FOR DVL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % I do not have a localization problem -> needed only for the sensors %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Kalman Filter for best estimation
    % [] = kf_estimation();

    % Robot rotation matrix computation
    wRr(:,:,k) = rotz(rob_rot(PSI,k))*roty(rob_rot(THETA,k))*rotx(rob_rot(PHI,k));

    %% Terrain Dynamic Update
    [plane, t_idx] = terrain_generator(plane, prob(:,k), dvl_speed_w, t_idx, step_length, max_planes, k);
    
    %% EKF: Real Measurement
    [z_meas(:,k), hmes, n_mes(:,k), R(:,:,k), cmd, a_true, b_true] = SBES_measurament(plane, s_dim, prob(:,k), wRr_real, ...
                                            t_idx, R(:,:,k), cmd, k);
    
    %%%%%%%%%%%%%%% NO NOISE FOR SBES %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    v_sbes = mvnrnd(zeros(m_dim,1), R_tp)'; 
    z_meas(:,k) = z_meas(:,k) + v_sbes;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    %% EKF: True State update (based on state)
    % TRUE state estimation   !!! Va costruito il piano in base a inidici che si salva per i diversi sensori
    x_true(:,k) = [hmes, a_true, b_true]';

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
    % if n_est(3,k) > 0
    %     warning('Normal still pointing upward at iteration %d!', k);
    % end

    %% State Machine Check of Results
    cmd = goal_controller(cmd, x_est(:,k), rob_rot(:,k), goal(k), N, state(k), k, d_dim);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\nGenerating plots and statistics...\n');
plot_results(time, N, h_ref, x_true, x_est, rob_rot, clean_rot, goal, u, ...
             n_est, n_mes, wRr, prob, n_dim, d_dim, i_dim);
