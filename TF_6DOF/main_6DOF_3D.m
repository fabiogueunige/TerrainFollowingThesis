clc; clear; close all;

%% Add Paths
addpath('controller');
addpath('data_management');
addpath('ekf_position');
addpath('ekf_sbes');
addpath('ekf_sbes/noise');
addpath('math_function');
addpath('model');
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
Tf = 50;            % Final time [s]
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

%% Structure for position ekf_position
% mettere struttura per avere anche nel main le informazioni di ekf_position

%% Noise
[Q] = ekf_setup_sbes();

%% Software Design
global h_ref;
h_ref = zeros(1, N);
h_ref(:) = 3;

%% AUV Parameters 
% position & orientation 
eta = zeros(w_dim,N);            % robot position & orientation
eta_gt = zeros(w_dim,N);         % NO Noise Position & Orientation for real state

% velocity & angular rates
nu = zeros(i_dim, N);            % Known inputs
nu_gt = zeros(i_dim, N);         % NO Noise Known inputs for real state

nu_dot = zeros(i_dim,N);         % AUV acceleration
nu_dot_gt = zeros(i_dim,N);      % NO Noise AUV acceleration for real state

eta_dot = zeros(w_dim,N);        % robot velocities & angular rates

% robot rotations
wRr = zeros(d_dim, d_dim, N);       % Robot in world rotation
wRr_gt = zeros(d_dim, d_dim);  % Real robot rotation (ground truth)

% echosonar part
s = zeros(d_dim,s_dim);             % Robot echosonar

%% Terrain Parameters
max_planes = 300; % Circular buffer size
step_length = 1.0; % Distance between consecutive planes
angle_range = [-pi/6, pi/6];
delta_limit = pi/3;
rate_of_change = 3;
pp_init_w = [-5; -5; 10];
n0 = [0, 0, 1]';

% terrain generation
[plane, t_idx] = terrain_init(pp_init_w, eta(1:3,1), max_planes, step_length, n0, ...
                    angle_range, rate_of_change, delta_limit);

% terrain variable estimation
wRt = zeros(d_dim, d_dim, N);
wRt_pre = zeros(d_dim, d_dim, N);
n_pre = zeros(d_dim,N);                 % Surface vector predicted
n_est = zeros(d_dim, N);                % surface vector estimated
n_mes = zeros(d_dim, N);                % surface vector measured

%% Sensors readings
z_AHRS = zeros(6, N);
z_DVL = zeros(3, N);
z_PS = zeros(1, N);
[R_AHRS, R_DVL, R_PS, R_SBES] = setup_sensors();

%% EKF Parameters
x_pred = zeros(n_dim, N);                       % State predicted
x_est = zeros(n_dim, N);                        % Estimated state
x_true = zeros(n_dim, N);                       % True state
z_meas = zeros(m_dim, N);                       % Measurements
z_pred = zeros(m_dim, N);                       % Predicted output
R = repmat(R_SBES, 1, 1, N);                      % Observation matrix
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
[Kp, Ki, Kd] = gainComputation(speed0, i_dim);


%% EKF Start
% covariance 
P0 = diag([1, 0.08, 2]);
P0 = P0 * (1.1) + 2*rand;
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
    goal(k) = goal_def(state(k), eta(4:6,k-1), x_est(:,k-1), k);

    % if mod(k, 5000) == 0
    %     [Kp, Ki, Kd] = gainComputation(nu(:,k-1), i_dim);
    % end
    
    %% EKF: Input Control Computation
    if k >= 3
        % PID Values with Saturation
        [pid(:,k), t_sum(:,k)] = input_control(goal(k), x_est(:,k-1), eta(4:6,k-1), pid(:,k-1), nu(:,k-1), nu_dot(:,k-1), ...
                                t_sum(:,k-1), wRr(:,:,k-1), n_est(:,k-1), Ts, i_dim, Kp, Ki, Kd); 
    end
    %% EKF: Dynamic and Kinematic Model
    % Dynamic model  %% mettere i ground truth qui
    [nu_dot(:,k), nu(:,k)] = dynamic_model(pid(:,k), tau0, speed0, eta(4:6, k-1), nu(:,k-1), Ts, i_dim, nu_dot(:,k-1));
    [eta_dot(:,k), eta(:,k)] = kinematic_model(eta(:,k-1), nu(:,k), wRr(:,:,k-1), eta_dot(:,k-1), i_dim, Ts);
    
    % Update rotation matrix with NEW angles after integration
    wRr(:,:,k) = rotz(eta(YAW,k))*roty(eta(PITCH,k))*rotx(eta(ROLL,k));
    wRr_gt = wRr(:,:,k);
    
    %% Sensor readings
    z_AHRS(:,k) = read_AHRS(eta(4:6,k), nu(4:6,k), R_AHRS);
    z_DVL(:,k) = read_DVL(nu(1:3,k), R_DVL);
    z_PS(:,k) = read_PS(eta(1,k), R_PS);

    %% EKF Position Estimation
    % [] = ekf_position();

    %% Terrain Dynamic Update
    w_sp = wRr(:,:,k) * nu(1:3,k);
    [plane, t_idx] = terrain_generator(plane, eta(1:3,k), w_sp, t_idx, step_length, max_planes, ...
                        k, angle_range, rate_of_change, delta_limit);
    
    %% EKF: Real Measurement
    [z_meas(:,k), hmes, n_mes(:,k), R(:,:,k), cmd, a_true, b_true] = SBES_measurament(plane, s_dim, eta(1:3,k), wRr_gt, ...
                                            t_idx, R(:,:,k), cmd, k);
    
    %%%%%%%%%%%%%%% NO NOISE FOR SBES %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    v_sbes = mvnrnd(zeros(m_dim,1), R_SBES)'; 
    z_meas(:,k) = z_meas(:,k) + v_sbes;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    %% EKF: True State update (based on state)
    % TRUE state estimation
    x_true(:,k) = [hmes, a_true, b_true]';

    %% EKF: Prediction
    % State prediction
    w = mvnrnd(zeros(n_dim,1), Q)'; % Process noise
    [x_pred(:,k), wRt_pre(:,:,k)] = f(x_est(:,k-1), nu(:,k), Ts, wRr(:,:,k));
    %%%%%%%%%%%%%%% NO NOISE %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%
    x_pred(:,k) = x_pred(:,k) + w;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Dynamics Jacobian
    F = jacobian_f(x_est(:,k-1), nu(:,k), Ts, n_dim, wRr(:,:,k)); 
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
    cmd = goal_controller(cmd, x_est(:,k), eta(4:6,k), goal(k), N, state(k), k, d_dim);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\nGenerating plots and statistics...\n');
plot_results(time, N, h_ref, x_true, x_est, eta(4:6,:), eta_gt(4:6,:), goal, nu, ...
             n_est, n_mes, wRr, eta(1:3,:), n_dim, d_dim, i_dim);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Data Saving %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\n===========================================\n');
fprintf('SIMULATION COMPLETED SUCCESSFULLY!\n');
fprintf('===========================================\n');

% Prompt user for data saving
fprintf('\nWould you like to save simulation data for future statistical analysis?\n');
fprintf('Data will be saved in a timestamped folder (e.g., results/run_YYYYMMDD_HHMMSS/)\n');
save_choice = input('Save data? (Y/N): ', 's');

if strcmpi(save_choice, 'Y') || strcmpi(save_choice, 'Yes') || strcmpi(save_choice, 'y')
    fprintf('\n--- DATA SAVING ---\n');
    fprintf('Auto-generate run name (run_YYYYMMDD_HHMMSS)\n');
    % Auto-generate with timestamp
    run_name = '';  % Empty means auto-generate in save function

    % Collect all simulation data
    fprintf('\nCollecting simulation data...\n');
    sim_data = collect_simulation_data(time, Ts, Tf, N, h_ref, ...
        x_true, x_est, x_pred, ni, S, P, P0, ...
        z_meas, z_pred, n_mes, n_est, n_pre, eta(4:6,:), eta_gt(4:6,:), R, ...
        pid, nu, nu_dot, goal, integral_err, p_err, i_err, t_sum, ...
        eta(1:3,:), wRr, wRt, wRt_pre, state, ...
        Q, R_SBES, R_AHRS, Kp, Ki, Kd, 0, speed0, tau0, x0, x0_est, ...
        max_planes, step_length, angle_range, rate_of_change, delta_limit, pp_init_w, n0);
    
    % Save to disk
    if isempty(run_name)
        save_simulation_data(sim_data);
    else
        save_simulation_data(sim_data, run_name);
    end
    
    fprintf('\n✓ Data saved successfully!\n');
    fprintf('\nTo analyze saved runs, use:\n');
    fprintf('  >> stats = analyze_statistics();\n');
    fprintf('To load this run later, use:\n');
    if ~isempty(run_name)
        fprintf('  >> sim_data = load_simulation_data(''%s'');\n', run_name);
    else
        fprintf('  >> sim_data = load_simulation_data();\n');
    end
else
    fprintf('\nData not saved. Simulation complete.\n');
end

fprintf('\n===========================================\n');
fprintf('END OF SIMULATION\n');
fprintf('===========================================\n');
