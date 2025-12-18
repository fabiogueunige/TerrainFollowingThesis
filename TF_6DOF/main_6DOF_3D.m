clc; clear; close all;

%% Add Paths
addpath('controller');
addpath('data_management');
addpath('ekf_position');
addpath('ekf_sbes');
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
Ts = 0.001;         % Sampling time [s] - 1000 Hz
Tf = 100;            % Final time [s]
time = 0:Ts:Tf;     % Time vector
N = length(time);   % Number of iterations
global DEBUG;
DEBUG = false;

%% Sensor Update Frequencies
freq_AHRS = 100;    % AHRS frequency [Hz]
freq_DVL = 10;      % DVL frequency [Hz]
freq_PS = 20;       % Pressure Sensor frequency [Hz]
freq_SBES = 50;
% Calculate update periods in samples (1 kHz base)
period_AHRS = round(1000 / freq_AHRS);  % Update ogni 10 samples (100 Hz)
period_DVL = round(1000 / freq_DVL);    % Update ogni 100 samples (10 Hz)
period_PS = round(1000 / freq_PS);      % Update ogni 50 samples (20 Hz)
period_SBES = round(1000 / freq_SBES);

%% Matrix Dimensions
n_dim = 3;          % Number of states
i_dim = 6;          % Number of inputs 
d_dim = 3;          % world space total dimensions
s_dim = 4;          % number of echosonar
w_dim = 6;          % world dimension
t_dim = 1000;       % terrain sampling
dim_ekf = 15;      % EKF state dimension

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

%% Software Design
global h_ref;
h_ref = zeros(1, N);
h_ref(:) = 3;

%% AUV Parameters 
% position & orientation 
x_loc = zeros(dim_ekf, N);                  % EKF position state
x_loc_pre = zeros(dim_ekf, N);              % EKF predicted state
[Q_loc, P_loc_init] = stateLoc_init(dim_ekf);    % EKF covariance and process noise
P_loc = zeros(dim_ekf, dim_ekf, N);         % 3D array for covariance history
P_loc(:,:,1) = P_loc_init;                  % Initialize first slice
P_loc_pre = zeros(dim_ekf, dim_ekf, N);     % EKF predicted covariance
P_loc_pre(:,:,1) = P_loc_init;              % Initialize first slice

eta = zeros(w_dim,N);                       % robot position & orientation
eta_gt = zeros(w_dim,N);                    % NO Noise Position & Orientation for real state
% velocity & angular rates
nu = zeros(i_dim, N);            % Known inputs
nu_gt = zeros(i_dim, N);         % NO Noise Known inputs for real state
nu_dot = zeros(i_dim,N);         % AUV accelerationte
eta_dot_gt = zeros(w_dim,N);     % robot velocities & angular rates

% robot rotations
wRr = zeros(d_dim, d_dim, N);       % Robot in world rotation (from EKF)
wRr_gt = zeros(d_dim, d_dim, N);    % Real robot rotation (ground truth)

%% echosonar part SBES
s = zeros(d_dim,s_dim);             % Robot echosonar
[Q, P0] = stateSbes_init();       % Initial covariance and process noise
[R_SBES] = setupSBES_sensors();

%% Terrain Parameters
max_planes = 300; % Circular buffer size
step_length = 3; % Distance between consecutive planes
min_look_ahead = 8;
angle_range = [-pi/6, pi/6;
                -pi/7, pi/7];
delta_limit = pi/8;
rate_of_change = 5;
% Terrain must start BEHIND the robot and extend FORWARD
% Robot starts at (0,0,0), terrain direction is [1,1,0] normalized
% So terrain should start at negative X,Y to cover robot position
pp_init_w = [-10; -10; 10];
n0 = [0, 0, 1]';

% terrain generation
[plane, t_idx] = terrain_init(pp_init_w, eta(1:3,1), max_planes, step_length, n0, ...
                    angle_range, rate_of_change, delta_limit, min_look_ahead);

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
z_meas = zeros(s_dim, N);                       % Measurements
y_pred = zeros(s_dim, N);                       % Predicted output
R = repmat(R_SBES, 1, 1, N);                      % Observation matrix
x0 = [10, plane(1).alpha, plane(1).beta]';      % True initial state 
% Initialize estimate close to true state (realistic initial guess)
x0_est = [30, plane(1).alpha, plane(1).beta]';  % Use reference altitude and initial terrain angles
ni = zeros(s_dim, N);                           % Innovation
S = zeros(s_dim, s_dim, N);                     % Covariance Innovation
P_sbes = zeros(n_dim, n_dim, N);                % EKF SBES covariance history [3x3xN]
P_pred = zeros(n_dim, n_dim, N);                % EKF SBES predicted covariance
     
%% Controller Parameters
pid = zeros(i_dim, N);              % PID for Dynamics
integral_err = zeros(i_dim, N);     % integral error
p_err = zeros(i_dim, N);            % proportional error
i_err = zeros(i_dim, N);            % integral error
t_sum = zeros(i_dim,N);

% initial controller
speed0 = [0.2; 0; 0; 0; 0; 0];
[Kp, Ki, Kd] = gainComputation(speed0, i_dim);

%% EKF Start
% covariance 
P_sbes(:,:,1) = P0;  % Store initial covariance
P_pred(:,:,1) = P0;  % Store initial predicted covariance

% State
x_true(:,1) = x0;
x_est(:,1) = x0_est;

% Rotations
wRr(:,:,1) = eye(d_dim);
wRr_gt(:,:,1) = eye(d_dim);  % Initialize ground truth rotation
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
    % 
    %% EKF: Input Control Computation
    if k >= 3
        % PID Values with Saturation
        [pid(:,k), t_sum(:,k)] = input_control(goal(k), x_est(:,k-1), eta(4:6,k-1), pid(:,k-1), nu(:,k-1), nu_dot(:,k-1), ...
                                t_sum(:,k-1), wRr(:,:,k-1), n_est(:,k-1), Ts, i_dim, Kp, Ki, Kd); 
    end
    %% EKF: Dynamic and Kinematic Model (Ground Truth)
    % Dynamic model computes ground truth velocities
    [nu_dot(:,k), nu_gt(:,k)] = dynamic_model(pid(:,k), eta_gt(4:6, k-1), nu_gt(:,k-1), Ts, i_dim, nu_dot(:,k-1));
    % Kinematic model uses ground truth rotation wRr_gt
    [eta_dot_gt(:,k), eta_gt(:,k)] = kinematic_model(eta_gt(:,k-1), nu_gt(:,k), wRr_gt(:,:,k-1), eta_dot_gt(:,k-1), i_dim, Ts);
    % Update ground truth rotation matrix (stored in 3D array)
    wRr_gt(:,:,k) = rotz(eta_gt(YAW,k))*roty(eta_gt(PITCH,k))*rotx(eta_gt(ROLL,k));

    %% EKF Position estimation con update condizionali
    % Determina quali sensori devono essere aggiornati in questo ciclo
    update_AHRS = (mod(k, period_AHRS) == 0);  % Ogni 10 ms (100 Hz)
    update_DVL = (mod(k, period_DVL) == 0);    % Ogni 100 ms (10 Hz)
    update_PS = (mod(k, period_PS) == 0);      % Ogni 50 ms (20 Hz)
    update_SBES = (mod(k, period_SBES) == 0);  % Ogni 20 ms (50 Hz)
    
    % Chiamata EKF con flag di update per ogni sensore
    [x_loc(:,k), P_loc(:,:,k), wRr(:,:,k), x_loc_pre(:,k), P_loc_pre(:,:,k)] = ...
        ekf_position(x_loc(:,k-1), pid(:,k), wRr(:,:,k-1), nu_gt(:,k), eta_gt(:,k), ...
                    P_loc(:,:,k-1), Q_loc, Ts, P_loc_pre(:,:,k-1), x_loc_pre(:,k-1), update_DVL, update_AHRS, update_PS);
    eta(:,k) = x_loc(1:6,k);
    nu(:,k) = x_loc(7:12,k);
    %%%%%%%%%%% NO EKF %%%%%%%%%%%%%%%%%
    % eta(:,k) = eta_gt(:,k);
    % nu(:,k) = nu_gt(:,k);
    % wRr(:,:,k) = wRr_gt(:,:,k);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Terrain Dynamic Update (uses ground truth)
    w_sp = wRr_gt(:,:,k) * nu_gt(1:3,k);
    [plane, t_idx] = terrain_generator(plane, eta_gt(1:3,k), w_sp, t_idx, step_length, max_planes, ...
                        k, angle_range, rate_of_change, delta_limit, min_look_ahead);
    
    %% EKF: Real Measurement SBES
    [z_meas(:,k), hmes, n_mes(:,k), R(:,:,k), cmd, a_true, b_true] = SBES_measurament(plane, s_dim, eta_gt(1:3,k), wRr_gt(:,:,k), ...
                                            t_idx, R(:,:,k), cmd, k);

    %% EKF: True State update (based on state)
    % TRUE state estimation
    x_true(:,k) = [hmes, a_true, b_true]';

    %% EKF: Prediction
    % State prediction
    [x_pred(:,k), wRt_pre(:,:,k)] = f(x_pred(:,k-1), nu(:,k), Ts, wRr(:,:,k),  wRt_pre(:,:,k-1));
    
    % Dynamics Jacobian
    F = jacobian_f(x_pred(:,k-1), nu(:,k), Ts, n_dim, wRr(:,:,k)); 
    % Covariance prediction
    P_pred(:,:,k) = F * P_pred(:,:,k-1) * F' + Q;

    %% Sensors Computation
    s = SBES_definition(wRr(:,:,k));

    %% Norm to the terrain
    n_pre(:,k) = wRt_pre(:,:,k)*n0;
    if (norm(n_pre(:,k)) ~= 1)
        n_pre(:,k) = vector_normalization(n_pre(:,k));
    end

    if update_SBES
        %% EKF: Gain and Prediction Update
        % Measurament prediction
        y_pred(:,k) = h(x_pred(:,k), s, s_dim, s_dim, n_pre(:,k));  

        % Observation Jacobian
        H = jacobian_h(x_pred(:,k), s, s_dim, n_dim, s_dim, n_pre(:,k), n0);
        if any(isnan(H(:)))
            error('Esecuzione interrotta: H contiene valori NaN. Istante %0.f', k);
        end
        % Covariance Innovation
        S(:,:,k) = (H * P_pred(:,:,k) * H' + R(:,:,k));
        % Kalman Gain
        K = P_pred(:,:,k) * H' / S(:,:,k); % Kalman Gain
        if any(isnan(K(:)))
            error('Esecuzione interrotta: K contiene valori NaN. Istante %0.f', k);
        end    

        %% EKF: State Update
        % State Innovation
        ni(:,k) = (z_meas(:,k) - y_pred(:,k));
        
        % State Estimated
        x_est(:,k) = x_pred(:,k) + K * ni(:,k); 
        % Wrap estimated terrain angles to [-pi, pi]
        x_est(ALPHA,k) = wrapToPi(x_est(ALPHA,k));
        x_est(BETA,k) = wrapToPi(x_est(BETA,k));
        % Covariance
        P_sbes(:,:,k) = (eye(n_dim) - K * H) * P_pred(:,:,k);

        %% Rotation Update 
        % terrain rotation
        wRt(:,:,k) = rotz(0)*roty(x_est(BETA, k))*rotx(x_est(ALPHA,k))*rotx(pi);

        %% Check for Z direction
        n_est(:,k) = wRt(:,:,k)*n0;
        if (norm(n_est(:,k)) ~= 1)
            n_est(:,k) = vector_normalization(n_est(:,k));
        end
        [x_est(ALPHA,k), x_est(BETA, k)] = reference_correction(n_est(:,k),x_est(ALPHA,k), x_est(BETA,k));

        %% Store predicted state and covariance
        x_pred(:,k) = x_est(:,k);
        P_pred(:,:,k) = P_sbes(:,:,k);
        wRt_pre(:,:,k) = wRt(:,:,k);
    else
        x_est(:,k) = x_est(:,k-1);
        P_sbes(:,:,k) = P_sbes(:,:,k-1);
        wRt(:,:,k) = wRt(:,:,k-1);
        n_est(:,k) = n_est(:,k-1);
    end

    %% State Machine Check of Results
    cmd = goal_controller(cmd, x_est(:,k), eta(4:6,k), goal(k), N, state(k), k, d_dim);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\nGenerating plots and statistics...\n');
plot_results(time, N, h_ref, x_true, x_est, eta(4:6,:), eta_gt(4:6,:), goal, nu, ...
             n_est, n_mes, wRr, eta(1:3,:), n_dim, d_dim, i_dim, ...
             x_loc, eta_gt, nu_gt);

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
        x_true, x_est, x_pred, ni, S, P0, ...
        z_meas, y_pred, n_mes, n_est, n_pre, eta(4:6,:), eta_gt(4:6,:), R, ...
        pid, nu, nu_dot, goal, integral_err, p_err, i_err, t_sum, ...
        eta(1:3,:), wRr, wRt, wRt_pre, state, ...
        Q, R_SBES, Kp, Ki, Kd, speed0, x0, x0_est, ...
        max_planes, step_length, angle_range, rate_of_change, delta_limit, pp_init_w, n0, ...
        x_loc, eta_gt, nu_gt, wRr_gt, P_sbes, P_loc, P_loc_init, Q_loc);
    
    % Save to disk
    if isempty(run_name)
        [saved_run_name, run_dir] = save_simulation_data(sim_data);
    else
        [saved_run_name, run_dir] = save_simulation_data(sim_data, run_name);
    end
    
    % Export all figures as PNG
    fprintf('\nExporting figures to PNG...\n');
    all_figs = findall(0, 'Type', 'figure');
    if ~isempty(all_figs)
        fig_dir = fullfile(run_dir, 'figures');
        if ~exist(fig_dir, 'dir')
            mkdir(fig_dir);
        end
        
        for i = 1:length(all_figs)
            fig = all_figs(i);
            % Get figure name or use number
            if ~isempty(fig.Name)
                fig_name = fig.Name;
                % Remove invalid filename characters
                fig_name = regexprep(fig_name, '[^\w\s-]', '_');
                fig_name = regexprep(fig_name, '\s+', '_');
            else
                fig_name = sprintf('Figure_%d', fig.Number);
            end
            
            output_file = fullfile(fig_dir, [fig_name, '.png']);
            try
                exportgraphics(fig, output_file, 'Resolution', 300);
                fprintf('  ✓ Saved: %s\n', [fig_name, '.png']);
            catch ME
                warning('Could not save figure %s: %s', fig_name, ME.message);
            end
        end
        fprintf('\n✓ All figures exported to: %s\n', fig_dir);
    else
        fprintf('  No figures to export.\n');
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

    analyze_single_run();
else
    fprintf('\nData not saved. Simulation complete.\n');
end

fprintf('\n===========================================\n');
fprintf('END OF SIMULATION\n');
fprintf('===========================================\n');
