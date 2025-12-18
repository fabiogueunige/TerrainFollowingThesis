%% COLLECT_SIMULATION_DATA - Collect all simulation variables into structure
%
% Collects all important simulation data into a single structure for saving.
% This function should be called at the end of the simulation loop.
%
% SYNTAX:
%   sim_data = collect_simulation_data(time, Ts, Tf, N, h_ref, ...
%       x_true, x_est, x_pred, ni, S, P, P0, ...
%       z_meas, z_pred, n_mes, n_est, n_pre, rob_rot, clean_rot, R, ...
%       pid, u, u_dot, goal, integral_err, p_err, i_err, t_sum, ...
%       prob, wRr, wRt, wRt_pre, state, ...
%       Q, R_SBES, Kp, Ki, Kd, speed0, x0, x0_est, ...
%       max_planes, step_length, angle_range, rate_of_change, delta_limit, pp_init_w, n0, ...
%       x_loc, eta_gt, nu_gt, wRr_gt, P_sbes, P_loc, P_loc_init, Q_loc)
%
% INPUTS:
%   All simulation variables (see main_6DOF_3D.m for definitions)
%   P_sbes - EKF SBES covariance history [3 x 3 x N] (optional, for NEES analysis)
%
% OUTPUTS:
%   sim_data - Structure containing all simulation data organized by category:
%              .time, .Ts, .Tf, .N              - Time parameters
%              .x_true, .x_est, .x_pred, .h_ref - EKF SBES states
%              .ni, .S, .P_sbes, .P_final, .P0  - EKF SBES covariance
%              .z_meas, .z_pred, .n_mes, ...    - Sensor data
%              .pid, .u, .u_dot, .goal, ...     - Control data
%              .prob, .wRr, .wRt, .state        - Trajectory data
%              .Q, .R_SBES, .Kp, ...            - Parameters
%              .x_loc, .P_loc, .Q_loc           - EKF Position filter data
%              .eta_gt, .nu_gt, .wRr_gt         - Ground truth data
%
% EXAMPLE:
%   % At end of simulation
%   sim_data = collect_simulation_data(time, Ts, Tf, N, h_ref, ...
%       x_true, x_est, x_pred, ni, S, P, P0, ..., P_sbes, P_loc, P_loc_init, Q_loc);
%   save_simulation_data(sim_data);
%
% See also: save_simulation_data, load_simulation_data

function sim_data = collect_simulation_data(time, Ts, Tf, N, h_ref, ...
    x_true, x_est, x_pred, ni, S,  P0, ...
    z_meas, z_pred, n_mes, n_est, n_pre, rob_rot, clean_rot, R, ...
    pid, u, u_dot, goal, integral_err, p_err, i_err, t_sum, ...
    prob, wRr, wRt, wRt_pre, state, ...
    Q, R_SBES, Kp, Ki, Kd, speed0, x0, x0_est, ...
    max_planes, step_length, angle_range, rate_of_change, delta_limit, pp_init_w, n0, ...
    x_loc, eta_gt, nu_gt, wRr_gt, varargin)

    fprintf('\n=== COLLECTING SIMULATION DATA ===\n');
    
    %% Parse optional arguments for backward compatibility
    P_sbes = [];
    P_loc = [];
    P_loc_init = [];
    Q_loc = [];
    
    if nargin >= 48 && ~isempty(varargin)
        if length(varargin) >= 1, P_sbes = varargin{1}; end      % EKF SBES covariance history
        if length(varargin) >= 2, P_loc = varargin{2}; end       % EKF Position covariance history
        if length(varargin) >= 3, P_loc_init = varargin{3}; end  % EKF Position initial covariance
        if length(varargin) >= 4, Q_loc = varargin{4}; end       % EKF Position process noise
    end
    
    %% Data Version (for backward compatibility)
    sim_data.data_version = 2.0;  % Versioning for future compatibility
    
    %% Time Parameters
    sim_data.time = time;
    sim_data.t = time;  % Alias for compatibility
    sim_data.Ts = Ts;
    sim_data.Tf = Tf;
    sim_data.N = N;
    
    %% ========================================================================
    %% EKF SBES (Terrain Following Filter) States
    %% ========================================================================
    fprintf('Collecting EKF SBES states...\n');
    sim_data.h_ref = h_ref;
    sim_data.x_true = x_true;           % True terrain state [h, alpha, beta]
    sim_data.x_est = x_est;             % Estimated terrain state
    sim_data.x_pred = x_pred;           % Predicted terrain state
    
    %% EKF SBES Covariance and Innovation
    fprintf('Collecting EKF SBES covariance data...\n');
    sim_data.ni = ni;                   % Innovation [4 x N]
    sim_data.S = S;                     % Innovation covariance [4 x 4 x N]
    sim_data.P0 = P0;                   % Initial state covariance
    sim_data.Q_sbes = Q;                % Process noise covariance (EKF SBES)
    
    % EKF SBES covariance history for NEES analysis
    if ~isempty(P_sbes)
        sim_data.P_sbes = P_sbes;       % Full covariance history [3 x 3 x N]
        fprintf('  - P_sbes covariance history saved [3x3x%d]\n', size(P_sbes, 3));
    end
    
    %% ========================================================================
    %% Sensor Data
    %% ========================================================================
    fprintf('Collecting sensor data...\n');
    sim_data.z_meas = z_meas;           % SBES measurements [4 x N]
    sim_data.z_pred = z_pred;           % Predicted measurements [4 x N]
    sim_data.n_mes = n_mes;             % Measured normals [3 x N]
    sim_data.n_est = n_est;             % Estimated normals [3 x N]
    sim_data.n_pre = n_pre;             % Predicted normals [3 x N]
    sim_data.rob_rot = rob_rot;         % Robot angles with noise [3 x N]
    sim_data.clean_rot = clean_rot;     % Robot angles ground truth [3 x N]
    sim_data.R = R;                     % Measurement covariance [4 x 4 x N]
    sim_data.R_SBES = R_SBES;           % SBES noise template
    
    % Compute sensor failure history from measurements
    fprintf('Computing sensor failure statistics...\n');
    sensor_fail = zeros(1, N);
    for k = 1:N
        % Count sensors with NaN or infinite measurements
        sensor_fail(k) = sum(isnan(z_meas(:,k)) | isinf(z_meas(:,k)));
    end
    sim_data.sensor_fail = sensor_fail;
    
    %% ========================================================================
    %% Control Data
    %% ========================================================================
    fprintf('Collecting control data...\n');
    sim_data.pid = pid;                 % PID output [6 x N]
    sim_data.u = u;                     % Body velocities [6 x N]
    sim_data.u_dot = u_dot;             % Body accelerations [6 x N]
    sim_data.goal = goal;               % Goal structure array [1 x N]
    sim_data.integral_err = integral_err;   % Integral error [6 x N]
    sim_data.p_err = p_err;             % Proportional error [6 x N]
    sim_data.i_err = i_err;             % Integral error term [6 x N]
    sim_data.t_sum = t_sum;             % Anti-windup term [6 x N]
    
    % Controller gains
    sim_data.Kp = Kp;
    sim_data.Ki = Ki;
    sim_data.Kd = Kd;
    sim_data.speed0 = speed0;
    
    % Extract goal arrays for easier analysis
    sim_data.goal_surge = [goal.surge];
    sim_data.goal_sway = [goal.sway];
    sim_data.goal_altitude = [goal.altitude];
    sim_data.goal_roll = [goal.roll];
    sim_data.goal_pitch = [goal.pitch];
    sim_data.goal_yaw = [goal.yaw];
    
    %% ========================================================================
    %% Trajectory Data
    %% ========================================================================
    fprintf('Collecting trajectory data...\n');
    sim_data.prob = prob;               % Robot position from EKF [3 x N]
    sim_data.wRr = wRr;                 % Robot rotation from EKF [3 x 3 x N]
    sim_data.wRt = wRt;                 % Terrain rotation [3 x 3 x N]
    sim_data.wRt_pre = wRt_pre;         % Predicted terrain rotation [3 x 3 x N]
    
    %% ========================================================================
    %% State Machine Data
    %% ========================================================================
    fprintf('Collecting state machine data...\n');
    sim_data.state = state;             % State string array [1 x N]
    
    % Compute state transitions
    state_numeric = zeros(1, N);
    state_names = {'Idle', 'Reset', 'TargetAltitude', 'ContactSearch', ...
                   'MovePitch', 'MoveRoll', 'RecoveryAltitude', 'Following', ...
                   'Emergency', 'EndSimulation'};
    for k = 1:N
        idx = find(strcmp(state_names, state(k)));
        if ~isempty(idx)
            state_numeric(k) = idx;
        else
            state_numeric(k) = 0;
        end
    end
    sim_data.state_numeric = state_numeric;
    sim_data.state_names = state_names;
    
    % Count state transitions
    state_transitions = find(diff(state_numeric) ~= 0);
    sim_data.state_transitions = state_transitions;
    sim_data.num_transitions = length(state_transitions);
    
    % Time in each state
    state_time = zeros(1, length(state_names));
    for s = 1:length(state_names)
        state_time(s) = sum(state_numeric == s) * Ts;
    end
    sim_data.state_time = state_time;
    sim_data.state_percentage = state_time / Tf * 100;
    
    % Note: cmd_history is not currently collected from main_6DOF_3D.m
    % This field is reserved for future implementation of command history logging
    
    %% ========================================================================
    %% EKF Position Filter Data
    %% ========================================================================
    fprintf('Collecting EKF position filter data...\n');
    sim_data.x_loc = x_loc;             % Full EKF position state [15 x N]
    
    % Position and orientation from EKF
    sim_data.eta_ekf = x_loc(1:6, :);   % Position & orientation from EKF [6 x N]
    sim_data.nu_ekf = x_loc(7:12, :);   % Velocities from EKF [6 x N]
    sim_data.bias_ekf = [];
    if size(x_loc, 1) >= 15
        sim_data.bias_ekf = x_loc(13:15, :);  % Gyro bias estimates [3 x N]
    end
    
    % EKF Position covariance
    if ~isempty(P_loc)
        sim_data.P_loc = P_loc;         % Covariance history [15 x 15 x N]
    end
    if ~isempty(P_loc_init)
        sim_data.P_loc_init = P_loc_init;  % Initial covariance
    end
    if ~isempty(Q_loc)
        sim_data.Q_loc = Q_loc;         % Process noise (EKF position)
    end
    
    %% ========================================================================
    %% Ground Truth Data
    %% ========================================================================
    fprintf('Collecting ground truth data...\n');
    sim_data.eta_gt = eta_gt;           % Ground truth pose [6 x N]
    sim_data.nu_gt = nu_gt;             % Ground truth velocities [6 x N]
    sim_data.wRr_gt = wRr_gt;           % Ground truth rotation [3 x 3 x N]
    
    % Compute position estimation errors
    sim_data.pos_error = x_loc(1:3,:) - eta_gt(1:3,:);  % Position error [3 x N]
    sim_data.ang_error = x_loc(4:6,:) - eta_gt(4:6,:);  % Orientation error [3 x N]
    sim_data.vel_error = x_loc(7:9,:) - nu_gt(1:3,:);   % Velocity error [3 x N]
    sim_data.rate_error = x_loc(10:12,:) - nu_gt(4:6,:); % Angular rate error [3 x N]
    
    %% ========================================================================
    %% Initial Conditions and Parameters
    %% ========================================================================
    fprintf('Collecting initial conditions...\n');
    sim_data.x0 = x0;                   % Initial true state
    sim_data.x0_est = x0_est;           % Initial estimated state
    
    %% Terrain Generation Parameters
    fprintf('Collecting terrain parameters...\n');
    sim_data.max_planes = max_planes;
    sim_data.step_length = step_length;
    sim_data.angle_range = angle_range;
    sim_data.rate_of_change = rate_of_change;
    sim_data.delta_limit = delta_limit;
    sim_data.pp_init_w = pp_init_w;
    sim_data.n0 = n0;
    
    %% ========================================================================
    %% Computed Statistics (Pre-computed for efficiency)
    %% ========================================================================
    fprintf('Pre-computing statistics...\n');
    
    % Altitude tracking
    h_error = h_ref - x_est(1,:);
    sim_data.stats.altitude_error_mean = mean(h_error);
    sim_data.stats.altitude_error_std = std(h_error);
    sim_data.stats.altitude_error_rms = rms(h_error);
    sim_data.stats.altitude_error_max = max(abs(h_error));
    
    % Terrain angle estimation errors
    alpha_error = x_true(2,:) - x_est(2,:);
    beta_error = x_true(3,:) - x_est(3,:);
    sim_data.stats.alpha_error_rms = rms(alpha_error);
    sim_data.stats.beta_error_rms = rms(beta_error);
    
    % Position filter accuracy
    sim_data.stats.pos_rmse = sqrt(mean(sim_data.pos_error.^2, 2));
    sim_data.stats.ang_rmse = sqrt(mean(sim_data.ang_error.^2, 2));
    sim_data.stats.vel_rmse = sqrt(mean(sim_data.vel_error.^2, 2));
    
    % Sensor failure rate
    sim_data.stats.sensor_failure_rate = sum(sensor_fail > 0) / N * 100;
    sim_data.stats.total_sensor_failures = sum(sensor_fail);
    
    % Control effort
    sim_data.stats.control_effort_rms = rms(u, 2);
    sim_data.stats.control_effort_total = sqrt(sum(sim_data.stats.control_effort_rms.^2));
    
    % Innovation statistics
    innovation_norm = sqrt(sum(ni.^2, 1));
    sim_data.stats.innovation_mean = mean(innovation_norm);
    sim_data.stats.innovation_max = max(innovation_norm);
    sim_data.stats.innovation_std = std(innovation_norm);
    
    fprintf('Collection complete!\n');
    fprintf('Total data fields: %d\n\n', length(fieldnames(sim_data)));
end
