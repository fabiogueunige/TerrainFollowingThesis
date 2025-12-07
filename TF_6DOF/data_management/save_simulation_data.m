%% SAVE_SIMULATION_DATA - Save complete simulation data for statistical analysis
%
% Saves all important simulation data to a structured folder for future
% statistical analysis. Each run is saved in a separate timestamped folder.
%
% SYNTAX:
%   save_simulation_data(sim_data, run_name)
%
% INPUTS:
%   sim_data - Structure containing all simulation variables
%   run_name - Optional custom name for the run (default: auto-generated)
%
% SAVED DATA STRUCTURE:
%   results/
%     run_YYYYMMDD_HHMMSS/
%       ekf_sbes_states.mat      - EKF SBES states (terrain following)
%       ekf_sbes_covariance.mat  - SBES covariance and innovation
%       ekf_position_states.mat  - EKF Position filter states
%       ekf_position_covariance.mat - Position filter covariance
%       ground_truth.mat         - Ground truth data for comparison
%       sensor_data.mat          - All sensor measurements
%       control_data.mat         - Controller outputs and errors
%       trajectory.mat           - Robot position and orientation
%       state_machine.mat        - State machine history and transitions
%       parameters.mat           - Simulation parameters
%       statistics.mat           - Pre-computed statistics
%       metadata.txt             - Human-readable run information
%
% EXAMPLE:
%   % After simulation
%   sim_data = collect_simulation_data(time, x_true, x_est, ...);
%   save_simulation_data(sim_data);
%
% See also: collect_simulation_data, load_simulation_data, analyze_statistics

function save_simulation_data(sim_data, run_name)
    %% Input Validation
    if nargin < 2 || isempty(run_name)
        % Auto-generate run name with timestamp
        run_name = sprintf('run_%s', datestr(now, 'yyyymmdd_HHMMSS'));
    end
    
    %% Create Results Directory Structure
    base_dir = 'results';
    run_dir = fullfile(base_dir, run_name);
    
    % Create directories
    if ~exist(base_dir, 'dir')
        mkdir(base_dir);
    end
    
    if ~exist(run_dir, 'dir')
        mkdir(run_dir);
    else
        warning('Directory %s already exists. Data may be overwritten.', run_dir);
    end
    
    fprintf('\n=== SAVING SIMULATION DATA ===\n');
    fprintf('Run name: %s\n', run_name);
    fprintf('Directory: %s\n', run_dir);
    
    %% Data Version
    data_version = 2.0;
    
    %% Save EKF SBES States (Terrain Following Filter)
    fprintf('Saving EKF SBES states...\n');
    ekf_sbes_states.x_true = sim_data.x_true;        % True terrain state [h, alpha, beta]
    ekf_sbes_states.x_est = sim_data.x_est;          % Estimated terrain state
    ekf_sbes_states.x_pred = sim_data.x_pred;        % Predicted terrain state
    ekf_sbes_states.h_ref = sim_data.h_ref;          % Reference altitude
    ekf_sbes_states.time = sim_data.time;            % Time vector
    ekf_sbes_states.data_version = data_version;
    save(fullfile(run_dir, 'ekf_sbes_states.mat'), '-struct', 'ekf_sbes_states');
    % Backward compatibility: also save as old filename
    save(fullfile(run_dir, 'ekf_states.mat'), '-struct', 'ekf_sbes_states');
    
    %% Save EKF SBES Covariance and Innovation
    fprintf('Saving EKF SBES covariance data...\n');
    ekf_sbes_cov.ni = sim_data.ni;                   % Innovation [4 x N]
    ekf_sbes_cov.S = sim_data.S;                     % Innovation covariance [4 x 4 x N]
    ekf_sbes_cov.P_final = sim_data.P_final;         % Final covariance matrix
    ekf_sbes_cov.P0 = sim_data.P0;                   % Initial covariance
    if isfield(sim_data, 'P_sbes')
        ekf_sbes_cov.P_sbes = sim_data.P_sbes;       % Full covariance history [3 x 3 x N] for NEES
        fprintf('  - P_sbes saved [3x3x%d] for NEES analysis\n', size(sim_data.P_sbes, 3));
    end
    if isfield(sim_data, 'Q_sbes')
        ekf_sbes_cov.Q = sim_data.Q_sbes;            % Process noise
    elseif isfield(sim_data, 'Q')
        ekf_sbes_cov.Q = sim_data.Q;
    end
    ekf_sbes_cov.data_version = data_version;
    save(fullfile(run_dir, 'ekf_sbes_covariance.mat'), '-struct', 'ekf_sbes_cov');
    % Backward compatibility
    save(fullfile(run_dir, 'ekf_covariance.mat'), '-struct', 'ekf_sbes_cov');
    
    %% Save EKF Position Filter States
    fprintf('Saving EKF Position filter states...\n');
    ekf_pos_states.x_loc = sim_data.x_loc;           % Full state [15 x N]
    if isfield(sim_data, 'eta_ekf')
        ekf_pos_states.eta_ekf = sim_data.eta_ekf;   % Position & orientation [6 x N]
    else
        ekf_pos_states.eta_ekf = sim_data.x_loc(1:6, :);
    end
    if isfield(sim_data, 'nu_ekf')
        ekf_pos_states.nu_ekf = sim_data.nu_ekf;     % Velocities [6 x N]
    else
        ekf_pos_states.nu_ekf = sim_data.x_loc(7:12, :);
    end
    if isfield(sim_data, 'bias_ekf') && ~isempty(sim_data.bias_ekf)
        ekf_pos_states.bias_ekf = sim_data.bias_ekf; % Gyro bias [3 x N]
    elseif size(sim_data.x_loc, 1) >= 15
        ekf_pos_states.bias_ekf = sim_data.x_loc(13:15, :);
    end
    ekf_pos_states.time = sim_data.time;
    ekf_pos_states.data_version = data_version;
    save(fullfile(run_dir, 'ekf_position_states.mat'), '-struct', 'ekf_pos_states');
    
    %% Save EKF Position Filter Covariance
    fprintf('Saving EKF Position filter covariance...\n');
    ekf_pos_cov = struct();
    ekf_pos_cov.data_version = data_version;
    if isfield(sim_data, 'P_loc')
        ekf_pos_cov.P_loc = sim_data.P_loc;          % Covariance history [15 x 15 x N]
    end
    if isfield(sim_data, 'P_loc_init')
        ekf_pos_cov.P_loc_init = sim_data.P_loc_init;
    end
    if isfield(sim_data, 'Q_loc')
        ekf_pos_cov.Q_loc = sim_data.Q_loc;
    end
    save(fullfile(run_dir, 'ekf_position_covariance.mat'), '-struct', 'ekf_pos_cov');
    
    %% Save Ground Truth Data
    fprintf('Saving ground truth data...\n');
    ground_truth.eta_gt = sim_data.eta_gt;           % Ground truth pose [6 x N]
    ground_truth.nu_gt = sim_data.nu_gt;             % Ground truth velocities [6 x N]
    ground_truth.wRr_gt = sim_data.wRr_gt;           % Ground truth rotation [3 x 3 x N]
    if isfield(sim_data, 'pos_error')
        ground_truth.pos_error = sim_data.pos_error; % Position error [3 x N]
    end
    if isfield(sim_data, 'ang_error')
        ground_truth.ang_error = sim_data.ang_error; % Orientation error [3 x N]
    end
    if isfield(sim_data, 'vel_error')
        ground_truth.vel_error = sim_data.vel_error; % Velocity error [3 x N]
    end
    if isfield(sim_data, 'rate_error')
        ground_truth.rate_error = sim_data.rate_error; % Angular rate error [3 x N]
    end
    ground_truth.time = sim_data.time;
    ground_truth.data_version = data_version;
    save(fullfile(run_dir, 'ground_truth.mat'), '-struct', 'ground_truth');
    
    %% Save Sensor Data
    fprintf('Saving sensor measurements...\n');
    sensors.z_meas = sim_data.z_meas;                % SBES measurements [4 x N]
    sensors.z_pred = sim_data.z_pred;                % Predicted measurements [4 x N]
    sensors.n_mes = sim_data.n_mes;                  % Measured normals [3 x N]
    sensors.n_est = sim_data.n_est;                  % Estimated normals [3 x N]
    sensors.n_pre = sim_data.n_pre;                  % Predicted normals [3 x N]
    sensors.rob_rot = sim_data.rob_rot;              % Robot angles (with noise) [3 x N]
    sensors.clean_rot = sim_data.clean_rot;          % Robot angles (clean) [3 x N]
    sensors.R = sim_data.R;                          % Measurement noise covariance [4 x 4 x N]
    if isfield(sim_data, 'R_SBES')
        sensors.R_SBES = sim_data.R_SBES;            % SBES noise template
    end
    if isfield(sim_data, 'sensor_fail')
        sensors.sensor_fail = sim_data.sensor_fail;  % Sensor failure count [1 x N]
    end
    sensors.time = sim_data.time;
    sensors.data_version = data_version;
    save(fullfile(run_dir, 'sensor_data.mat'), '-struct', 'sensors');
    
    %% Save Control Data
    fprintf('Saving control data...\n');
    control.pid = sim_data.pid;                      % PID output [6 x N]
    control.u = sim_data.u;                          % Body velocities [6 x N]
    control.u_dot = sim_data.u_dot;                  % Body accelerations [6 x N]
    control.goal = sim_data.goal;                    % Goal structure array
    control.integral_err = sim_data.integral_err;    % Integral error [6 x N]
    control.p_err = sim_data.p_err;                  % Proportional error [6 x N]
    control.i_err = sim_data.i_err;                  % Integral error term [6 x N]
    control.t_sum = sim_data.t_sum;                  % Anti-windup term [6 x N]
    control.Kp = sim_data.Kp;                        % Proportional gains
    control.Ki = sim_data.Ki;                        % Integral gains
    if isfield(sim_data, 'Kd')
        control.Kd = sim_data.Kd;                    % Derivative gains
    end
    if isfield(sim_data, 'speed0')
        control.speed0 = sim_data.speed0;            % Operating point velocity
    end
    % Extract goal arrays for easier analysis
    if isfield(sim_data, 'goal_surge')
        control.goal_surge = sim_data.goal_surge;
        control.goal_sway = sim_data.goal_sway;
        control.goal_altitude = sim_data.goal_altitude;
        control.goal_roll = sim_data.goal_roll;
        control.goal_pitch = sim_data.goal_pitch;
        control.goal_yaw = sim_data.goal_yaw;
    end
    control.time = sim_data.time;
    control.data_version = data_version;
    save(fullfile(run_dir, 'control_data.mat'), '-struct', 'control');
    
    %% Save Trajectory Data
    fprintf('Saving trajectory data...\n');
    trajectory.prob = sim_data.prob;                 % Robot position [3 x N]
    trajectory.wRr = sim_data.wRr;                   % Robot rotation matrices [3 x 3 x N]
    trajectory.wRt = sim_data.wRt;                   % Terrain rotation matrices [3 x 3 x N]
    trajectory.wRt_pre = sim_data.wRt_pre;           % Predicted terrain rotation [3 x 3 x N]
    trajectory.time = sim_data.time;
    trajectory.data_version = data_version;
    save(fullfile(run_dir, 'trajectory.mat'), '-struct', 'trajectory');
    
    %% Save State Machine Data
    fprintf('Saving state machine data...\n');
    state_machine_data.state = sim_data.state;       % State string array [1 x N]
    if isfield(sim_data, 'state_numeric')
        state_machine_data.state_numeric = sim_data.state_numeric;
    end
    if isfield(sim_data, 'state_names')
        state_machine_data.state_names = sim_data.state_names;
    end
    if isfield(sim_data, 'state_transitions')
        state_machine_data.state_transitions = sim_data.state_transitions;
    end
    if isfield(sim_data, 'num_transitions')
        state_machine_data.num_transitions = sim_data.num_transitions;
    end
    if isfield(sim_data, 'state_time')
        state_machine_data.state_time = sim_data.state_time;
    end
    if isfield(sim_data, 'state_percentage')
        state_machine_data.state_percentage = sim_data.state_percentage;
    end
    if isfield(sim_data, 'cmd_history')
        state_machine_data.cmd_history = sim_data.cmd_history;
    end
    state_machine_data.time = sim_data.time;
    state_machine_data.data_version = data_version;
    save(fullfile(run_dir, 'state_machine.mat'), '-struct', 'state_machine_data');
    
    %% Save Simulation Parameters
    fprintf('Saving simulation parameters...\n');
    params.Ts = sim_data.Ts;                         % Sampling time
    params.Tf = sim_data.Tf;                         % Final time
    params.N = sim_data.N;                           % Number of iterations
    if isfield(sim_data, 'Q_sbes')
        params.Q_sbes = sim_data.Q_sbes;             % SBES process noise
    elseif isfield(sim_data, 'Q')
        params.Q_sbes = sim_data.Q;
    end
    if isfield(sim_data, 'R_SBES')
        params.R_SBES = sim_data.R_SBES;             % SBES measurement noise template
    end
    if isfield(sim_data, 'Q_loc')
        params.Q_loc = sim_data.Q_loc;               % Position filter process noise
    end
    params.Kp = sim_data.Kp;                         % Proportional gains
    params.Ki = sim_data.Ki;                         % Integral gains
    if isfield(sim_data, 'Kd')
        params.Kd = sim_data.Kd;                     % Derivative gains
    end
    if isfield(sim_data, 'speed0')
        params.speed0 = sim_data.speed0;             % Operating point velocity
    end
    params.x0 = sim_data.x0;                         % Initial true state
    params.x0_est = sim_data.x0_est;                 % Initial estimated state
    params.max_planes = sim_data.max_planes;         % Terrain buffer size
    params.step_length = sim_data.step_length;       % Distance between planes
    params.angle_range = sim_data.angle_range;       % Terrain angle limits
    params.rate_of_change = sim_data.rate_of_change; % Terrain variation rate
    params.delta_limit = sim_data.delta_limit;       % Max angle change between planes
    params.pp_init_w = sim_data.pp_init_w;           % Initial plane position
    params.n0 = sim_data.n0;                         % Reference normal vector
    params.data_version = data_version;
    save(fullfile(run_dir, 'parameters.mat'), '-struct', 'params');
    
    %% Save Pre-computed Statistics
    fprintf('Saving pre-computed statistics...\n');
    if isfield(sim_data, 'stats')
        statistics = sim_data.stats;
        statistics.data_version = data_version;
        save(fullfile(run_dir, 'statistics.mat'), '-struct', 'statistics');
    end
    
    %% Save Metadata (Human-Readable)
    fprintf('Saving metadata...\n');
    fid = fopen(fullfile(run_dir, 'metadata.txt'), 'w');
    fprintf(fid, '=== SIMULATION RUN METADATA ===\n');
    fprintf(fid, 'Data Version: %.1f\n', data_version);
    fprintf(fid, 'Run name: %s\n', run_name);
    fprintf(fid, 'Date: %s\n', datestr(now));
    fprintf(fid, '\n--- SIMULATION PARAMETERS ---\n');
    fprintf(fid, 'Sampling time: %.4f s\n', sim_data.Ts);
    fprintf(fid, 'Final time: %.1f s\n', sim_data.Tf);
    fprintf(fid, 'Iterations: %d\n', sim_data.N);
    fprintf(fid, 'Reference altitude: %.2f m\n', sim_data.h_ref(1));
    fprintf(fid, '\n--- INITIAL CONDITIONS ---\n');
    fprintf(fid, 'Initial true state: [%.2f, %.4f, %.4f]\n', sim_data.x0);
    fprintf(fid, 'Initial estimated state: [%.2f, %.4f, %.4f]\n', sim_data.x0_est);
    if isfield(sim_data, 'speed0')
        fprintf(fid, 'Operating point velocity: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', sim_data.speed0);
    end
    fprintf(fid, '\n--- EKF SBES PARAMETERS ---\n');
    if isfield(sim_data, 'Q_sbes')
        fprintf(fid, 'Process noise Q (SBES):\n');
        fprintf(fid, '  %.6f  %.6f  %.6f\n', sim_data.Q_sbes');
    elseif isfield(sim_data, 'Q')
        fprintf(fid, 'Process noise Q:\n');
        fprintf(fid, '  %.6f  %.6f  %.6f\n', sim_data.Q');
    end
    fprintf(fid, 'Initial covariance P0:\n');
    fprintf(fid, '  %.6f  %.6f  %.6f\n', sim_data.P0');
    fprintf(fid, '\n--- CONTROLLER GAINS ---\n');
    fprintf(fid, 'Kp: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', sim_data.Kp);
    fprintf(fid, 'Ki: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', sim_data.Ki);
    if isfield(sim_data, 'Kd')
        fprintf(fid, 'Kd: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', sim_data.Kd);
    end
    fprintf(fid, '\n--- TERRAIN PARAMETERS ---\n');
    fprintf(fid, 'Max planes (buffer): %d\n', sim_data.max_planes);
    fprintf(fid, 'Step length: %.2f m\n', sim_data.step_length);
    fprintf(fid, 'Angle range: [%.2f, %.2f] deg ([%.4f, %.4f] rad)\n', ...
        rad2deg(sim_data.angle_range(1)), rad2deg(sim_data.angle_range(2)), ...
        sim_data.angle_range(1), sim_data.angle_range(2));
    fprintf(fid, 'Rate of change: %.2f\n', sim_data.rate_of_change);
    fprintf(fid, 'Delta limit: %.2f deg (%.4f rad)\n', rad2deg(sim_data.delta_limit), sim_data.delta_limit);
    fprintf(fid, 'Initial plane position: [%.2f, %.2f, %.2f]\n', sim_data.pp_init_w);
    fprintf(fid, 'Reference normal: [%.2f, %.2f, %.2f]\n', sim_data.n0);
    fprintf(fid, '\n--- FINAL RESULTS ---\n');
    fprintf(fid, 'Final altitude error: %.4f m\n', sim_data.h_ref(end) - sim_data.x_est(1,end));
    fprintf(fid, 'Mean altitude error: %.4f m\n', mean(sim_data.h_ref - sim_data.x_est(1,:)));
    fprintf(fid, 'RMS altitude error: %.4f m\n', rms(sim_data.h_ref - sim_data.x_est(1,:)));
    if isfield(sim_data, 'stats')
        fprintf(fid, '\n--- PRE-COMPUTED STATISTICS ---\n');
        fprintf(fid, 'Sensor failure rate: %.2f%%\n', sim_data.stats.sensor_failure_rate);
        if isfield(sim_data.stats, 'pos_rmse')
            fprintf(fid, 'Position RMSE: [%.4f, %.4f, %.4f] m\n', sim_data.stats.pos_rmse);
        end
    end
    if isfield(sim_data, 'num_transitions')
        fprintf(fid, '\n--- STATE MACHINE ---\n');
        fprintf(fid, 'Total state transitions: %d\n', sim_data.num_transitions);
        fprintf(fid, 'Transitions per minute: %.2f\n', sim_data.num_transitions / (sim_data.Tf / 60));
    end
    fclose(fid);
    
    fprintf('\n=== SAVE COMPLETE ===\n');
    fprintf('Data saved to: %s\n', run_dir);
    fprintf('Total files: 13 (11 .mat + 1 metadata.txt + backward compat)\n\n');
end
