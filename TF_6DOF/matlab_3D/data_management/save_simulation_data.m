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
%       ekf_states.mat       - EKF states (true, estimated, predicted)
%       ekf_covariance.mat   - Covariance matrices and innovation
%       sensor_data.mat      - All sensor measurements
%       control_data.mat     - Controller outputs and errors
%       trajectory.mat       - Robot position and orientation
%       parameters.mat       - Simulation parameters
%       metadata.txt         - Human-readable run information
%       plots/               - Saved figures (optional)
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
    
    %% Save EKF States
    fprintf('Saving EKF states...\n');
    ekf_states.x_true = sim_data.x_true;        % True states [h, alpha, beta]
    ekf_states.x_est = sim_data.x_est;          % Estimated states
    ekf_states.x_pred = sim_data.x_pred;        % Predicted states
    ekf_states.h_ref = sim_data.h_ref;          % Reference altitude
    ekf_states.time = sim_data.time;            % Time vector
    save(fullfile(run_dir, 'ekf_states.mat'), '-struct', 'ekf_states');
    
    %% Save EKF Covariance and Innovation
    fprintf('Saving EKF covariance data...\n');
    ekf_cov.ni = sim_data.ni;                   % Innovation
    ekf_cov.S = sim_data.S;                     % Innovation covariance
    ekf_cov.P_final = sim_data.P_final;         % Final covariance matrix
    ekf_cov.P0 = sim_data.P0;                   % Initial covariance
    save(fullfile(run_dir, 'ekf_covariance.mat'), '-struct', 'ekf_cov');
    
    %% Save Sensor Data
    fprintf('Saving sensor measurements...\n');
    sensors.z_meas = sim_data.z_meas;           % SBES measurements
    sensors.z_pred = sim_data.z_pred;           % Predicted measurements
    sensors.n_mes = sim_data.n_mes;             % Measured normals
    sensors.n_est = sim_data.n_est;             % Estimated normals
    sensors.n_pre = sim_data.n_pre;             % Predicted normals
    sensors.rob_rot = sim_data.rob_rot;         % Robot angles (with noise)
    sensors.clean_rot = sim_data.clean_rot;     % Robot angles (clean)
    sensors.R = sim_data.R;                     % Measurement noise covariance
    save(fullfile(run_dir, 'sensor_data.mat'), '-struct', 'sensors');
    
    %% Save Control Data
    fprintf('Saving control data...\n');
    control.pid = sim_data.pid;                 % PID output
    control.u = sim_data.u;                     % Robot velocities
    control.u_dot = sim_data.u_dot;             % Robot accelerations
    control.goal = sim_data.goal;               % Desired setpoints
    control.integral_err = sim_data.integral_err;
    control.p_err = sim_data.p_err;
    control.i_err = sim_data.i_err;
    control.t_sum = sim_data.t_sum;
    save(fullfile(run_dir, 'control_data.mat'), '-struct', 'control');
    
    %% Save Trajectory Data
    fprintf('Saving trajectory data...\n');
    trajectory.prob = sim_data.prob;            % Robot position
    trajectory.wRr = sim_data.wRr;              % Robot rotation matrices
    trajectory.wRt = sim_data.wRt;              % Terrain rotation matrices
    trajectory.wRt_pre = sim_data.wRt_pre;      % Predicted terrain rotation
    trajectory.state = sim_data.state;          % State machine states
    save(fullfile(run_dir, 'trajectory.mat'), '-struct', 'trajectory');
    
    %% Save Simulation Parameters
    fprintf('Saving simulation parameters...\n');
    params.Ts = sim_data.Ts;                    % Sampling time
    params.Tf = sim_data.Tf;                    % Final time
    params.N = sim_data.N;                      % Number of iterations
    params.Q = sim_data.Q;                      % Process noise covariance
    params.R_tp = sim_data.R_tp;                % SBES noise covariance template
    params.R_a = sim_data.R_a;                  % AHRS noise covariance
    params.Kp = sim_data.Kp;                    % Proportional gains
    params.Ki = sim_data.Ki;                    % Integral gains
    params.Kd = sim_data.Kd;                    % Derivative gains
    params.Kt = sim_data.Kt;                    % Anti-windup gains
    params.speed0 = sim_data.speed0;            % Operating point velocity
    params.tau0 = sim_data.tau0;                % Equilibrium forces
    params.x0 = sim_data.x0;                    % Initial true state
    params.x0_est = sim_data.x0_est;            % Initial estimated state
    params.max_planes = sim_data.max_planes;    % Terrain buffer size
    params.step_length = sim_data.step_length;  % Distance between planes
    params.angle_range = sim_data.angle_range;  % Terrain angle limits
    params.rate_of_change = sim_data.rate_of_change; % Terrain variation rate
    params.pp_init_w = sim_data.pp_init_w;      % Initial plane position
    params.n0 = sim_data.n0;                    % Reference normal vector
    save(fullfile(run_dir, 'parameters.mat'), '-struct', 'params');
    
    %% Save Metadata (Human-Readable)
    fprintf('Saving metadata...\n');
    fid = fopen(fullfile(run_dir, 'metadata.txt'), 'w');
    fprintf(fid, '=== SIMULATION RUN METADATA ===\n');
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
    fprintf(fid, 'Operating point velocity: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', sim_data.speed0);
    fprintf(fid, '\n--- EKF PARAMETERS ---\n');
    fprintf(fid, 'Process noise Q:\n');
    fprintf(fid, '  %.6f  %.6f  %.6f\n', sim_data.Q');
    fprintf(fid, 'Initial covariance P0:\n');
    fprintf(fid, '  %.6f  %.6f  %.6f\n', sim_data.P0');
    fprintf(fid, '\n--- CONTROLLER GAINS ---\n');
    fprintf(fid, 'Kp: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', sim_data.Kp);
    fprintf(fid, 'Ki: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', sim_data.Ki);
    fprintf(fid, 'Kd: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', sim_data.Kd);
    fprintf(fid, '\n--- TERRAIN PARAMETERS ---\n');
    fprintf(fid, 'Max planes (buffer): %d\n', sim_data.max_planes);
    fprintf(fid, 'Step length: %.2f m\n', sim_data.step_length);
    fprintf(fid, 'Angle range: [%.2f, %.2f] deg ([%.4f, %.4f] rad)\n', ...
        rad2deg(sim_data.angle_range(1)), rad2deg(sim_data.angle_range(2)), ...
        sim_data.angle_range(1), sim_data.angle_range(2));
    fprintf(fid, 'Rate of change: %.2f\n', sim_data.rate_of_change);
    fprintf(fid, 'Initial plane position: [%.2f, %.2f, %.2f]\n', sim_data.pp_init_w);
    fprintf(fid, 'Reference normal: [%.2f, %.2f, %.2f]\n', sim_data.n0);
    fprintf(fid, '\n--- FINAL RESULTS ---\n');
    fprintf(fid, 'Final altitude error: %.4f m\n', sim_data.h_ref(end) - sim_data.x_est(1,end));
    fprintf(fid, 'Mean altitude error: %.4f m\n', mean(sim_data.h_ref - sim_data.x_est(1,:)));
    fprintf(fid, 'RMS altitude error: %.4f m\n', rms(sim_data.h_ref - sim_data.x_est(1,:)));
    fclose(fid);
    
    fprintf('\n=== SAVE COMPLETE ===\n');
    fprintf('Data saved to: %s\n', run_dir);
    fprintf('Total files: 7 (6 .mat + 1 metadata.txt)\n\n');
end
