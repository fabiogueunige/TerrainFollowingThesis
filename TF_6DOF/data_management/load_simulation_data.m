%% LOAD_SIMULATION_DATA - Load saved simulation data for analysis
%
% Loads all simulation data from a saved run directory for post-processing
% and statistical analysis. Supports backward compatibility with older data formats.
%
% SYNTAX:
%   sim_data = load_simulation_data(run_name)
%   [sim_data, metadata] = load_simulation_data(run_name)
%
% INPUTS:
%   run_name - Name of the run directory (e.g., 'run_20251023_143022')
%              If empty, shows list of available runs
%
% OUTPUTS:
%   sim_data - Structure containing all simulation data
%   metadata - String containing metadata text
%
% BACKWARD COMPATIBILITY:
%   This function automatically detects the data version and handles:
%   - Version 1.x: Old format with limited fields
%   - Version 2.x: New format with EKF position, ground truth, state machine
%   Missing fields are initialized to empty arrays or zeros where appropriate.
%
% EXAMPLE:
%   % Load specific run
%   sim_data = load_simulation_data('run_20251023_143022');
%   
%   % Show available runs and select
%   sim_data = load_simulation_data('');
%
% See also: save_simulation_data, collect_simulation_data

function [sim_data, metadata] = load_simulation_data(run_name)
    %% Check Results Directory
    base_dir = 'results';
    
    if ~exist(base_dir, 'dir')
        error('No results directory found. Run simulations first.');
    end
    
    %% List Available Runs
    if nargin < 1 || isempty(run_name)
        % Show available runs
        runs = dir(fullfile(base_dir, 'run_*'));
        
        if isempty(runs)
            error('No simulation runs found in %s', base_dir);
        end
        
        fprintf('\n=== AVAILABLE SIMULATION RUNS ===\n');
        for i = 1:length(runs)
            fprintf('%d. %s\n', i, runs(i).name);
        end
        
        % Prompt user to select
        choice = input(sprintf('\nSelect run (1-%d): ', length(runs)));
        
        if choice < 1 || choice > length(runs)
            error('Invalid selection');
        end
        
        run_name = runs(choice).name;
    end
    
    run_dir = fullfile(base_dir, run_name);
    
    if ~exist(run_dir, 'dir')
        error('Run directory not found: %s', run_dir);
    end
    
    fprintf('\n=== LOADING SIMULATION DATA ===\n');
    fprintf('Run: %s\n', run_name);
    fprintf('Directory: %s\n\n', run_dir);
    
    %% Initialize Empty Structure
    sim_data = struct();
    
    %% Detect Data Version
    % Check for new file format
    has_new_format = exist(fullfile(run_dir, 'ekf_sbes_states.mat'), 'file') || ...
                     exist(fullfile(run_dir, 'ekf_position_states.mat'), 'file');
    
    if has_new_format
        fprintf('Detected data version 2.x (new format)\n\n');
        sim_data.data_version = 2.0;
    else
        fprintf('Detected data version 1.x (legacy format)\n\n');
        sim_data.data_version = 1.0;
    end
    
    %% Load EKF SBES States
    fprintf('Loading EKF SBES states...\n');
    if exist(fullfile(run_dir, 'ekf_sbes_states.mat'), 'file')
        ekf_sbes_states = load(fullfile(run_dir, 'ekf_sbes_states.mat'));
        sim_data = merge_struct(sim_data, ekf_sbes_states);
    elseif exist(fullfile(run_dir, 'ekf_states.mat'), 'file')
        % Backward compatibility
        ekf_states = load(fullfile(run_dir, 'ekf_states.mat'));
        sim_data = merge_struct(sim_data, ekf_states);
    end
    
    %% Load EKF SBES Covariance
    fprintf('Loading EKF SBES covariance...\n');
    if exist(fullfile(run_dir, 'ekf_sbes_covariance.mat'), 'file')
        ekf_sbes_cov = load(fullfile(run_dir, 'ekf_sbes_covariance.mat'));
        sim_data = merge_struct(sim_data, ekf_sbes_cov);
    elseif exist(fullfile(run_dir, 'ekf_covariance.mat'), 'file')
        % Backward compatibility
        ekf_cov = load(fullfile(run_dir, 'ekf_covariance.mat'));
        sim_data = merge_struct(sim_data, ekf_cov);
    end
    
    %% Load EKF Position Filter States
    fprintf('Loading EKF Position filter states...\n');
    if exist(fullfile(run_dir, 'ekf_position_states.mat'), 'file')
        ekf_pos_states = load(fullfile(run_dir, 'ekf_position_states.mat'));
        sim_data = merge_struct(sim_data, ekf_pos_states);
    end
    
    %% Load EKF Position Filter Covariance
    fprintf('Loading EKF Position filter covariance...\n');
    if exist(fullfile(run_dir, 'ekf_position_covariance.mat'), 'file')
        ekf_pos_cov = load(fullfile(run_dir, 'ekf_position_covariance.mat'));
        sim_data = merge_struct(sim_data, ekf_pos_cov);
    end
    
    %% Load Ground Truth Data
    fprintf('Loading ground truth data...\n');
    if exist(fullfile(run_dir, 'ground_truth.mat'), 'file')
        ground_truth = load(fullfile(run_dir, 'ground_truth.mat'));
        sim_data = merge_struct(sim_data, ground_truth);
    end
    
    %% Load Sensor Data
    fprintf('Loading sensor data...\n');
    if exist(fullfile(run_dir, 'sensor_data.mat'), 'file')
        sensors = load(fullfile(run_dir, 'sensor_data.mat'));
        sim_data = merge_struct(sim_data, sensors);
    end
    
    %% Load Control Data
    fprintf('Loading control data...\n');
    if exist(fullfile(run_dir, 'control_data.mat'), 'file')
        control = load(fullfile(run_dir, 'control_data.mat'));
        sim_data = merge_struct(sim_data, control);
    end
    
    %% Load Trajectory Data
    fprintf('Loading trajectory...\n');
    if exist(fullfile(run_dir, 'trajectory.mat'), 'file')
        trajectory = load(fullfile(run_dir, 'trajectory.mat'));
        sim_data = merge_struct(sim_data, trajectory);
    end
    
    %% Load State Machine Data
    fprintf('Loading state machine data...\n');
    if exist(fullfile(run_dir, 'state_machine.mat'), 'file')
        state_machine_data = load(fullfile(run_dir, 'state_machine.mat'));
        sim_data = merge_struct(sim_data, state_machine_data);
    elseif isfield(sim_data, 'state')
        % State was loaded from trajectory file (old format)
        fprintf('  (State loaded from trajectory file - legacy format)\n');
    end
    
    %% Load Parameters
    fprintf('Loading parameters...\n');
    if exist(fullfile(run_dir, 'parameters.mat'), 'file')
        params = load(fullfile(run_dir, 'parameters.mat'));
        sim_data = merge_struct(sim_data, params);
    end
    
    %% Load Pre-computed Statistics
    fprintf('Loading statistics...\n');
    if exist(fullfile(run_dir, 'statistics.mat'), 'file')
        statistics = load(fullfile(run_dir, 'statistics.mat'));
        sim_data.stats = statistics;
    end
    
    %% Apply Backward Compatibility Fixes
    fprintf('Applying backward compatibility fixes...\n');
    sim_data = apply_backward_compatibility(sim_data);
    
    %% Create Time Alias
    if isfield(sim_data, 'time') && ~isfield(sim_data, 't')
        sim_data.t = sim_data.time;
    elseif isfield(sim_data, 't') && ~isfield(sim_data, 'time')
        sim_data.time = sim_data.t;
    end
    
    %% Load Metadata
    metadata_file = fullfile(run_dir, 'metadata.txt');
    if exist(metadata_file, 'file')
        fprintf('Loading metadata...\n');
        fid = fopen(metadata_file, 'r');
        metadata = fread(fid, '*char')';
        fclose(fid);
    else
        metadata = '';
    end
    
    fprintf('\n=== LOAD COMPLETE ===\n');
    fprintf('Data version: %.1f\n', sim_data.data_version);
    fprintf('Total fields loaded: %d\n', length(fieldnames(sim_data)));
    
    if nargout < 2
        fprintf('\n--- METADATA ---\n');
        fprintf('%s\n', metadata);
    end
end

%% Helper Function: Merge Structures
function s1 = merge_struct(s1, s2)
    % Merge fields from s2 into s1, without overwriting existing fields
    fields = fieldnames(s2);
    for i = 1:length(fields)
        if ~isfield(s1, fields{i})
            s1.(fields{i}) = s2.(fields{i});
        end
    end
end

%% Helper Function: Apply Backward Compatibility
function sim_data = apply_backward_compatibility(sim_data)
    % Ensure all required fields exist for analysis functions
    
    % Get N from available time data
    N = 0;
    if isfield(sim_data, 'time')
        N = length(sim_data.time);
    elseif isfield(sim_data, 't')
        N = length(sim_data.t);
    elseif isfield(sim_data, 'x_est')
        N = size(sim_data.x_est, 2);
    end
    
    % Get Ts from time vector or default
    if ~isfield(sim_data, 'Ts')
        if isfield(sim_data, 'time') && length(sim_data.time) > 1
            sim_data.Ts = sim_data.time(2) - sim_data.time(1);
        else
            sim_data.Ts = 0.001;  % Default
        end
    end
    
    % Get Tf from time vector or default
    if ~isfield(sim_data, 'Tf')
        if isfield(sim_data, 'time')
            sim_data.Tf = sim_data.time(end);
        else
            sim_data.Tf = N * sim_data.Ts;
        end
    end
    
    % Ensure N is set
    if ~isfield(sim_data, 'N')
        sim_data.N = N;
    end
    
    % EKF Position filter states (initialize if missing)
    if ~isfield(sim_data, 'x_loc')
        sim_data.x_loc = zeros(15, N);
        fprintf('  Warning: x_loc not found, initialized to zeros\n');
    end
    
    if ~isfield(sim_data, 'eta_ekf')
        if size(sim_data.x_loc, 1) >= 6
            sim_data.eta_ekf = sim_data.x_loc(1:6, :);
        else
            sim_data.eta_ekf = zeros(6, N);
        end
    end
    
    if ~isfield(sim_data, 'nu_ekf')
        if size(sim_data.x_loc, 1) >= 12
            sim_data.nu_ekf = sim_data.x_loc(7:12, :);
        else
            sim_data.nu_ekf = zeros(6, N);
        end
    end
    
    % Ground truth (initialize if missing)
    if ~isfield(sim_data, 'eta_gt')
        sim_data.eta_gt = zeros(6, N);
        fprintf('  Warning: eta_gt not found, initialized to zeros\n');
    end
    
    if ~isfield(sim_data, 'nu_gt')
        sim_data.nu_gt = zeros(6, N);
        fprintf('  Warning: nu_gt not found, initialized to zeros\n');
    end
    
    if ~isfield(sim_data, 'wRr_gt')
        sim_data.wRr_gt = zeros(3, 3, N);
        for k = 1:N
            sim_data.wRr_gt(:,:,k) = eye(3);
        end
        fprintf('  Warning: wRr_gt not found, initialized to identity\n');
    end
    
    % Sensor failure (compute from measurements if missing)
    if ~isfield(sim_data, 'sensor_fail')
        if isfield(sim_data, 'z_meas')
            sensor_fail = zeros(1, size(sim_data.z_meas, 2));
            for k = 1:size(sim_data.z_meas, 2)
                sensor_fail(k) = sum(isnan(sim_data.z_meas(:,k)) | isinf(sim_data.z_meas(:,k)));
            end
            sim_data.sensor_fail = sensor_fail;
        else
            sim_data.sensor_fail = zeros(1, N);
        end
    end
    
    % State machine data
    if ~isfield(sim_data, 'state')
        sim_data.state = strings(1, N);
        sim_data.state(:) = "Unknown";
        fprintf('  Warning: state not found, initialized to Unknown\n');
    end
    
    if ~isfield(sim_data, 'state_numeric')
        % Compute state_numeric from state strings
        state_names = {'Idle', 'Reset', 'TargetAltitude', 'ContactSearch', ...
                       'MovePitch', 'MoveRoll', 'RecoveryAltitude', 'Following', ...
                       'Emergency', 'EndSimulation'};
        sim_data.state_names = state_names;
        sim_data.state_numeric = zeros(1, N);
        for k = 1:N
            idx = find(strcmp(state_names, sim_data.state(k)));
            if ~isempty(idx)
                sim_data.state_numeric(k) = idx;
            else
                sim_data.state_numeric(k) = 0;
            end
        end
    end
    
    if ~isfield(sim_data, 'state_names')
        sim_data.state_names = {'Idle', 'Reset', 'TargetAltitude', 'ContactSearch', ...
                                'MovePitch', 'MoveRoll', 'RecoveryAltitude', 'Following', ...
                                'Emergency', 'EndSimulation'};
    end
    
    % Compute state transitions if missing
    if ~isfield(sim_data, 'state_transitions')
        sim_data.state_transitions = find(diff(sim_data.state_numeric) ~= 0);
        sim_data.num_transitions = length(sim_data.state_transitions);
    end
    
    % State time and percentage
    if ~isfield(sim_data, 'state_time')
        state_time = zeros(1, length(sim_data.state_names));
        for s = 1:length(sim_data.state_names)
            state_time(s) = sum(sim_data.state_numeric == s) * sim_data.Ts;
        end
        sim_data.state_time = state_time;
        sim_data.state_percentage = state_time / sim_data.Tf * 100;
    end
    
    % Error computations
    if ~isfield(sim_data, 'pos_error') && isfield(sim_data, 'x_loc') && isfield(sim_data, 'eta_gt')
        sim_data.pos_error = sim_data.x_loc(1:3,:) - sim_data.eta_gt(1:3,:);
    end
    
    if ~isfield(sim_data, 'ang_error') && isfield(sim_data, 'x_loc') && isfield(sim_data, 'eta_gt')
        sim_data.ang_error = sim_data.x_loc(4:6,:) - sim_data.eta_gt(4:6,:);
    end
    
    % Control gains (initialize if missing)
    if ~isfield(sim_data, 'Kp')
        sim_data.Kp = zeros(1, 6);
    end
    if ~isfield(sim_data, 'Ki')
        sim_data.Ki = zeros(1, 6);
    end
    if ~isfield(sim_data, 'Kd')
        sim_data.Kd = zeros(1, 6);
    end
    
    % Goal arrays
    if ~isfield(sim_data, 'goal_roll') && isfield(sim_data, 'goal')
        try
            sim_data.goal_surge = [sim_data.goal.surge];
            sim_data.goal_sway = [sim_data.goal.sway];
            sim_data.goal_altitude = [sim_data.goal.altitude];
            sim_data.goal_roll = [sim_data.goal.roll];
            sim_data.goal_pitch = [sim_data.goal.pitch];
            sim_data.goal_yaw = [sim_data.goal.yaw];
        catch
            % Goal structure may have different format
            sim_data.goal_roll = zeros(1, N);
            sim_data.goal_pitch = zeros(1, N);
        end
    end
    
    % SBES process noise Q alias
    if ~isfield(sim_data, 'Q_sbes') && isfield(sim_data, 'Q')
        sim_data.Q_sbes = sim_data.Q;
    end
    
    fprintf('  Backward compatibility fixes applied.\n');
end
