%% LOAD_SIMULATION_DATA - Load saved simulation data for analysis
%
% Loads all simulation data from a saved run directory for post-processing
% and statistical analysis.
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
    
    %% Load All Data Files
    fprintf('Loading EKF states...\n');
    ekf_states = load(fullfile(run_dir, 'ekf_states.mat'));
    
    fprintf('Loading EKF covariance...\n');
    ekf_cov = load(fullfile(run_dir, 'ekf_covariance.mat'));
    
    fprintf('Loading sensor data...\n');
    sensors = load(fullfile(run_dir, 'sensor_data.mat'));
    
    fprintf('Loading control data...\n');
    control = load(fullfile(run_dir, 'control_data.mat'));
    
    fprintf('Loading trajectory...\n');
    trajectory = load(fullfile(run_dir, 'trajectory.mat'));
    
    fprintf('Loading parameters...\n');
    params = load(fullfile(run_dir, 'parameters.mat'));
    
    %% Merge into Single Structure
    sim_data = struct();
    
    % Merge all fields
    fields_ekf = fieldnames(ekf_states);
    for i = 1:length(fields_ekf)
        sim_data.(fields_ekf{i}) = ekf_states.(fields_ekf{i});
    end
    
    fields_cov = fieldnames(ekf_cov);
    for i = 1:length(fields_cov)
        sim_data.(fields_cov{i}) = ekf_cov.(fields_cov{i});
    end
    
    fields_sensors = fieldnames(sensors);
    for i = 1:length(fields_sensors)
        sim_data.(fields_sensors{i}) = sensors.(fields_sensors{i});
    end
    
    fields_control = fieldnames(control);
    for i = 1:length(fields_control)
        sim_data.(fields_control{i}) = control.(fields_control{i});
    end
    
    fields_traj = fieldnames(trajectory);
    for i = 1:length(fields_traj)
        sim_data.(fields_traj{i}) = trajectory.(fields_traj{i});
    end
    
    fields_params = fieldnames(params);
    for i = 1:length(fields_params)
        sim_data.(fields_params{i}) = params.(fields_params{i});
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
    fprintf('Total fields loaded: %d\n', length(fieldnames(sim_data)));
    
    if nargout < 2
        fprintf('\n--- METADATA ---\n');
        fprintf('%s\n', metadata);
    end
end
