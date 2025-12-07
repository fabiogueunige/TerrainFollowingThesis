%% ANALYZE_SINGLE_RUN - Complete analysis of a single simulation run
%
% Performs comprehensive analysis by computing ALL performance metrics,
% displaying formatted results, and generating ALL plots with separate figures.
%
% SYNTAX:
%   analyze_single_run(run_name)
%   analyze_single_run()  % Analyzes most recent run
%   analyze_single_run(run_name, 'verbose', true)
%   analyze_single_run(run_name, 'save_plots', true)
%
% INPUTS:
%   run_name     - Name of run to analyze (e.g., 'run_20251201_105813')
%                  If empty/omitted, analyzes the most recent run
%   Name-Value Pairs:
%     'verbose'    - true/false for detailed output (default: true)
%     'save_plots' - true/false to save plots to file (default: false)
%
% WHAT IT DOES:
%   1. Loads simulation data with backward compatibility
%   2. Computes ALL performance metrics:
%      - Altitude tracking (RMS, MAE, settling time)
%      - Angle tracking errors |φ-α| and |θ-β|
%      - EKF SBES estimation accuracy
%      - EKF Position filter accuracy (vs ground truth)
%      - Sensor failure rate
%      - State transitions and occupancy
%      - Control effort (6-DOF RMS)
%      - Innovation statistics
%      - Normal parallelism and robot alignment
%   3. Displays formatted results with quality assessment
%   4. Generates ALL plots (each in separate figure):
%      - State machine transitions
%      - 3D trajectory
%      - Altitude tracking
%      - Terrain angle tracking
%      - Robot angles (roll, pitch, yaw)
%      - EKF SBES states
%      - EKF Position filter states
%      - Control signals
%      - Sensor measurements
%      - Innovation analysis
%      - Normal vector analysis
%      - Covariance evolution
%
% OUTPUTS:
%   None (displays results and plots in console/figures)
%
% EXAMPLES:
%   % Analyze most recent run
%   analyze_single_run();
%   
%   % Analyze specific run
%   analyze_single_run('run_20251201_105813');
%
%   % Analyze with saved plots
%   analyze_single_run('run_20251201_105813', 'save_plots', true);
%
% See also: compute_performance_metrics, display_performance_metrics,
%           plot_results, load_simulation_data

function analyze_single_run(run_name, varargin)
    %% Parse Input Arguments
    p = inputParser;
    addOptional(p, 'run_name', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'verbose', true, @islogical);
    addParameter(p, 'save_plots', false, @islogical);
    parse(p, run_name, varargin{:});
    
    run_name = p.Results.run_name;
    verbose = p.Results.verbose;
    save_plots = p.Results.save_plots;
    
    %% Handle Input Arguments - Find Run
    if isempty(run_name)
        % Find most recent run
        base_dir = 'results';
        runs = dir(fullfile(base_dir, 'run_*'));
        
        if isempty(runs)
            error('No simulation runs found in results/');
        end
        
        % Sort by date to get most recent
        [~, idx] = sort([runs.datenum], 'descend');
        run_name = runs(idx(1)).name;
        
        fprintf('\n=== SINGLE RUN ANALYSIS ===\n');
        fprintf('No run specified, analyzing most recent: %s\n', run_name);
    else
        fprintf('\n=== SINGLE RUN ANALYSIS ===\n');
        fprintf('Analyzing run: %s\n', run_name);
    end
    
    %% STEP 1: Load Simulation Data
    fprintf('\n[1/4] Loading simulation data...\n');
    
    try
        sim_data = load_simulation_data(run_name);
        fprintf('      ✓ Loaded successfully\n');
        fprintf('      - Data version: %.1f\n', sim_data.data_version);
        fprintf('      - Simulation time: %.2f s\n', sim_data.Tf);
        fprintf('      - Time steps: %d\n', sim_data.N);
        fprintf('      - Data fields: %d\n', length(fieldnames(sim_data)));
    catch ME
        error('Failed to load simulation data: %s', ME.message);
    end
    
    %% STEP 2: Compute Performance Metrics
    fprintf('\n[2/4] Computing performance metrics...\n');
    
    try
        metrics = compute_performance_metrics(sim_data);
        fprintf('      ✓ Computed all metrics successfully\n');
    catch ME
        warning('Failed to compute metrics: %s', ME.message);
        fprintf('      ⚠ Continuing without full metrics...\n');
        metrics = struct();
    end
    
    %% STEP 3: Display Performance Results
    fprintf('\n[3/4] Displaying performance results...\n\n');
    
    if ~isempty(fieldnames(metrics))
        try
            % Display with verbose formatting
            display_performance_metrics(metrics, verbose);
        catch ME
            warning('Failed to display metrics: %s', ME.message);
            fprintf('      ⚠ Metrics computed but display failed\n');
        end
    else
        fprintf('      ⚠ No metrics to display\n');
    end
    
    %% STEP 4: Generate All Plots
    fprintf('\n[4/4] Generating all plots...\n');
    
    try
        % Generate comprehensive plots with separate figures
        generate_all_plots(sim_data, run_name, save_plots);
        fprintf('      ✓ All plots generated successfully\n');
    catch ME
        warning('Failed to generate plots: %s', ME.message);
        fprintf('      ⚠ Plot generation encountered errors: %s\n', ME.message);
    end
    
    %% Summary
    fprintf('\n=== ANALYSIS COMPLETE ===\n');
    fprintf('Run: %s\n', run_name);
    
    if isfield(metrics, 'overall')
        fprintf('Overall Performance: %s (%.1f/100)\n', ...
                metrics.overall.grade, metrics.overall.score);
    end
    
    fprintf('\nAll results and plots have been generated.\n');
    fprintf('Use window controls to navigate between figures.\n\n');
end

%% Helper Function: Generate All Plots with Separate Figures
function generate_all_plots(sim_data, run_name, save_plots)
    
    % Extract commonly used data
    time = sim_data.time;
    N = sim_data.N;
    
    % Create plots directory if saving
    if save_plots
        plots_dir = fullfile('results', run_name, 'plots');
        if ~exist(plots_dir, 'dir')
            mkdir(plots_dir);
        end
    end
    
    %% ========================================================================
    %% FIGURE 1: State Machine Transitions
    %% ========================================================================
    fprintf('  - State machine transitions...\n');
    fig1 = figure('Name', 'State Machine Transitions', 'NumberTitle', 'off');
    plot_state_machine(sim_data, time);
    if save_plots
        saveas(fig1, fullfile(plots_dir, 'state_machine.png'));
    end
    
    %% ========================================================================
    %% FIGURE 2: 3D Trajectory
    %% ========================================================================
    fprintf('  - 3D trajectory...\n');
    fig2 = figure('Name', '3D Trajectory', 'NumberTitle', 'off');
    plot_3d_trajectory(sim_data, time);
    if save_plots
        saveas(fig2, fullfile(plots_dir, 'trajectory_3d.png'));
    end
    
    %% ========================================================================
    %% FIGURE 3: Altitude Tracking
    %% ========================================================================
    fprintf('  - Altitude tracking...\n');
    fig3 = figure('Name', 'Altitude Tracking', 'NumberTitle', 'off');
    plot_altitude_tracking(sim_data, time);
    if save_plots
        saveas(fig3, fullfile(plots_dir, 'altitude_tracking.png'));
    end
    
    %% ========================================================================
    %% FIGURE 4: Terrain Angle Tracking (Alpha)
    %% ========================================================================
    fprintf('  - Terrain angle alpha...\n');
    fig4 = figure('Name', 'Terrain Angle Alpha', 'NumberTitle', 'off');
    plot_terrain_angle_alpha(sim_data, time);
    if save_plots
        saveas(fig4, fullfile(plots_dir, 'terrain_alpha.png'));
    end
    
    %% ========================================================================
    %% FIGURE 5: Terrain Angle Tracking (Beta)
    %% ========================================================================
    fprintf('  - Terrain angle beta...\n');
    fig5 = figure('Name', 'Terrain Angle Beta', 'NumberTitle', 'off');
    plot_terrain_angle_beta(sim_data, time);
    if save_plots
        saveas(fig5, fullfile(plots_dir, 'terrain_beta.png'));
    end
    
    %% ========================================================================
    %% FIGURE 6-8: Robot Angles (Roll, Pitch, Yaw)
    %% ========================================================================
    fprintf('  - Robot angles (roll, pitch, yaw)...\n');
    fig6 = figure('Name', 'Robot Roll', 'NumberTitle', 'off');
    plot_robot_roll(sim_data, time);
    if save_plots
        saveas(fig6, fullfile(plots_dir, 'robot_roll.png'));
    end
    
    fig7 = figure('Name', 'Robot Pitch', 'NumberTitle', 'off');
    plot_robot_pitch(sim_data, time);
    if save_plots
        saveas(fig7, fullfile(plots_dir, 'robot_pitch.png'));
    end
    
    fig8 = figure('Name', 'Robot Yaw', 'NumberTitle', 'off');
    plot_robot_yaw(sim_data, time);
    if save_plots
        saveas(fig8, fullfile(plots_dir, 'robot_yaw.png'));
    end
    
    %% ========================================================================
    %% FIGURE 9-14: Control Inputs (6-DOF)
    %% ========================================================================
    fprintf('  - Control inputs (6-DOF)...\n');
    control_names = {'Surge (u)', 'Sway (v)', 'Heave (w)', 'Roll (p)', 'Pitch (q)', 'Yaw (r)'};
    for i = 1:6
        fig = figure('Name', sprintf('Control: %s', control_names{i}), 'NumberTitle', 'off');
        plot_control_input(sim_data, time, i, control_names{i});
        if save_plots
            saveas(fig, fullfile(plots_dir, sprintf('control_%d.png', i)));
        end
    end
    
    %% ========================================================================
    %% FIGURE 15: Normal Vector Parallelism
    %% ========================================================================
    fprintf('  - Normal vector analysis...\n');
    fig15 = figure('Name', 'Normal Vector Parallelism', 'NumberTitle', 'off');
    plot_normal_parallelism(sim_data, time);
    if save_plots
        saveas(fig15, fullfile(plots_dir, 'normal_parallelism.png'));
    end
    
    %% ========================================================================
    %% FIGURE 16: Innovation Analysis
    %% ========================================================================
    fprintf('  - Innovation analysis...\n');
    fig16 = figure('Name', 'EKF Innovation', 'NumberTitle', 'off');
    plot_innovation(sim_data, time);
    if save_plots
        saveas(fig16, fullfile(plots_dir, 'innovation.png'));
    end
    
    %% ========================================================================
    %% FIGURE 17: SBES Measurements
    %% ========================================================================
    fprintf('  - SBES measurements...\n');
    fig17 = figure('Name', 'SBES Measurements', 'NumberTitle', 'off');
    plot_sbes_measurements(sim_data, time);
    if save_plots
        saveas(fig17, fullfile(plots_dir, 'sbes_measurements.png'));
    end
    
    %% ========================================================================
    %% FIGURE 18-20: EKF Position Filter States
    %% ========================================================================
    if isfield(sim_data, 'x_loc') && isfield(sim_data, 'eta_gt') && ~all(sim_data.eta_gt(:) == 0)
        fprintf('  - EKF Position filter states...\n');
        
        fig18 = figure('Name', 'EKF Position: XYZ', 'NumberTitle', 'off');
        plot_ekf_position_xyz(sim_data, time);
        if save_plots
            saveas(fig18, fullfile(plots_dir, 'ekf_pos_xyz.png'));
        end
        
        fig19 = figure('Name', 'EKF Position: Angles', 'NumberTitle', 'off');
        plot_ekf_position_angles(sim_data, time);
        if save_plots
            saveas(fig19, fullfile(plots_dir, 'ekf_pos_angles.png'));
        end
        
        fig20 = figure('Name', 'EKF Position: Velocities', 'NumberTitle', 'off');
        plot_ekf_position_velocities(sim_data, time);
        if save_plots
            saveas(fig20, fullfile(plots_dir, 'ekf_pos_velocities.png'));
        end
        
        fig21 = figure('Name', 'EKF Position: Estimation Errors', 'NumberTitle', 'off');
        plot_ekf_position_errors(sim_data, time);
        if save_plots
            saveas(fig21, fullfile(plots_dir, 'ekf_pos_errors.png'));
        end
    end
    
    %% ========================================================================
    %% FIGURE 22: Sensor Failure History
    %% ========================================================================
    fprintf('  - Sensor failure history...\n');
    fig22 = figure('Name', 'Sensor Failure History', 'NumberTitle', 'off');
    plot_sensor_failures(sim_data, time);
    if save_plots
        saveas(fig22, fullfile(plots_dir, 'sensor_failures.png'));
    end
    
end

%% ============================================================================
%% Individual Plot Functions - Each creates one subplot/figure
%% ============================================================================

function plot_state_machine(sim_data, time)
    % State machine transitions visualization
    if isfield(sim_data, 'state_numeric')
        state_numeric = sim_data.state_numeric;
    else
        state_numeric = zeros(size(time));
    end
    
    state_names = {'Idle', 'Reset', 'TargetAltitude', 'ContactSearch', ...
                   'MovePitch', 'MoveRoll', 'RecoveryAltitude', 'Following', ...
                   'Emergency', 'EndSimulation'};
    if isfield(sim_data, 'state_names')
        state_names = sim_data.state_names;
    end
    
    % Create colorful state plot
    subplot(2,1,1);
    area(time, state_numeric, 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'b');
    xlabel('Time [s]');
    ylabel('State Number');
    title('State Machine - State Over Time');
    yticks(1:length(state_names));
    yticklabels(state_names);
    ylim([0 length(state_names)+1]);
    grid on;
    
    % State transitions markers
    subplot(2,1,2);
    transitions = find(diff(state_numeric) ~= 0);
    stem(time(transitions), ones(size(transitions)), 'b', 'LineWidth', 1.5, 'Marker', 'v');
    xlabel('Time [s]');
    ylabel('Transition');
    title(sprintf('State Transitions (Total: %d)', length(transitions)));
    xlim([time(1) time(end)]);
    grid on;
    
    % Add state occupancy as text annotation
    occupancy = zeros(1, length(state_names));
    for s = 1:length(state_names)
        occupancy(s) = sum(state_numeric == s) / length(state_numeric) * 100;
    end
    
    % Find most occupied states
    [sorted_occ, sorted_idx] = sort(occupancy, 'descend');
    annotation_text = 'Top States:';
    for i = 1:min(3, sum(sorted_occ > 0))
        annotation_text = sprintf('%s\n%s: %.1f%%', annotation_text, ...
            state_names{sorted_idx(i)}, sorted_occ(i));
    end
    annotation('textbox', [0.75 0.4 0.2 0.15], 'String', annotation_text, ...
        'FitBoxToText', 'on', 'BackgroundColor', 'white');
end

function plot_3d_trajectory(sim_data, time)
    prob = sim_data.prob;
    scatter3(prob(1,:), prob(2,:), -prob(3,:), 20, time, 'filled');
    colorbar;
    colormap(jet);
    xlabel('X (North) [m]');
    ylabel('Y (East) [m]');
    zlabel('Z (Down - inverted for viz) [m]');
    title('3D Robot Trajectory (Color = Time)');
    grid on;
    axis equal;
    view(45, 30);
end

function plot_altitude_tracking(sim_data, time)
    h_ref = sim_data.h_ref;
    h_est = sim_data.x_est(1,:);
    h_true = sim_data.x_true(1,:);
    
    subplot(2,1,1);
    plot(time, h_ref, 'b', 'LineWidth', 2, 'DisplayName', 'Reference');
    hold on;
    plot(time, h_true, 'g--', 'LineWidth', 1.5, 'DisplayName', 'True');
    plot(time, h_est, 'r', 'LineWidth', 1, 'DisplayName', 'Estimated');
    xlabel('Time [s]');
    ylabel('Altitude [m]');
    title('Altitude Tracking');
    legend('Location', 'best');
    grid on;
    hold off;
    
    subplot(2,1,2);
    h_error = h_ref - h_est;
    plot(time, h_error, 'b', 'LineWidth', 1);
    hold on;
    yline(0, 'k--');
    yline(rms(h_error), 'r--', sprintf('RMS = %.3f m', rms(h_error)));
    yline(-rms(h_error), 'r--');
    xlabel('Time [s]');
    ylabel('Altitude Error [m]');
    title('Altitude Tracking Error');
    grid on;
    hold off;
end

function plot_terrain_angle_alpha(sim_data, time)
    alpha_true = rad2deg(sim_data.x_true(2,:));
    alpha_est = rad2deg(sim_data.x_est(2,:));
    rob_roll = rad2deg(sim_data.rob_rot(1,:));
    
    subplot(2,1,1);
    plot(time, alpha_true, 'b', 'LineWidth', 1.5, 'DisplayName', 'True Terrain α');
    hold on;
    plot(time, alpha_est, 'r--', 'LineWidth', 1, 'DisplayName', 'Estimated α');
    plot(time, rob_roll, 'g', 'LineWidth', 1, 'DisplayName', 'Robot Roll φ');
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Terrain Angle Alpha vs Robot Roll');
    legend('Location', 'best');
    grid on;
    hold off;
    
    subplot(2,1,2);
    tracking_error = abs(rob_roll - alpha_est);
    plot(time, tracking_error, 'b', 'LineWidth', 1);
    hold on;
    yline(mean(tracking_error), 'r--', sprintf('Mean = %.2f°', mean(tracking_error)));
    xlabel('Time [s]');
    ylabel('|φ - α| [deg]');
    title('Roll-Alpha Tracking Error');
    grid on;
    hold off;
end

function plot_terrain_angle_beta(sim_data, time)
    beta_true = rad2deg(sim_data.x_true(3,:));
    beta_est = rad2deg(sim_data.x_est(3,:));
    rob_pitch = rad2deg(sim_data.rob_rot(2,:));
    
    subplot(2,1,1);
    plot(time, beta_true, 'b', 'LineWidth', 1.5, 'DisplayName', 'True Terrain β');
    hold on;
    plot(time, beta_est, 'r--', 'LineWidth', 1, 'DisplayName', 'Estimated β');
    plot(time, rob_pitch, 'g', 'LineWidth', 1, 'DisplayName', 'Robot Pitch θ');
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Terrain Angle Beta vs Robot Pitch');
    legend('Location', 'best');
    grid on;
    hold off;
    
    subplot(2,1,2);
    tracking_error = abs(rob_pitch - beta_est);
    plot(time, tracking_error, 'b', 'LineWidth', 1);
    hold on;
    yline(mean(tracking_error), 'r--', sprintf('Mean = %.2f°', mean(tracking_error)));
    xlabel('Time [s]');
    ylabel('|θ - β| [deg]');
    title('Pitch-Beta Tracking Error');
    grid on;
    hold off;
end

function plot_robot_roll(sim_data, time)
    roll_noisy = rad2deg(sim_data.rob_rot(1,:));
    roll_clean = rad2deg(sim_data.clean_rot(1,:));
    
    plot(time, roll_clean, 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
    hold on;
    plot(time, roll_noisy, 'r', 'LineWidth', 1, 'DisplayName', 'EKF Estimate');
    if isfield(sim_data, 'goal_roll')
        plot(time, rad2deg(sim_data.goal_roll), 'g--', 'LineWidth', 1, 'DisplayName', 'Goal');
    end
    xlabel('Time [s]');
    ylabel('Roll [deg]');
    title('Robot Roll Angle');
    legend('Location', 'best');
    grid on;
    hold off;
end

function plot_robot_pitch(sim_data, time)
    pitch_noisy = rad2deg(sim_data.rob_rot(2,:));
    pitch_clean = rad2deg(sim_data.clean_rot(2,:));
    
    plot(time, pitch_clean, 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
    hold on;
    plot(time, pitch_noisy, 'r', 'LineWidth', 1, 'DisplayName', 'EKF Estimate');
    if isfield(sim_data, 'goal_pitch')
        plot(time, rad2deg(sim_data.goal_pitch), 'g--', 'LineWidth', 1, 'DisplayName', 'Goal');
    end
    xlabel('Time [s]');
    ylabel('Pitch [deg]');
    title('Robot Pitch Angle');
    legend('Location', 'best');
    grid on;
    hold off;
end

function plot_robot_yaw(sim_data, time)
    yaw_noisy = rad2deg(sim_data.rob_rot(3,:));
    yaw_clean = rad2deg(sim_data.clean_rot(3,:));
    
    plot(time, yaw_clean, 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
    hold on;
    plot(time, yaw_noisy, 'r', 'LineWidth', 1, 'DisplayName', 'EKF Estimate');
    xlabel('Time [s]');
    ylabel('Yaw [deg]');
    title('Robot Yaw Angle');
    legend('Location', 'best');
    grid on;
    hold off;
end

function plot_control_input(sim_data, time, idx, name)
    u = sim_data.u(idx,:);
    
    if idx <= 3
        % Linear velocities
        plot(time, u, 'b', 'LineWidth', 1);
        ylabel('Velocity [m/s]');
    else
        % Angular velocities
        plot(time, rad2deg(u), 'b', 'LineWidth', 1);
        ylabel('Angular Rate [deg/s]');
    end
    
    xlabel('Time [s]');
    title(sprintf('Control Input: %s', name));
    grid on;
    
    % Add RMS annotation
    if idx <= 3
        rms_val = rms(u);
        annotation_str = sprintf('RMS = %.4f m/s', rms_val);
    else
        rms_val = rad2deg(rms(u));
        annotation_str = sprintf('RMS = %.4f deg/s', rms_val);
    end
    text(0.02, 0.95, annotation_str, 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'BackgroundColor', 'white');
end

function plot_normal_parallelism(sim_data, time)
    n_est = sim_data.n_est;
    n_mes = sim_data.n_mes;
    wRr = sim_data.wRr;
    N = length(time);
    
    % Compute angles
    aa12 = zeros(1, N);  % Est vs Mes
    aa13 = zeros(1, N);  % Est vs Robot
    aa23 = zeros(1, N);  % Mes vs Robot
    
    for i = 1:N
        if ~any(isnan(n_est(:,i))) && ~any(isnan(n_mes(:,i)))
            aa12(i) = acosd(abs(dot(n_est(:,i)/norm(n_est(:,i)), n_mes(:,i)/norm(n_mes(:,i)))));
            z_robot = wRr(:,3,i);
            aa13(i) = acosd(abs(dot(n_est(:,i)/norm(n_est(:,i)), z_robot/norm(z_robot))));
            aa23(i) = acosd(abs(dot(n_mes(:,i)/norm(n_mes(:,i)), z_robot/norm(z_robot))));
        else
            aa12(i) = NaN;
            aa13(i) = NaN;
            aa23(i) = NaN;
        end
    end
    
    subplot(2,1,1);
    plot(time, aa12, 'r', 'LineWidth', 1, 'DisplayName', 'Est vs Mes');
    hold on;
    plot(time, aa13, 'g', 'LineWidth', 1, 'DisplayName', 'Est vs Robot');
    plot(time, aa23, 'b', 'LineWidth', 1, 'DisplayName', 'Mes vs Robot');
    yline(5, 'k--', 'LineWidth', 1, 'DisplayName', '5° threshold');
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Normal Vector Parallelism');
    legend('Location', 'best');
    grid on;
    hold off;
    
    subplot(2,1,2);
    % Z-component of normals
    plot(time, n_est(3,:), 'r', 'LineWidth', 1, 'DisplayName', 'n_est Z');
    hold on;
    plot(time, n_mes(3,:), 'g', 'LineWidth', 1, 'DisplayName', 'n_mes Z');
    xlabel('Time [s]');
    ylabel('Z component');
    title('Normal Vector Z-Component');
    legend('Location', 'best');
    grid on;
    hold off;
end

function plot_innovation(sim_data, time)
    ni = sim_data.ni;
    
    subplot(2,1,1);
    for s = 1:size(ni, 1)
        plot(time, ni(s,:), 'DisplayName', sprintf('Sensor %d', s));
        hold on;
    end
    xlabel('Time [s]');
    ylabel('Innovation');
    title('EKF Innovation (per sensor)');
    legend('Location', 'best');
    grid on;
    hold off;
    
    subplot(2,1,2);
    innovation_norm = sqrt(sum(ni.^2, 1));
    plot(time, innovation_norm, 'b', 'LineWidth', 1);
    hold on;
    yline(mean(innovation_norm), 'r--', sprintf('Mean = %.3f', mean(innovation_norm)));
    xlabel('Time [s]');
    ylabel('||ν||');
    title('Innovation Norm');
    grid on;
    hold off;
end

function plot_sbes_measurements(sim_data, time)
    z_meas = sim_data.z_meas;
    z_pred = sim_data.z_pred;
    
    for s = 1:size(z_meas, 1)
        subplot(2, 2, s);
        plot(time, z_meas(s,:), 'b', 'LineWidth', 1, 'DisplayName', 'Measured');
        hold on;
        plot(time, z_pred(s,:), 'r--', 'LineWidth', 1, 'DisplayName', 'Predicted');
        xlabel('Time [s]');
        ylabel('Range [m]');
        title(sprintf('SBES Sensor %d', s));
        legend('Location', 'best');
        grid on;
        hold off;
    end
end

function plot_ekf_position_xyz(sim_data, time)
    eta_gt = sim_data.eta_gt;
    x_loc = sim_data.x_loc;
    
    pos_labels = {'X (North)', 'Y (East)', 'Z (Down)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, eta_gt(i,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
        hold on;
        plot(time, x_loc(i,:), 'r--', 'LineWidth', 1, 'DisplayName', 'EKF Estimate');
        xlabel('Time [s]');
        ylabel([pos_labels{i} ' [m]']);
        title(['Position ' pos_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
end

function plot_ekf_position_angles(sim_data, time)
    eta_gt = sim_data.eta_gt;
    x_loc = sim_data.x_loc;
    
    ang_labels = {'Roll (φ)', 'Pitch (θ)', 'Yaw (ψ)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, rad2deg(eta_gt(3+i,:)), 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
        hold on;
        plot(time, rad2deg(x_loc(3+i,:)), 'r--', 'LineWidth', 1, 'DisplayName', 'EKF Estimate');
        xlabel('Time [s]');
        ylabel([ang_labels{i} ' [deg]']);
        title(['Orientation ' ang_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
end

function plot_ekf_position_velocities(sim_data, time)
    nu_gt = sim_data.nu_gt;
    x_loc = sim_data.x_loc;
    
    vel_labels = {'Surge (u)', 'Sway (v)', 'Heave (w)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, nu_gt(i,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
        hold on;
        plot(time, x_loc(6+i,:), 'r--', 'LineWidth', 1, 'DisplayName', 'EKF Estimate');
        xlabel('Time [s]');
        ylabel([vel_labels{i} ' [m/s]']);
        title(['Velocity ' vel_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
end

function plot_ekf_position_errors(sim_data, time)
    eta_gt = sim_data.eta_gt;
    x_loc = sim_data.x_loc;
    nu_gt = sim_data.nu_gt;
    
    % Position errors
    subplot(2,2,1);
    pos_err = x_loc(1:3,:) - eta_gt(1:3,:);
    plot(time, pos_err(1,:), 'r', 'DisplayName', 'X');
    hold on;
    plot(time, pos_err(2,:), 'g', 'DisplayName', 'Y');
    plot(time, pos_err(3,:), 'b', 'DisplayName', 'Z');
    xlabel('Time [s]');
    ylabel('Error [m]');
    title('Position Estimation Errors');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % Orientation errors
    subplot(2,2,2);
    ang_err = rad2deg(x_loc(4:6,:) - eta_gt(4:6,:));
    plot(time, ang_err(1,:), 'r', 'DisplayName', 'φ');
    hold on;
    plot(time, ang_err(2,:), 'g', 'DisplayName', 'θ');
    plot(time, ang_err(3,:), 'b', 'DisplayName', 'ψ');
    xlabel('Time [s]');
    ylabel('Error [deg]');
    title('Orientation Estimation Errors');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % Velocity errors
    subplot(2,2,3);
    vel_err = x_loc(7:9,:) - nu_gt(1:3,:);
    plot(time, vel_err(1,:), 'r', 'DisplayName', 'u');
    hold on;
    plot(time, vel_err(2,:), 'g', 'DisplayName', 'v');
    plot(time, vel_err(3,:), 'b', 'DisplayName', 'w');
    xlabel('Time [s]');
    ylabel('Error [m/s]');
    title('Velocity Estimation Errors');
    legend('Location', 'best');
    grid on;
    hold off;
    
    % Angular rate errors
    subplot(2,2,4);
    rate_err = rad2deg(x_loc(10:12,:) - nu_gt(4:6,:));
    plot(time, rate_err(1,:), 'r', 'DisplayName', 'p');
    hold on;
    plot(time, rate_err(2,:), 'g', 'DisplayName', 'q');
    plot(time, rate_err(3,:), 'b', 'DisplayName', 'r');
    xlabel('Time [s]');
    ylabel('Error [deg/s]');
    title('Angular Rate Estimation Errors');
    legend('Location', 'best');
    grid on;
    hold off;
end

function plot_sensor_failures(sim_data, time)
    sensor_fail = sim_data.sensor_fail;
    
    subplot(2,1,1);
    area(time, sensor_fail, 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'r');
    xlabel('Time [s]');
    ylabel('# Failed Sensors');
    title('Sensor Failures Over Time');
    ylim([0 4.5]);
    grid on;
    
    subplot(2,1,2);
    % Per-sensor failure (from z_meas NaN)
    if isfield(sim_data, 'z_meas')
        z_meas = sim_data.z_meas;
        for s = 1:size(z_meas, 1)
            sensor_fail_s = double(isnan(z_meas(s,:)) | isinf(z_meas(s,:)));
            plot(time, sensor_fail_s + (s-1)*1.2, 'LineWidth', 1.5, ...
                'DisplayName', sprintf('Sensor %d', s));
            hold on;
        end
        xlabel('Time [s]');
        ylabel('Sensor State (offset per sensor)');
        title('Individual Sensor Failures (1=fail)');
        legend('Location', 'best');
        grid on;
        hold off;
    else
        text(0.5, 0.5, 'z_meas not available', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center');
    end
end

