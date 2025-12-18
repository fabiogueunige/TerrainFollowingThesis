%% COMPUTE_PERFORMANCE_METRICS - Calculate all performance metrics for terrain following
%
% Computes comprehensive performance metrics for terrain following simulation
% including EKF SBES, EKF Position, controllers, sensors, and state machine.
%
% SYNTAX:
%   metrics = compute_performance_metrics(sim_data)
%
% INPUTS:
%   sim_data - Structure from load_simulation_data containing all simulation data
%
% OUTPUTS:
%   metrics - Structure containing:
%             .altitude        - Altitude tracking metrics (RMS, MAE, max error)
%             .angle_tracking  - Terrain angle tracking (phi-alpha, theta-beta)
%             .sensors         - Sensor failure rate and reliability
%             .state_machine   - State transitions and occupancy
%             .control         - Control effort per axis
%             .ekf_sbes        - EKF SBES innovation and consistency
%             .ekf_position    - EKF Position filter accuracy (vs ground truth)
%             .geometry        - Normal parallelism and robot alignment
%             .overall         - Overall performance score
%
% BACKWARD COMPATIBILITY:
%   Handles missing fields gracefully - returns NaN or empty structs
%
% EXAMPLE:
%   sim_data = load_simulation_data('run_20251201_120426');
%   metrics = compute_performance_metrics(sim_data);
%   fprintf('RMS altitude error: %.4f m\n', metrics.altitude.rms_error);
%
% See also: load_simulation_data, display_performance_metrics, analyze_single_run

function metrics = compute_performance_metrics(sim_data)
    
    fprintf('\n=== COMPUTING PERFORMANCE METRICS ===\n');
    
    % Get basic parameters
    N = get_field_or_default(sim_data, 'N', size(sim_data.x_est, 2));
    Ts = get_field_or_default(sim_data, 'Ts', 0.001);
    Tf = get_field_or_default(sim_data, 'Tf', N * Ts);
    
    %% ========================================================================
    %% 1. ALTITUDE TRACKING METRICS
    %% ========================================================================
    fprintf('Computing altitude tracking metrics...\n');
    
    h_ref = get_field_or_default(sim_data, 'h_ref', []);
    x_est = get_field_or_default(sim_data, 'x_est', []);
    
    if ~isempty(h_ref) && ~isempty(x_est)
        h_error = h_ref - x_est(1,:);
        
        metrics.altitude.rms_error = rms(h_error);
        metrics.altitude.mae = mean(abs(h_error));
        metrics.altitude.max_error = max(abs(h_error));
        metrics.altitude.std_error = std(h_error);
        metrics.altitude.mean_error = mean(h_error);
        metrics.altitude.final_error = h_error(end);
        
        % Transient analysis (first 20% vs steady state 80%)
        transient_end = round(0.2 * N);
        metrics.altitude.transient_rms = rms(h_error(1:transient_end));
        metrics.altitude.steady_state_rms = rms(h_error(transient_end+1:end));
        
        % Settling time (time to stay within 5% of final value)
        threshold = 0.05 * abs(h_ref(end));
        settled = abs(h_error) < max(threshold, 0.1);  % At least 0.1m tolerance
        settling_idx = find(settled, 1, 'first');
        if ~isempty(settling_idx)
            metrics.altitude.settling_time = settling_idx * Ts;
        else
            metrics.altitude.settling_time = Tf;
        end
    else
        metrics.altitude = struct();
    end
    
    %% ========================================================================
    %% 2. TERRAIN ANGLE TRACKING METRICS (Robot vs Terrain)
    %% ========================================================================
    fprintf('Computing angle tracking metrics...\n');
    
    rob_rot = get_field_or_default(sim_data, 'rob_rot', []);
    x_true = get_field_or_default(sim_data, 'x_true', []);
    
    if ~isempty(rob_rot) && ~isempty(x_est) && size(rob_rot, 1) >= 2
        % Robot angles (from EKF position filter)
        phi_robot = rob_rot(1,:);     % Roll
        theta_robot = rob_rot(2,:);   % Pitch
        
        % Terrain angles (from EKF SBES state)
        alpha = x_est(2,:);  % Terrain roll angle
        beta = x_est(3,:);   % Terrain pitch angle
        
        % Tracking errors |robot - terrain|
        phi_alpha_error = abs(phi_robot - alpha);
        theta_beta_error = abs(theta_robot - beta);
        
        metrics.angle_tracking.phi_alpha_mean = mean(phi_alpha_error);
        metrics.angle_tracking.phi_alpha_std = std(phi_alpha_error);
        metrics.angle_tracking.phi_alpha_max = max(phi_alpha_error);
        metrics.angle_tracking.phi_alpha_rms = rms(phi_alpha_error);
        
        metrics.angle_tracking.theta_beta_mean = mean(theta_beta_error);
        metrics.angle_tracking.theta_beta_std = std(theta_beta_error);
        metrics.angle_tracking.theta_beta_max = max(theta_beta_error);
        metrics.angle_tracking.theta_beta_rms = rms(theta_beta_error);
        
        metrics.angle_tracking.combined_mean = mean([phi_alpha_error, theta_beta_error]);
        metrics.angle_tracking.combined_rms = rms([phi_alpha_error, theta_beta_error]);
    else
        metrics.angle_tracking = struct();
    end
    
    %% ========================================================================
    %% 3. EKF SBES ESTIMATION ERRORS (True vs Estimated)
    %% ========================================================================
    fprintf('Computing EKF SBES estimation metrics...\n');
    
    if ~isempty(x_true) && ~isempty(x_est)
        % Alpha estimation error
        alpha_error = x_true(2,:) - x_est(2,:);
        metrics.ekf_sbes.alpha_mae = mean(abs(alpha_error));
        metrics.ekf_sbes.alpha_rmse = rms(alpha_error);
        metrics.ekf_sbes.alpha_max = max(abs(alpha_error));
        
        % Beta estimation error
        beta_error = x_true(3,:) - x_est(3,:);
        metrics.ekf_sbes.beta_mae = mean(abs(beta_error));
        metrics.ekf_sbes.beta_rmse = rms(beta_error);
        metrics.ekf_sbes.beta_max = max(abs(beta_error));
        
        % Altitude estimation error
        h_est_error = x_true(1,:) - x_est(1,:);
        metrics.ekf_sbes.h_mae = mean(abs(h_est_error));
        metrics.ekf_sbes.h_rmse = rms(h_est_error);
        metrics.ekf_sbes.h_max = max(abs(h_est_error));
    end
    
    % Innovation statistics
    ni = get_field_or_default(sim_data, 'ni', []);
    if ~isempty(ni)
        innovation_norms = sqrt(sum(ni.^2, 1));
        metrics.ekf_sbes.innovation_mean = mean(innovation_norms);
        metrics.ekf_sbes.innovation_max = max(innovation_norms);
        metrics.ekf_sbes.innovation_std = std(innovation_norms);
        
        % Normalized innovation (if covariance available)
        S = get_field_or_default(sim_data, 'S', []);
        if ~isempty(S) && ndims(S) == 3
            nis = zeros(1, N);  % Normalized Innovation Squared
            for k = 1:N
                if det(S(:,:,k)) > eps
                    nis(k) = ni(:,k)' / S(:,:,k) * ni(:,k);
                end
            end
            metrics.ekf_sbes.nis_mean = mean(nis);
            metrics.ekf_sbes.nis_max = max(nis);
            % NEES consistency: should be around n_meas for consistent filter
            metrics.ekf_sbes.consistency_ratio = mean(nis) / size(ni, 1);
        end
    else
        metrics.ekf_sbes.innovation_mean = NaN;
        metrics.ekf_sbes.innovation_max = NaN;
    end
    
    %% ========================================================================
    %% 4. SENSOR PERFORMANCE
    %% ========================================================================
    fprintf('Computing sensor metrics...\n');
    
    sensor_fail = get_field_or_default(sim_data, 'sensor_fail', []);
    z_meas = get_field_or_default(sim_data, 'z_meas', []);
    
    if ~isempty(sensor_fail)
        total_steps = length(sensor_fail);
        failed_steps = sum(sensor_fail > 0);
        
        metrics.sensors.failure_rate = (failed_steps / total_steps) * 100;
        metrics.sensors.total_failures = sum(sensor_fail);
        metrics.sensors.mean_failures_per_step = mean(sensor_fail);
        metrics.sensors.max_simultaneous_failures = max(sensor_fail);
        
        % Per-sensor failure rate
        if ~isempty(z_meas)
            for s = 1:size(z_meas, 1)
                sensor_fails = sum(isnan(z_meas(s,:)) | isinf(z_meas(s,:)));
                metrics.sensors.(['sensor_' num2str(s) '_failure_rate']) = sensor_fails / N * 100;
            end
        end
        
        % Time between failures
        failure_indices = find(sensor_fail > 0);
        if length(failure_indices) > 1
            time_between = diff(failure_indices) * Ts;
            metrics.sensors.mean_time_between_failures = mean(time_between);
        else
            metrics.sensors.mean_time_between_failures = Tf;
        end
    else
        metrics.sensors.failure_rate = 0;
    end
    
    %% ========================================================================
    %% 5. STATE MACHINE METRICS
    %% ========================================================================
    fprintf('Computing state machine metrics...\n');
    
    state = get_field_or_default(sim_data, 'state', []);
    state_numeric = get_field_or_default(sim_data, 'state_numeric', []);
    
    if ~isempty(state_numeric)
        % Transitions
        state_changes = sum(diff(state_numeric) ~= 0);
        duration_min = Tf / 60;
        
        metrics.state_machine.total_transitions = state_changes;
        metrics.state_machine.transitions_per_minute = state_changes / duration_min;
        metrics.state_machine.duration_min = duration_min;
        
        % State occupancy
        state_names = get_field_or_default(sim_data, 'state_names', ...
            {'Idle', 'Reset', 'TargetAltitude', 'ContactSearch', ...
             'MovePitch', 'MoveRoll', 'RecoveryAltitude', 'Following', ...
             'Emergency', 'EndSimulation'});
        
        metrics.state_machine.state_names = state_names;
        metrics.state_machine.occupancy = zeros(1, length(state_names));
        metrics.state_machine.time_in_state = zeros(1, length(state_names));
        
        for s = 1:length(state_names)
            count = sum(state_numeric == s);
            metrics.state_machine.occupancy(s) = count / N * 100;
            metrics.state_machine.time_in_state(s) = count * Ts;
        end
        
        % Following state time (important for terrain following success)
        following_idx = find(strcmp(state_names, 'Following'));
        if ~isempty(following_idx)
            metrics.state_machine.following_percentage = metrics.state_machine.occupancy(following_idx);
            metrics.state_machine.following_time = metrics.state_machine.time_in_state(following_idx);
        else
            metrics.state_machine.following_percentage = 0;
            metrics.state_machine.following_time = 0;
        end
        
        % Recovery states time
        recovery_states = {'MovePitch', 'MoveRoll', 'RecoveryAltitude', 'Reset'};
        recovery_time = 0;
        for rs = 1:length(recovery_states)
            idx = find(strcmp(state_names, recovery_states{rs}));
            if ~isempty(idx)
                recovery_time = recovery_time + metrics.state_machine.time_in_state(idx);
            end
        end
        metrics.state_machine.recovery_time = recovery_time;
        metrics.state_machine.recovery_percentage = recovery_time / Tf * 100;
    else
        metrics.state_machine = struct();
        metrics.state_machine.total_transitions = 0;
        metrics.state_machine.transitions_per_minute = 0;
    end
    
    %% ========================================================================
    %% 6. CONTROL EFFORT
    %% ========================================================================
    fprintf('Computing control effort metrics...\n');
    
    u = get_field_or_default(sim_data, 'u', []);
    pid = get_field_or_default(sim_data, 'pid', []);
    
    if ~isempty(u)
        % Per-axis RMS
        metrics.control.surge_rms = rms(u(1,:));
        metrics.control.sway_rms = rms(u(2,:));
        metrics.control.heave_rms = rms(u(3,:));
        metrics.control.roll_rms = rms(u(4,:));
        metrics.control.pitch_rms = rms(u(5,:));
        metrics.control.yaw_rms = rms(u(6,:));
        
        % Total effort
        control_magnitude = sqrt(sum(u.^2, 1));
        metrics.control.total_effort_rms = rms(control_magnitude);
        metrics.control.total_effort_mean = mean(control_magnitude);
        metrics.control.total_effort_max = max(control_magnitude);
        metrics.control.total_effort_std = std(control_magnitude);
        
        % Per-axis max
        metrics.control.surge_max = max(abs(u(1,:)));
        metrics.control.sway_max = max(abs(u(2,:)));
        metrics.control.heave_max = max(abs(u(3,:)));
        metrics.control.roll_max = max(abs(u(4,:)));
        metrics.control.pitch_max = max(abs(u(5,:)));
        metrics.control.yaw_max = max(abs(u(6,:)));
        
        % Energy consumption proxy (integral of squared control)
        metrics.control.energy = sum(sum(u.^2, 1)) * Ts;
    else
        metrics.control = struct();
    end
    
    % PID output analysis
    if ~isempty(pid)
        metrics.control.pid_rms = rms(pid, 2)';
        metrics.control.pid_max = max(abs(pid), [], 2)';
    end
    
    %% ========================================================================
    %% 7. GEOMETRY - Normal Vector Analysis
    %% ========================================================================
    fprintf('Computing geometry metrics...\n');
    
    n_est = get_field_or_default(sim_data, 'n_est', []);
    n_mes = get_field_or_default(sim_data, 'n_mes', []);
    wRr = get_field_or_default(sim_data, 'wRr', []);
    
    % Normal parallelism: angle between estimated and measured normals
    if ~isempty(n_est) && ~isempty(n_mes)
        parallelism_angles = zeros(1, N);
        
        for i = 1:N
            n_e = n_est(:,i);
            n_m = n_mes(:,i);
            
            % Skip if NaN
            if any(isnan(n_e)) || any(isnan(n_m))
                parallelism_angles(i) = NaN;
                continue;
            end
            
            % Normalize vectors
            n_e = n_e / max(norm(n_e), eps);
            n_m = n_m / max(norm(n_m), eps);
            
            % Compute angle (in degrees)
            cos_angle = abs(dot(n_e, n_m));
            cos_angle = min(1.0, max(-1.0, cos_angle));
            parallelism_angles(i) = acos(cos_angle) * 180/pi;
        end
        
        valid = ~isnan(parallelism_angles);
        metrics.geometry.normal_parallelism_mean = mean(parallelism_angles(valid));
        metrics.geometry.normal_parallelism_std = std(parallelism_angles(valid));
        metrics.geometry.normal_parallelism_max = max(parallelism_angles(valid));
        metrics.geometry.normal_parallelism_below_5deg = sum(parallelism_angles(valid) < 5) / sum(valid) * 100;
    else
        metrics.geometry.normal_parallelism_mean = NaN;
    end
    
    % Robot alignment: angle between robot Z-axis and estimated normal
    if ~isempty(n_est) && ~isempty(wRr) && ndims(wRr) == 3
        alignment_angles = zeros(1, N);
        
        for i = 1:N
            R = wRr(:,:,i);
            z_robot = R(:,3);  % Third column is Z-axis in world frame
            n_e = n_est(:,i);
            
            if any(isnan(n_e)) || any(isnan(z_robot))
                alignment_angles(i) = NaN;
                continue;
            end
            
            % Normalize
            z_robot = z_robot / max(norm(z_robot), eps);
            n_e = n_e / max(norm(n_e), eps);
            
            % Compute angle (in degrees)
            cos_angle = abs(dot(z_robot, n_e));
            cos_angle = min(1.0, max(-1.0, cos_angle));
            alignment_angles(i) = acos(cos_angle) * 180/pi;
        end
        
        valid = ~isnan(alignment_angles);
        metrics.geometry.robot_alignment_mean = mean(alignment_angles(valid));
        metrics.geometry.robot_alignment_std = std(alignment_angles(valid));
        metrics.geometry.robot_alignment_max = max(alignment_angles(valid));
    else
        metrics.geometry.robot_alignment_mean = NaN;
    end
    
    %% ========================================================================
    %% 8. EKF POSITION FILTER PERFORMANCE (vs Ground Truth)
    %% ========================================================================
    fprintf('Computing EKF Position filter metrics...\n');
    
    x_loc = get_field_or_default(sim_data, 'x_loc', []);
    eta_gt = get_field_or_default(sim_data, 'eta_gt', []);
    nu_gt = get_field_or_default(sim_data, 'nu_gt', []);
    
    if ~isempty(x_loc) && ~isempty(eta_gt) && size(x_loc, 1) >= 12 && ~all(eta_gt(:) == 0)
        % Position errors [m]
        pos_err = x_loc(1:3,:) - eta_gt(1:3,:);
        metrics.ekf_position.pos_rmse_x = rms(pos_err(1,:));
        metrics.ekf_position.pos_rmse_y = rms(pos_err(2,:));
        metrics.ekf_position.pos_rmse_z = rms(pos_err(3,:));
        metrics.ekf_position.pos_rmse_total = norm([metrics.ekf_position.pos_rmse_x, ...
                                                    metrics.ekf_position.pos_rmse_y, ...
                                                    metrics.ekf_position.pos_rmse_z]);
        metrics.ekf_position.pos_max_err = max(abs(pos_err(:)));
        
        % Orientation errors [deg]
        ang_err = x_loc(4:6,:) - eta_gt(4:6,:);
        metrics.ekf_position.ang_rmse_roll = rad2deg(rms(ang_err(1,:)));
        metrics.ekf_position.ang_rmse_pitch = rad2deg(rms(ang_err(2,:)));
        metrics.ekf_position.ang_rmse_yaw = rad2deg(rms(ang_err(3,:)));
        metrics.ekf_position.ang_max_err = rad2deg(max(abs(ang_err(:))));
        
        % Linear velocity errors [m/s]
        if ~isempty(nu_gt) && ~all(nu_gt(:) == 0)
            vel_err = x_loc(7:9,:) - nu_gt(1:3,:);
            metrics.ekf_position.vel_rmse_surge = rms(vel_err(1,:));
            metrics.ekf_position.vel_rmse_sway = rms(vel_err(2,:));
            metrics.ekf_position.vel_rmse_heave = rms(vel_err(3,:));
            metrics.ekf_position.vel_max_err = max(abs(vel_err(:)));
            
            % Angular velocity errors [deg/s]
            rate_err = x_loc(10:12,:) - nu_gt(4:6,:);
            metrics.ekf_position.rate_rmse_p = rad2deg(rms(rate_err(1,:)));
            metrics.ekf_position.rate_rmse_q = rad2deg(rms(rate_err(2,:)));
            metrics.ekf_position.rate_rmse_r = rad2deg(rms(rate_err(3,:)));
            metrics.ekf_position.rate_max_err = rad2deg(max(abs(rate_err(:))));
        end
        
        % Gyro bias estimation (if available)
        if size(x_loc, 1) >= 15
            metrics.ekf_position.bias_x_final = rad2deg(x_loc(13, end));
            metrics.ekf_position.bias_y_final = rad2deg(x_loc(14, end));
            metrics.ekf_position.bias_z_final = rad2deg(x_loc(15, end));
        end
    else
        metrics.ekf_position = struct();
        metrics.ekf_position.pos_rmse_total = NaN;
    end
    
    %% ========================================================================
    %% 9. OVERALL PERFORMANCE SCORE
    %% ========================================================================
    fprintf('Computing overall performance score...\n');
    
    score = 0;
    max_score = 0;
    
    % Altitude tracking (25 points)
    if isfield(metrics, 'altitude') && isfield(metrics.altitude, 'rms_error')
        max_score = max_score + 25;
        if metrics.altitude.rms_error < 0.2
            score = score + 25;
        elseif metrics.altitude.rms_error < 0.5
            score = score + 20;
        elseif metrics.altitude.rms_error < 1.0
            score = score + 15;
        elseif metrics.altitude.rms_error < 2.0
            score = score + 10;
        end
    end
    
    % Following time percentage (25 points)
    if isfield(metrics, 'state_machine') && isfield(metrics.state_machine, 'following_percentage')
        max_score = max_score + 25;
        following_pct = metrics.state_machine.following_percentage;
        score = score + min(25, following_pct / 4);  % 100% following = 25 points
    end
    
    % Sensor reliability (20 points)
    if isfield(metrics, 'sensors') && isfield(metrics.sensors, 'failure_rate')
        max_score = max_score + 20;
        if metrics.sensors.failure_rate < 5
            score = score + 20;
        elseif metrics.sensors.failure_rate < 10
            score = score + 15;
        elseif metrics.sensors.failure_rate < 20
            score = score + 10;
        elseif metrics.sensors.failure_rate < 30
            score = score + 5;
        end
    end
    
    % Angle tracking (15 points)
    if isfield(metrics, 'angle_tracking') && isfield(metrics.angle_tracking, 'combined_mean')
        max_score = max_score + 15;
        combined_deg = rad2deg(metrics.angle_tracking.combined_mean);
        if combined_deg < 3
            score = score + 15;
        elseif combined_deg < 5
            score = score + 12;
        elseif combined_deg < 10
            score = score + 8;
        elseif combined_deg < 15
            score = score + 5;
        end
    end
    
    % Control effort (15 points - lower is better)
    if isfield(metrics, 'control') && isfield(metrics.control, 'total_effort_rms')
        max_score = max_score + 15;
        effort = metrics.control.total_effort_rms;
        if effort < 0.5
            score = score + 15;
        elseif effort < 1.0
            score = score + 12;
        elseif effort < 2.0
            score = score + 8;
        elseif effort < 5.0
            score = score + 5;
        end
    end
    
    % Calculate overall score
    if max_score > 0
        metrics.overall.score = (score / max_score) * 100;
    else
        metrics.overall.score = 0;
    end
    
    % Assign grade
    if metrics.overall.score >= 90
        metrics.overall.grade = 'A - Excellent';
    elseif metrics.overall.score >= 80
        metrics.overall.grade = 'B - Good';
    elseif metrics.overall.score >= 70
        metrics.overall.grade = 'C - Acceptable';
    elseif metrics.overall.score >= 60
        metrics.overall.grade = 'D - Marginal';
    else
        metrics.overall.grade = 'F - Needs Improvement';
    end
    
    fprintf('Overall performance: %.1f/100 (%s)\n\n', metrics.overall.score, metrics.overall.grade);
end

%% Helper Function: Get field or default value
function value = get_field_or_default(s, field, default)
    if isfield(s, field)
        value = s.(field);
    else
        value = default;
    end
end
