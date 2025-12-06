%% COMPUTE_PERFORMANCE_METRICS - Calculate all performance metrics from RESULTS_GUIDE
%
% Computes comprehensive performance metrics as specified in RESULTS_GUIDE_2.pdf
% Section 2.4.2 and 2.4.3.
%
% SYNTAX:
%   metrics = compute_performance_metrics(sim_data)
%
% INPUTS:
%   sim_data - Structure from load_simulation_data containing:
%              .x_true, .x_est, .h_ref - State vectors
%              .wRr_noisy, .wRr - Robot orientation (with/without noise)
%              .n_est, .n_mes - Estimated and measured normals
%              .u - Control inputs
%              .sensor_fail - Sensor failure indicator
%              .state - State machine history
%              .t - Time vector
%              .ni - Innovation
%
% OUTPUTS:
%   metrics - Structure containing:
%             .altitude_tracking   - RMS altitude error
%             .angle_tracking      - Mean angle errors (phi-alpha, theta-beta)
%             .sensor_failure_rate - Percentage of time with sensor_fail > 0
%             .state_transitions   - Number of transitions per minute
%             .control_effort      - Time-averaged PID output magnitude
%             .innovation_max      - Maximum innovation magnitude
%             .normal_parallelism  - Mean angle between n_est and n_mes
%             .robot_alignment     - Mean angle between robot Z and n_est
%
% REFERENCE:
%   RESULTS_GUIDE_2.pdf Section 2.4.3 "Batch Analysis and Statistical Evaluation"
%
% EXAMPLE:
%   sim_data = load_simulation_data('run_20251201_120426');
%   metrics = compute_performance_metrics(sim_data);
%   fprintf('RMS altitude error: %.4f m\n', metrics.altitude_tracking);
%
% See also: load_simulation_data, analyze_statistics, replay_plots

function metrics = compute_performance_metrics(sim_data)
    
    %% 1. RMS Altitude Error
    % From guide: "RMS altitude error quantifying altitude-holding precision"
    h_error = sim_data.h_ref - sim_data.x_est(1,:);
    metrics.altitude_tracking = rms(h_error);
    
    %% 2. Mean Angle Tracking Error
    % From guide: "Average absolute difference between estimated and true terrain angles"
    
    % Get robot orientation (from rob_rot which is eta(4:6,:))
    N = size(sim_data.rob_rot, 2);
    phi_measured = sim_data.rob_rot(1,:);    % Roll from EKF position filter
    theta_measured = sim_data.rob_rot(2,:);  % Pitch from EKF position filter
    
    % Terrain angles from SBES EKF state
    alpha = sim_data.x_est(2,:);  % Roll angle of terrain
    beta = sim_data.x_est(3,:);   % Pitch angle of terrain
    
    % Mean angle tracking errors
    phi_error = abs(phi_measured - alpha);
    theta_error = abs(theta_measured - beta);
    
    metrics.angle_tracking.phi_alpha_mean = mean(phi_error);
    metrics.angle_tracking.phi_alpha_std = std(phi_error);
    metrics.angle_tracking.theta_beta_mean = mean(theta_error);
    metrics.angle_tracking.theta_beta_std = std(theta_error);
    metrics.angle_tracking.combined_mean = mean([phi_error, theta_error]);
    
    %% 3. Sensor Failure Rate
    % From guide: "Percentage of time steps with sensor_fail > 0"
    if isfield(sim_data, 'sensor_fail')
        total_steps = length(sim_data.sensor_fail);
        failed_steps = sum(sim_data.sensor_fail > 0);
        metrics.sensor_failure_rate = (failed_steps / total_steps) * 100;  % Percentage
    else
        metrics.sensor_failure_rate = 0;
        warning('sensor_fail field not found in sim_data');
    end
    
    %% 4. State Transition Frequency
    % From guide: "Number of state machine transitions per minute"
    if isfield(sim_data, 'state') && isfield(sim_data, 't')
        % Count state changes
        state_changes = sum(diff(sim_data.state) ~= 0);
        
        % Mission duration in minutes
        duration_min = (sim_data.t(end) - sim_data.t(1)) / 60;
        
        metrics.state_transitions.total = state_changes;
        metrics.state_transitions.per_minute = state_changes / duration_min;
        metrics.state_transitions.duration_min = duration_min;
    else
        metrics.state_transitions.total = 0;
        metrics.state_transitions.per_minute = 0;
        warning('state or t field not found in sim_data');
    end
    
    %% 5. Control Effort
    % From guide: "Time-averaged magnitude of PID outputs"
    if isfield(sim_data, 'u')
        % Compute L2 norm of control vector at each time step
        control_magnitude = sqrt(sum(sim_data.u.^2, 1));
        metrics.control_effort.mean = mean(control_magnitude);
        metrics.control_effort.std = std(control_magnitude);
        metrics.control_effort.max = max(control_magnitude);
        
        % Per-axis effort
        metrics.control_effort.surge = rms(sim_data.u(1,:));
        metrics.control_effort.sway = rms(sim_data.u(2,:));
        metrics.control_effort.heave = rms(sim_data.u(3,:));
        metrics.control_effort.roll = rms(sim_data.u(4,:));
        metrics.control_effort.pitch = rms(sim_data.u(5,:));
        metrics.control_effort.yaw = rms(sim_data.u(6,:));
    else
        metrics.control_effort = struct();
        warning('u field not found in sim_data');
    end
    
    %% 6. Maximum Innovation Magnitude
    % From guide: "Maximum innovation magnitude ||ν||_∞"
    if isfield(sim_data, 'ni')
        innovation_norms = sqrt(sum(sim_data.ni.^2, 1));
        metrics.innovation_max = max(innovation_norms);
        metrics.innovation_mean = mean(innovation_norms);
        metrics.innovation_std = std(innovation_norms);
    else
        metrics.innovation_max = NaN;
        warning('ni (innovation) field not found in sim_data');
    end
    
    %% 7. Normal Vector Parallelism
    % From guide Eq 2.6: "Angle between EKF-estimated normal n_est and SBES-measured normal n_mes"
    % ∠(n_est, n_mes) = arccos(|n_est^T * n_mes|)
    if isfield(sim_data, 'n_est') && isfield(sim_data, 'n_mes')
        N = size(sim_data.n_est, 2);
        parallelism_angles = zeros(1, N);
        
        for i = 1:N
            n_est = sim_data.n_est(:,i);
            n_mes = sim_data.n_mes(:,i);
            
            % Normalize vectors
            n_est = n_est / norm(n_est);
            n_mes = n_mes / norm(n_mes);
            
            % Compute angle (in degrees)
            cos_angle = abs(dot(n_est, n_mes));
            cos_angle = min(1.0, max(-1.0, cos_angle));  % Clamp to valid range
            parallelism_angles(i) = acos(cos_angle) * 180/pi;
        end
        
        metrics.normal_parallelism.mean = mean(parallelism_angles);
        metrics.normal_parallelism.std = std(parallelism_angles);
        metrics.normal_parallelism.max = max(parallelism_angles);
        metrics.normal_parallelism.below_5deg = sum(parallelism_angles < 5) / N * 100;  % Percentage
    else
        metrics.normal_parallelism = struct();
        warning('n_est or n_mes field not found in sim_data');
    end
    
    %% 8. Robot-Terrain Alignment
    % From guide Eq 2.7: "Angle between robot Z-axis and estimated normal"
    % ∠(z_robot, n_est) = arccos(|z_robot^T * n_est|)
    if isfield(sim_data, 'wRr') && isfield(sim_data, 'n_est')
        N = size(sim_data.n_est, 2);
        alignment_angles = zeros(1, N);
        
        for i = 1:N
            R = sim_data.wRr(:,:,i);
            z_robot = R(:,3);  % Third column is Z-axis in world frame
            n_est = sim_data.n_est(:,i);
            
            % Normalize
            z_robot = z_robot / norm(z_robot);
            n_est = n_est / norm(n_est);
            
            % Compute angle (in degrees)
            cos_angle = abs(dot(z_robot, n_est));
            cos_angle = min(1.0, max(-1.0, cos_angle));
            alignment_angles(i) = acos(cos_angle) * 180/pi;
        end
        
        metrics.robot_alignment.mean = mean(alignment_angles);
        metrics.robot_alignment.std = std(alignment_angles);
        metrics.robot_alignment.max = max(alignment_angles);
    else
        metrics.robot_alignment = struct();
        warning('wRr or n_est field not found in sim_data');
    end
    
    %% 9. Additional Statistics (MAE, RMSE)
    % Standard error metrics
    metrics.altitude_errors.mae = mean(abs(h_error));
    metrics.altitude_errors.rmse = rms(h_error);
    metrics.altitude_errors.max = max(abs(h_error));
    metrics.altitude_errors.std = std(h_error);
    
    % Alpha and Beta errors
    if isfield(sim_data, 'x_true')
        alpha_error = sim_data.x_true(2,:) - sim_data.x_est(2,:);
        beta_error = sim_data.x_true(3,:) - sim_data.x_est(3,:);
        
        metrics.alpha_errors.mae = mean(abs(alpha_error));
        metrics.alpha_errors.rmse = rms(alpha_error);
        metrics.alpha_errors.max = max(abs(alpha_error));
        
        metrics.beta_errors.mae = mean(abs(beta_error));
        metrics.beta_errors.rmse = rms(beta_error);
        metrics.beta_errors.max = max(abs(beta_error));
    end
    
    %% 10. State Machine Statistics
    % Percentage time in each state
    if isfield(sim_data, 'state')
        unique_states = unique(sim_data.state);
        for s = unique_states
            state_time = sum(sim_data.state == s) / length(sim_data.state) * 100;
            metrics.state_occupancy.(sprintf('state_%d', s)) = state_time;
        end
    end
    
    %% 11. EKF Position Filter Performance (NEW)
    % RMSE between estimated and ground truth states
    if isfield(sim_data, 'x_loc') && isfield(sim_data, 'eta_gt') && isfield(sim_data, 'nu_gt')
        % Position errors
        pos_err = sim_data.x_loc(1:3,:) - sim_data.eta_gt(1:3,:);
        metrics.ekf_position.pos_rmse_x = rms(pos_err(1,:));
        metrics.ekf_position.pos_rmse_y = rms(pos_err(2,:));
        metrics.ekf_position.pos_rmse_z = rms(pos_err(3,:));
        metrics.ekf_position.pos_rmse_total = norm([metrics.ekf_position.pos_rmse_x, ...
                                                    metrics.ekf_position.pos_rmse_y, ...
                                                    metrics.ekf_position.pos_rmse_z]);
        
        % Orientation errors (in degrees)
        ang_err = sim_data.x_loc(4:6,:) - sim_data.eta_gt(4:6,:);
        metrics.ekf_position.ang_rmse_roll = rad2deg(rms(ang_err(1,:)));
        metrics.ekf_position.ang_rmse_pitch = rad2deg(rms(ang_err(2,:)));
        metrics.ekf_position.ang_rmse_yaw = rad2deg(rms(ang_err(3,:)));
        
        % Linear velocity errors
        vel_err = sim_data.x_loc(7:9,:) - sim_data.nu_gt(1:3,:);
        metrics.ekf_position.vel_rmse_surge = rms(vel_err(1,:));
        metrics.ekf_position.vel_rmse_sway = rms(vel_err(2,:));
        metrics.ekf_position.vel_rmse_heave = rms(vel_err(3,:));
        
        % Angular velocity errors (in degrees/s)
        rate_err = sim_data.x_loc(10:12,:) - sim_data.nu_gt(4:6,:);
        metrics.ekf_position.rate_rmse_p = rad2deg(rms(rate_err(1,:)));
        metrics.ekf_position.rate_rmse_q = rad2deg(rms(rate_err(2,:)));
        metrics.ekf_position.rate_rmse_r = rad2deg(rms(rate_err(3,:)));
        
        % Max errors
        metrics.ekf_position.pos_max_err = max(abs(pos_err(:)));
        metrics.ekf_position.ang_max_err = rad2deg(max(abs(ang_err(:))));
        metrics.ekf_position.vel_max_err = max(abs(vel_err(:)));
        metrics.ekf_position.rate_max_err = rad2deg(max(abs(rate_err(:))));
    else
        metrics.ekf_position = struct();
        warning('x_loc, eta_gt or nu_gt field not found in sim_data');
    end

end
