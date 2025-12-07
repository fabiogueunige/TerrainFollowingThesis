function controller_step_response_analysis()
    % CONTROLLER STEP RESPONSE ANALYSIS
    % This script analyzes the transient response characteristics of the
    % terrain following controller using step inputs.
    %
    % Key metrics computed:
    % - Rise Time (10% to 90%)
    % - Settling Time (2% criterion)
    % - Overshoot Percentage
    % - Steady-State Error
    
    clc; clear; close all;
    addpath('../for_controller');
    addpath('../rotations');
    addpath('../sensors');
    addpath('../model');
    addpath('../ekf_position');

    fprintf('=================================================================\n');
    fprintf('       CONTROLLER STEP RESPONSE ANALYSIS\n');
    fprintf('=================================================================\n\n');

    %% Configuration
    Ts = 0.001; 
    Tf = 30;  % Shorter simulation for step response
    time = 0:Ts:Tf; 
    N = length(time);
    
    % Step parameters
    step_time = 5;  % Time when step is applied [s]
    step_idx = find(time >= step_time, 1);
    
    %% Test 1: Altitude Step Response
    fprintf('--- TEST 1: ALTITUDE STEP RESPONSE (h: 3m -> 5m) ---\n');
    [data_alt, metrics_alt] = run_altitude_step(Ts, Tf, step_time, 3, 5);
    print_step_metrics('Altitude', metrics_alt, 'm');
    
    %% Test 2: Roll Step Response
    fprintf('\n--- TEST 2: ROLL STEP RESPONSE (0 -> 30 deg) ---\n');
    [data_roll, metrics_roll] = run_angle_step(Ts, Tf, step_time, 0, deg2rad(30), 'roll');
    print_step_metrics('Roll', metrics_roll, 'deg');
    
    %% Test 3: Pitch Step Response
    fprintf('\n--- TEST 3: PITCH STEP RESPONSE (0 -> 30 deg) ---\n');
    [data_pitch, metrics_pitch] = run_angle_step(Ts, Tf, step_time, 0, deg2rad(30), 'pitch');
    print_step_metrics('Pitch', metrics_pitch, 'deg');
    
    %% Test 4: Surge Velocity Step Response
    fprintf('\n--- TEST 4: SURGE VELOCITY STEP RESPONSE (0.2 -> 0.5 m/s) ---\n');
    [data_surge, metrics_surge] = run_velocity_step(Ts, Tf, step_time, 0.2, 0.5);
    print_step_metrics('Surge', metrics_surge, 'm/s');
    
    %% Generate Summary Table
    fprintf('\n=================================================================\n');
    fprintf('                    STEP RESPONSE SUMMARY\n');
    fprintf('=================================================================\n');
    fprintf('%-12s | %-10s | %-12s | %-12s | %-12s\n', 'Variable', 'Rise Time', 'Settle Time', 'Overshoot', 'SS Error');
    fprintf('-----------------------------------------------------------------\n');
    fprintf('%-12s | %8.3f s | %10.3f s | %10.2f%% | %10.4f m\n', ...
        'Altitude', metrics_alt.rise_time, metrics_alt.settling_time, metrics_alt.overshoot_pct, metrics_alt.ss_error);
    fprintf('%-12s | %8.3f s | %10.3f s | %10.2f%% | %10.4f deg\n', ...
        'Roll', metrics_roll.rise_time, metrics_roll.settling_time, metrics_roll.overshoot_pct, rad2deg(metrics_roll.ss_error));
    fprintf('%-12s | %8.3f s | %10.3f s | %10.2f%% | %10.4f deg\n', ...
        'Pitch', metrics_pitch.rise_time, metrics_pitch.settling_time, metrics_pitch.overshoot_pct, rad2deg(metrics_pitch.ss_error));
    fprintf('%-12s | %8.3f s | %10.3f s | %10.2f%% | %10.4f m/s\n', ...
        'Surge', metrics_surge.rise_time, metrics_surge.settling_time, metrics_surge.overshoot_pct, metrics_surge.ss_error);
    fprintf('=================================================================\n');
    
    %% Plot Results
    plot_step_responses(data_alt, data_roll, data_pitch, data_surge, step_time);
    
    %% Performance Classification
    fprintf('\n--- PERFORMANCE CLASSIFICATION ---\n');
    classify_response('Altitude', metrics_alt);
    classify_response('Roll', metrics_roll);
    classify_response('Pitch', metrics_pitch);
    classify_response('Surge', metrics_surge);
    
    fprintf('\nAnalysis Complete.\n');
end

function print_step_metrics(name, m, unit)
    fprintf('  Rise Time (10%%-90%%): %.3f s\n', m.rise_time);
    fprintf('  Settling Time (2%%):  %.3f s\n', m.settling_time);
    fprintf('  Overshoot:           %.2f %%\n', m.overshoot_pct);
    fprintf('  Steady-State Error:  %.4f %s\n', m.ss_error, unit);
end

function classify_response(name, m)
    % Classification based on typical control system standards
    if m.overshoot_pct > 25
        overshoot_class = 'UNDERDAMPED (high overshoot)';
    elseif m.overshoot_pct > 5
        overshoot_class = 'SLIGHTLY UNDERDAMPED';
    elseif m.overshoot_pct < 1
        overshoot_class = 'OVERDAMPED (no overshoot)';
    else
        overshoot_class = 'CRITICALLY DAMPED (optimal)';
    end
    
    if m.settling_time < 2
        speed_class = 'FAST';
    elseif m.settling_time < 5
        speed_class = 'MODERATE';
    else
        speed_class = 'SLOW';
    end
    
    fprintf('%s: %s response, %s settling\n', name, overshoot_class, speed_class);
end

function [data, metrics] = run_altitude_step(Ts, Tf, step_time, h_init, h_final)
    time = 0:Ts:Tf;
    N = length(time);
    step_idx = find(time >= step_time, 1);
    
    i_dim = 6; d_dim = 3; ekf_dim = 15;
    
    % Initialize
    eta = zeros(i_dim,N); 
    nu = zeros(i_dim, N);
    u = zeros(i_dim, N);
    nu_dot = zeros(i_dim,N);
    h = zeros(N,1);
    
    P_loc = zeros(ekf_dim,ekf_dim,N);
    [Q_loc, P_loc(:,:,1)] = stateLoc_init(ekf_dim);
    wRr = zeros(d_dim, d_dim, N);
    wRr(:,:,1) = eye(3);
    x_loc = zeros(ekf_dim, N);
    
    pos = zeros(i_dim,N);
    rot = zeros(d_dim,d_dim,N); rot(:,:,1) = eye(3);
    
    p_seafloor_NED = [-3; 0; 5];
    n0 = [0; 0; 1];
    w_n = n0;
    
    % Compute initial altitude offset to start at h_init
    % h = (w_n'*(pos - p_seafloor)) / norm(w_n)
    % We want h(1) = h_init, so pos(3,1) = p_seafloor(3) - h_init * norm(w_n) / w_n(3)
    % For flat terrain (w_n = [0;0;1]): pos(3,1) = p_seafloor(3) - h_init
    pos(3,1) = p_seafloor_NED(3) - h_init;
    eta(:,1) = pos(:,1);
    h(1) = h_init;
    
    % Step target
    h_star = zeros(1, N);
    h_star(1:step_idx-1) = h_init;
    h_star(step_idx:end) = h_final;
    
    speed0 = [0.2; 0; 0; 0; 0; 0];
    u_star = 0.3; v_star = 0;
    
    err = zeros(i_dim, N);
    pid = zeros(i_dim, N);
    i_err = zeros(i_dim, N);
    term_sum = zeros(i_dim, N);
    
    % Differentiated saturation limits
    max_pid_vec = [4.0; 4.0; 2.5; 2.5; 2.5; 4.0]; % [surge,sway,heave,roll,pitch,yaw]
    
    [Kp, Ki, Kd] = gainComputation(speed0, i_dim);
    
    for k = 2:N
        h_err = (h_star(k) - h(k-1)); 
        r_n = wRr(:,:,k-1)' * w_n;
        r_n = r_n / norm(r_n);
        h_contribution = h_err * r_n;
        
        err(1,k) = u_star - nu(1,k-1) + 0.5 * h_contribution(1);
        err(2,k) = v_star - nu(2,k-1) + 0.5 * h_contribution(2);
        err(3,k) = h_contribution(3);
        err(4,k) = 0 - eta(4,k-1);
        err(5,k) = 0 - eta(5,k-1);
        err(6,k) = 0 - eta(6,k-1);
        
        for l = 1:i_dim
            if l == 1 || l == 2
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu_dot(l,k-1);
                d_err = 0;
            else
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu(l,k-1);
                d_err = Kd(l) * nu_dot(l,k-1);
            end
            term_sum(l,k) = i_err(l,k) - p_err - d_err;
            pid(l,k) = integrator(pid(l,k-1), term_sum(l,k), term_sum(l,k-1), Ts);
            pid(l,k) = max(min(pid(l,k), max_pid_vec(l)), -max_pid_vec(l));
        end
        
        [nu_dot(:,k), u(:,k)] = dynamic_model(pid(:,k), pos(4:6,k-1), u(:,k-1), Ts, i_dim, nu_dot(:,k-1));
        
        T = transformationT(pos(4:6,k-1));
        pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*u(1:3,k)*Ts;
        pos(4:6,k) = pos(4:6,k-1) + T * u(4:6,k) *Ts;
        rot(:,:,k) = rotz(pos(6,k)) * roty(pos(5,k)) * rotx(pos(4,k));
        eta_clean = pos(:,k);
    
        [x_loc(:,k), P_loc(:,:,k), wRr(:,:,k)] = ekf_position(x_loc(:,k-1), pid(:,k), wRr(:,:,k-1), u(:,k), eta_clean, P_loc(:,:,k-1), Q_loc, Ts);
        eta(:,k) = x_loc(1:6,k);
        nu(:,k) = x_loc(7:12,k);
        
        h(k) = abs((w_n'*(pos(1:3,k) - p_seafloor_NED))/norm(w_n));
    end
    
    data.time = time;
    data.response = h';
    data.target = h_star;
    data.step_time = step_time;
    
    % Compute metrics
    metrics = compute_step_metrics(time, h', h_star, step_idx, h_init, h_final);
end

function [data, metrics] = run_angle_step(Ts, Tf, step_time, angle_init, angle_final, angle_type)
    time = 0:Ts:Tf;
    N = length(time);
    step_idx = find(time >= step_time, 1);
    
    i_dim = 6; d_dim = 3; ekf_dim = 15;
    
    eta = zeros(i_dim,N); 
    nu = zeros(i_dim, N);
    u = zeros(i_dim, N);
    nu_dot = zeros(i_dim,N);
    h = zeros(N,1);
    
    P_loc = zeros(ekf_dim,ekf_dim,N);
    [Q_loc, P_loc(:,:,1)] = stateLoc_init(ekf_dim);
    wRr = zeros(d_dim, d_dim, N);
    wRr(:,:,1) = eye(3);
    x_loc = zeros(ekf_dim, N);
    
    pos = zeros(i_dim,N);
    rot = zeros(d_dim,d_dim,N); rot(:,:,1) = eye(3);
    
    p_seafloor_NED = [-3; 0; 5];
    
    % Target angles (step)
    alpha = zeros(1,N); % roll target
    beta = zeros(1,N);  % pitch target
    
    if strcmp(angle_type, 'roll')
        alpha(1:step_idx-1) = angle_init;
        alpha(step_idx:end) = angle_final;
        beta(:) = 0;
    else % pitch
        alpha(:) = 0;
        beta(1:step_idx-1) = angle_init;
        beta(step_idx:end) = angle_final;
    end
    
    n0 = [0; 0; 1];
    wRs = rotx(pi) * rotz(0) * roty(beta(1)) * rotx(alpha(1));
    w_n = wRs * n0;
    h(1) = 3; % Start at target altitude
    
    speed0 = [0.2; 0; 0; 0; 0; 0];
    u_star = 0.3; v_star = 0; h_star = 3;
    
    err = zeros(i_dim, N);
    pid = zeros(i_dim, N);
    i_err = zeros(i_dim, N);
    term_sum = zeros(i_dim, N);
    
    % Differentiated saturation limits
    max_pid_vec = [4.0; 4.0; 2.5; 2.5; 2.5; 4.0]; % [surge,sway,heave,roll,pitch,yaw]
    
    [Kp, Ki, Kd] = gainComputation(speed0, i_dim);
    
    for k = 2:N
        wRs = rotx(pi) * rotz(0) * roty(beta(k)) * rotx(alpha(k));
        w_n = wRs * n0;
        
        h_err = (h_star - h(k-1)); 
        r_n = wRr(:,:,k-1)' * w_n;
        r_n = r_n / norm(r_n);
        h_contribution = h_err * r_n;
        
        err(1,k) = u_star - nu(1,k-1) + 0.5 * h_contribution(1);
        err(2,k) = v_star - nu(2,k-1) + 0.5 * h_contribution(2);
        err(3,k) = h_contribution(3);
        err(4,k) = alpha(k) - eta(4,k-1);
        err(5,k) = beta(k) - eta(5,k-1);
        err(6,k) = 0 - eta(6,k-1);
        
        for l = 1:i_dim
            if l == 1 || l == 2
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu_dot(l,k-1);
                d_err = 0;
            else
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu(l,k-1);
                d_err = Kd(l) * nu_dot(l,k-1);
            end
            term_sum(l,k) = i_err(l,k) - p_err - d_err;
            pid(l,k) = integrator(pid(l,k-1), term_sum(l,k), term_sum(l,k-1), Ts);
            pid(l,k) = max(min(pid(l,k), max_pid_vec(l)), -max_pid_vec(l));
        end
        
        [nu_dot(:,k), u(:,k)] = dynamic_model(pid(:,k), pos(4:6,k-1), u(:,k-1), Ts, i_dim, nu_dot(:,k-1));
        
        T = transformationT(pos(4:6,k-1));
        pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*u(1:3,k)*Ts;
        pos(4:6,k) = pos(4:6,k-1) + T * u(4:6,k) *Ts;
        rot(:,:,k) = rotz(pos(6,k)) * roty(pos(5,k)) * rotx(pos(4,k));
        eta_clean = pos(:,k);
    
        [x_loc(:,k), P_loc(:,:,k), wRr(:,:,k)] = ekf_position(x_loc(:,k-1), pid(:,k), wRr(:,:,k-1), u(:,k), eta_clean, P_loc(:,:,k-1), Q_loc, Ts);
        eta(:,k) = x_loc(1:6,k);
        nu(:,k) = x_loc(7:12,k);
        
        h(k) = abs((w_n'*(pos(1:3,k) - p_seafloor_NED))/norm(w_n));
    end
    
    if strcmp(angle_type, 'roll')
        response = pos(4,:);
        target = alpha;
    else
        response = pos(5,:);
        target = beta;
    end
    
    data.time = time;
    data.response = rad2deg(response);
    data.target = rad2deg(target);
    data.step_time = step_time;
    
    metrics = compute_step_metrics(time, response, target, step_idx, angle_init, angle_final);
end

function [data, metrics] = run_velocity_step(Ts, Tf, step_time, u_init, u_final)
    time = 0:Ts:Tf;
    N = length(time);
    step_idx = find(time >= step_time, 1);
    
    i_dim = 6; d_dim = 3; ekf_dim = 15;
    
    eta = zeros(i_dim,N); 
    nu = zeros(i_dim, N);
    u = zeros(i_dim, N);
    nu_dot = zeros(i_dim,N);
    h = zeros(N,1);
    
    P_loc = zeros(ekf_dim,ekf_dim,N);
    [Q_loc, P_loc(:,:,1)] = stateLoc_init(ekf_dim);
    wRr = zeros(d_dim, d_dim, N);
    wRr(:,:,1) = eye(3);
    x_loc = zeros(ekf_dim, N);
    
    pos = zeros(i_dim,N);
    rot = zeros(d_dim,d_dim,N); rot(:,:,1) = eye(3);
    
    p_seafloor_NED = [-3; 0; 5];
    n0 = [0; 0; 1];
    w_n = n0;
    h(1) = 3;
    
    % Step target velocity
    u_star = zeros(1, N);
    u_star(1:step_idx-1) = u_init;
    u_star(step_idx:end) = u_final;
    
    speed0 = [0.2; 0; 0; 0; 0; 0];
    v_star = 0; h_star = 3;
    
    err = zeros(i_dim, N);
    pid = zeros(i_dim, N);
    i_err = zeros(i_dim, N);
    term_sum = zeros(i_dim, N);
    
    % Differentiated saturation limits
    max_pid_vec = [4.0; 4.0; 2.5; 2.5; 2.5; 4.0]; % [surge,sway,heave,roll,pitch,yaw]
    
    [Kp, Ki, Kd] = gainComputation(speed0, i_dim);
    
    for k = 2:N
        h_err = (h_star - h(k-1)); 
        r_n = wRr(:,:,k-1)' * w_n;
        r_n = r_n / norm(r_n);
        h_contribution = h_err * r_n;
        
        err(1,k) = u_star(k) - nu(1,k-1) + 0.5 * h_contribution(1);
        err(2,k) = v_star - nu(2,k-1) + 0.5 * h_contribution(2);
        err(3,k) = h_contribution(3);
        err(4,k) = 0 - eta(4,k-1);
        err(5,k) = 0 - eta(5,k-1);
        err(6,k) = 0 - eta(6,k-1);
        
        for l = 1:i_dim
            if l == 1 || l == 2
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu_dot(l,k-1);
                d_err = 0;
            else
                i_err(l,k) = Ki(l) * err(l,k);
                p_err = Kp(l) * nu(l,k-1);
                d_err = Kd(l) * nu_dot(l,k-1);
            end
            term_sum(l,k) = i_err(l,k) - p_err - d_err;
            pid(l,k) = integrator(pid(l,k-1), term_sum(l,k), term_sum(l,k-1), Ts);
            pid(l,k) = max(min(pid(l,k), max_pid_vec(l)), -max_pid_vec(l));
        end
        
        [nu_dot(:,k), u(:,k)] = dynamic_model(pid(:,k), pos(4:6,k-1), u(:,k-1), Ts, i_dim, nu_dot(:,k-1));
        
        T = transformationT(pos(4:6,k-1));
        pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*u(1:3,k)*Ts;
        pos(4:6,k) = pos(4:6,k-1) + T * u(4:6,k) *Ts;
        rot(:,:,k) = rotz(pos(6,k)) * roty(pos(5,k)) * rotx(pos(4,k));
        eta_clean = pos(:,k);
    
        [x_loc(:,k), P_loc(:,:,k), wRr(:,:,k)] = ekf_position(x_loc(:,k-1), pid(:,k), wRr(:,:,k-1), u(:,k), eta_clean, P_loc(:,:,k-1), Q_loc, Ts);
        eta(:,k) = x_loc(1:6,k);
        nu(:,k) = x_loc(7:12,k);
        
        h(k) = abs((w_n'*(pos(1:3,k) - p_seafloor_NED))/norm(w_n));
    end
    
    data.time = time;
    data.response = nu(1,:);
    data.target = u_star;
    data.step_time = step_time;
    
    metrics = compute_step_metrics(time, nu(1,:), u_star, step_idx, u_init, u_final);
end

function metrics = compute_step_metrics(time, response, target, step_idx, init_val, final_val)
    Ts = time(2) - time(1);
    step_magnitude = final_val - init_val;
    
    % Use response after step
    resp_after_step = response(step_idx:end);
    time_after_step = time(step_idx:end) - time(step_idx);
    
    % Normalize response (0 to 1 scale)
    resp_norm = (resp_after_step - init_val) / step_magnitude;
    
    % Rise Time (10% to 90%)
    idx_10 = find(resp_norm >= 0.1, 1);
    idx_90 = find(resp_norm >= 0.9, 1);
    if isempty(idx_10), idx_10 = 1; end
    if isempty(idx_90), idx_90 = length(resp_norm); end
    metrics.rise_time = time_after_step(idx_90) - time_after_step(idx_10);
    
    % Settling Time (2% band)
    settling_band = 0.02;
    settled = abs(resp_norm - 1) <= settling_band;
    % Find last time it leaves the band
    idx_settle = find(~settled, 1, 'last');
    if isempty(idx_settle)
        metrics.settling_time = 0;
    else
        metrics.settling_time = time_after_step(min(idx_settle+1, length(time_after_step)));
    end
    
    % Overshoot
    if step_magnitude > 0
        peak_val = max(resp_after_step);
    else
        peak_val = min(resp_after_step);
    end
    metrics.overshoot_pct = 100 * abs(peak_val - final_val) / abs(step_magnitude);
    
    % Steady-State Error (last 10% of simulation)
    ss_region = resp_after_step(round(0.9*length(resp_after_step)):end);
    target_ss = final_val;
    metrics.ss_error = abs(mean(ss_region) - target_ss);
end

function plot_step_responses(data_alt, data_roll, data_pitch, data_surge, step_time)
    lw = 1.5; fs = 12; fn = 'Times New Roman';
    
    % Plot 1: Altitude Step Response
    figure('Name', 'Step Response: Altitude');
    plot(data_alt.time, data_alt.target, 'r--', 'LineWidth', lw); hold on;
    plot(data_alt.time, data_alt.response, 'b-', 'LineWidth', lw);
    xline(step_time, 'k:', 'Step', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Altitude [m]', 'FontSize', fs, 'FontName', fn);
    title('Altitude Step Response (3m \rightarrow 5m)', 'FontSize', fs, 'FontName', fn);
    legend({'Target', 'Response'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best');
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 2: Roll Step Response
    figure('Name', 'Step Response: Roll');
    plot(data_roll.time, data_roll.target, 'r--', 'LineWidth', lw); hold on;
    plot(data_roll.time, data_roll.response, 'b-', 'LineWidth', lw);
    xline(step_time, 'k:', 'Step', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Roll [deg]', 'FontSize', fs, 'FontName', fn);
    title('Roll Step Response (0 \rightarrow 30 deg)', 'FontSize', fs, 'FontName', fn);
    legend({'Target', 'Response'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best');
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 3: Pitch Step Response
    figure('Name', 'Step Response: Pitch');
    plot(data_pitch.time, data_pitch.target, 'r--', 'LineWidth', lw); hold on;
    plot(data_pitch.time, data_pitch.response, 'b-', 'LineWidth', lw);
    xline(step_time, 'k:', 'Step', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Pitch [deg]', 'FontSize', fs, 'FontName', fn);
    title('Pitch Step Response (0 \rightarrow 30 deg)', 'FontSize', fs, 'FontName', fn);
    legend({'Target', 'Response'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best');
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Plot 4: Surge Velocity Step Response
    figure('Name', 'Step Response: Surge Velocity');
    plot(data_surge.time, data_surge.target, 'r--', 'LineWidth', lw); hold on;
    plot(data_surge.time, data_surge.response, 'b-', 'LineWidth', lw);
    xline(step_time, 'k:', 'Step', 'LineWidth', 1);
    xlabel('Time [s]', 'FontSize', fs, 'FontName', fn);
    ylabel('Surge Velocity [m/s]', 'FontSize', fs, 'FontName', fn);
    title('Surge Velocity Step Response (0.2 \rightarrow 0.5 m/s)', 'FontSize', fs, 'FontName', fn);
    legend({'Target', 'Response'}, 'FontSize', fs, 'FontName', fn, 'Location', 'best');
    grid on;
    set(gca, 'FontSize', fs, 'FontName', fn);
    
    % Combined Plot
    figure('Name', 'Step Response Summary');
    
    subplot(2,2,1);
    plot(data_alt.time, data_alt.target, 'r--', 'LineWidth', 1); hold on;
    plot(data_alt.time, data_alt.response, 'b-', 'LineWidth', 1);
    xline(step_time, 'k:', 'LineWidth', 0.5);
    xlabel('Time [s]', 'FontSize', fs-2, 'FontName', fn);
    ylabel('Altitude [m]', 'FontSize', fs-2, 'FontName', fn);
    title('Altitude', 'FontSize', fs-2, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs-2, 'FontName', fn);
    
    subplot(2,2,2);
    plot(data_roll.time, data_roll.target, 'r--', 'LineWidth', 1); hold on;
    plot(data_roll.time, data_roll.response, 'b-', 'LineWidth', 1);
    xline(step_time, 'k:', 'LineWidth', 0.5);
    xlabel('Time [s]', 'FontSize', fs-2, 'FontName', fn);
    ylabel('Roll [deg]', 'FontSize', fs-2, 'FontName', fn);
    title('Roll', 'FontSize', fs-2, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs-2, 'FontName', fn);
    
    subplot(2,2,3);
    plot(data_pitch.time, data_pitch.target, 'r--', 'LineWidth', 1); hold on;
    plot(data_pitch.time, data_pitch.response, 'b-', 'LineWidth', 1);
    xline(step_time, 'k:', 'LineWidth', 0.5);
    xlabel('Time [s]', 'FontSize', fs-2, 'FontName', fn);
    ylabel('Pitch [deg]', 'FontSize', fs-2, 'FontName', fn);
    title('Pitch', 'FontSize', fs-2, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs-2, 'FontName', fn);
    
    subplot(2,2,4);
    plot(data_surge.time, data_surge.target, 'r--', 'LineWidth', 1); hold on;
    plot(data_surge.time, data_surge.response, 'b-', 'LineWidth', 1);
    xline(step_time, 'k:', 'LineWidth', 0.5);
    xlabel('Time [s]', 'FontSize', fs-2, 'FontName', fn);
    ylabel('Surge [m/s]', 'FontSize', fs-2, 'FontName', fn);
    title('Surge Velocity', 'FontSize', fs-2, 'FontName', fn);
    grid on;
    set(gca, 'FontSize', fs-2, 'FontName', fn);
end
