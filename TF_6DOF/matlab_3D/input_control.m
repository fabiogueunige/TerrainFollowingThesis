function [tau, p_err, int_err] = input_control(x, Ts, prev_err, int_err, v_surge)
    %% Definitions
    IND_H = 1;                  I_IND_U = 1;
    ALPHA = 2;                  I_IND_W = 2;
    BETA = 3;
    PHI = 4;      M_PHI = 5;
    THETA = 5;    M_THETA = 6;
    IND_P = 6;    M_IND_P = 7;  I_IND_P = 3;
    IND_Q = 7;    M_IND_Q = 8;  I_IND_Q = 4;  

    C_U = 1;    C_H = 2;    C_ROLL = 3;     C_PITCH = 4;
    fprintf('       Input Control\n');
    %% SYSTEM PARAMETERS
    u_star = 0.1;         % [m/s] Constant surge velocity
    tau_u_max = 10;       % [m/s] Robot surge velocity saturation
    tau_w_max = 10;       % [m/s] Vertical velocity saturation
    tau_p_max = 5;        % Roll velocity saturation
    tau_q_max = 5;        % Pitch velocity saturation
    h_star = 7;           % Reference altitude
    
    %% Redefinition
    % surge = 1; heave = 2; roll = 3; pitch = 4 %
    prev_err_u = prev_err(C_U);
    prev_err_h = prev_err(C_H);
    prev_err_r = prev_err(C_ROLL);
    prev_err_p = prev_err(C_PITCH);
    int_err_u = int_err(C_U);
    int_err_h = int_err(C_H);
    int_err_r = int_err(C_ROLL);
    int_err_p = int_err(C_PITCH);

    %% PID PARAMETERS % ---- TO IMPROVE ---- %
    Kp = 0.8;          % Proportional gain
    Ki = 0.1;          % Integral gain
    Kd = 0.05;         % Derivative gain

    integral_max = 1.0; % Maximum allowed integral error
    
    %% ERROR CALCULATION FOR SURGE
    tau_u = u_star;
    err_u = 0;
    int_err_u = 0;
    % err_u = u_star - v_surge;
    % int_err_u = int_err_u + err_u * Ts; % Accumulate integral error
    % der_err_u = (err_u - prev_err_u) / Ts; % Calculate derivative error
    % 
    % % Anti-windup for the integrator
    % int_err_u = max(min(int_err_u, integral_max), -integral_max); % Clamp integral error
    % 
    % % PID
    % tau_u = (Kp * err_u + Ki * int_err_u + Kd * der_err_u); % Calculate PID output
    % % Having the PID i can consider it as tau
    % 
    % % Final saturation
    % tau_u = max(min(tau_u, tau_u_max), -tau_u_max); % Clamp pitch velocity to limits

    u_ref = tau_u;
    % Memory update
    prev_err_u = err_u; % Store current error for next iteration's derivative calculation

    %% ERROR CALCULATION FOR H
    err_h = 0; % tmpp
    int_err_h = 0;
    tau_w = 0;
    
    % err_h = -(h_star - x(IND_H)); % Negative for robot frame velocity (error calculation)
    % int_err_h = int_err_h + err_h * Ts; % Accumulate integral error
    % der_err_h = (err_h - prev_err_h) / Ts; % Calculate derivative error
    % 
    % % --- CAMBIAREEEEE ----- %
    % 
    % % Anti-windup for the integrator
    % int_err_h = max(min(int_err_h, integral_max), -integral_max); % Clamp integral error
    % 
    % % PID CONTROL 
    % cos_b = cos(x(2)); % Cosine of sonar angle (works for this case)
    % if abs(cos_b) < 1e-3
    %     cos_b = sign(cos_b) * 1e-3;  % Numerical protection (avoid division by zero)
    % end
    % 
    % % PID on altitude frame
    % pid_term = (Kp * err_h + Ki * int_err_h + Kd * der_err_h) / cos_b; % Calculate PID output
    % 
    % % Using the new velocity expected from the controller
    % tau_w = (u_ref*(sin(x(3))-cos(x(3))*tan(x(2))) + pid_term) / (cos(x(3))+sin(x(3))*tan(x(2)));
    % 
    % % Final saturation
    % tau_w = max(min(tau_w, tau_w_max), -tau_w_max); % Clamp vertical velocity to limits
    
    % Memory update
    prev_err_h = err_h; % Store current error for next iteration's derivative calculation
    
    %% ERROR CALCULATION FOR PHI -> ROLL
    err_r = 0; % tmpp
    int_err_r = 0;
    tau_p = 0;
    
    % ----------- TO IMPROVE -------------------
    % err_r = x(ALPHA) - x(PHI); % beta - theta
    % int_err_r = int_err_r + err_r * Ts; % Accumulate integral error
    % der_err_ = (err_r - prev_err_p) / Ts; % Calculate derivative error
    % 
    % % Anti-windup for the integrator
    % int_err_r = max(min(int_err_r, integral_max), -integral_max); % Clamp integral error
    % 
    % % PID
    % tau_p = (Kp * err_r + Ki * int_err_r + Kd * der_err_); % Calculate PID output
    % 
    % % Final saturation q_ref = tau
    % tau_p = max(min(tau_p, tau_p_max), -tau_p_max); % Clamp pitch velocity to limits

    % Memory update
    prev_err_r = err_r; % Store current error for next iteration's derivative calculation

    %% ERROR CALCULATION FOR THETA -> PITCH
    err_p = 0; % tmpp
    int_err_p = 0;
    tau_q = 0;
    
    % ----------- TO IMPROVE -------------------
    % err_p = x(BETA) - x(THETA); % beta - theta
    % int_err_p = int_err_p + err_p * Ts; % Accumulate integral error
    % der_err_p = (err_p - prev_err_p) / Ts; % Calculate derivative error
    % 
    % % Anti-windup for the integrator
    % int_err_p = max(min(int_err_p, integral_max), -integral_max); % Clamp integral error
    % 
    % % PID
    % tau_q = (Kp * err_p + Ki * int_err_p + Kd * der_err_p); % Calculate PID output
    % 
    % % Final saturation q_ref = tau
    % tau_q = max(min(tau_q, tau_q_max), -tau_q_max); % Clamp pitch velocity to limits

    % Memory update
    prev_err_p = err_p; % Store current error for next iteration's derivative calculation

    %% FINAL
    % Output control commands
    tau = [tau_u, tau_w, tau_p, tau_q]'; 
    % Error update
    p_err = [prev_err_u, prev_err_h, prev_err_r, prev_err_p]';
    int_err = [int_err_u, int_err_h, int_err_r, int_err_p]';

    fprintf('Pred Surge: %.2f m | Error: %.2f | u_ref: %.3f m/s\n', v_surge, err_u, tau_u);
    fprintf('Pred Alt: %.2f m | Error: %.2f | w_ref: %.3f m/s\n', x(1), err_h, tau_w);
    fprintf('Pred roll: %.2f | Error: %.2f | p_ref: %.3f\n', rad2deg(x(PHI)), rad2deg(err_r), tau_p);
    fprintf('Pred Pitch: %.2f | Error: %.2f | p_ref: %.3f\n', rad2deg(x(THETA)), rad2deg(err_p), tau_q);
end