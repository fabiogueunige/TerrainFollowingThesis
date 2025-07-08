function [tau, p_err, int_err] = input_control(x, Ts, prev_err, int_err, v_surge, wRr, wRt)
    %% Definitions
    IND_H = 1;                  
    ALPHA = 2;                  
    BETA = 3;
    PHI = 4;        M_PHI = 5;    
    THETA = 5;      M_THETA = 6;  
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;
    C_U = 1;        C_H = 2;        C_ROLL = 3;     C_PITCH = 4;

    global DEBUG
    printDebug('       Input Control\n');
    %% SYSTEM PARAMETERS
    u_star = 0.3;         % [m/s] Constant surge velocity
    h_star = 7;           % Reference altitude
    des_u_max = 1;        % [m/s] Robot surge velocity saturation
    des_w_max = 1;        % [m/s] Vertical velocity saturation
    des_p_max = 1;        % Roll velocity saturation
    des_q_max = 1;        % Pitch velocity saturation
    
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
    u_des = u_star;
    err_u = 0;
    int_err_u = 0;

    % % NON FUNZIONA, Qualche problema!! A SAPERE QUALE BOH
    % err_u = u_star - v_surge;
    % int_err_u = int_err_u + err_u * Ts; % Accumulate integral error
    % der_err_u = (err_u - prev_err_u) / Ts; % Calculate derivative error
    % 
    % % Anti-windup for the integrator
    % int_err_u = max(min(int_err_u, integral_max), -integral_max); % Clamp integral error
    % 
    % % PID
    % u_des = (Kp * err_u + Ki * int_err_u + Kd * der_err_u); % Calculate PID output
    % Having the PID i can consider it as tau
    % 
    % % Final saturation
    % u_des = max(min(u_des, des_u_max), -des_u_max); % Clamp pitch velocity to limits

    % Memory update
    prev_err_u = err_u; % Store current error for next iteration's derivative calculation

    %% ERROR CALCULATION FOR H
    % err_h = 0; % tmpp
    % int_err_h = 0;
    % tau_w = 0;

    % Quello vero
    err_h = (h_star - x(IND_H)); % Negative for robot frame velocity (error calculation)
    int_err_h = int_err_h + err_h * Ts; % Accumulate integral error
    der_err_h = (err_h - prev_err_h) / Ts; % Calculate derivative error

    % Anti-windup for the integrator
    int_err_h = max(min(int_err_h, integral_max), -integral_max); % Clamp integral error

    % % NON VA BENE: Devo considerare all'interno l'effetto del surge
    % % NON VA BENE: Devo considerare all'interno l'effetto del surge
    % PID on altitude frame
    pid = (Kp * err_h + Ki * int_err_h + Kd * der_err_h);

    % PID on world and robot frame
    tp_speed = wRr' * wRt * [0; 0; pid];
    w_des = tp_speed(3);

    % Final saturation
    w_des = max(min(w_des, des_w_max), -des_w_max); % Clamp vertical velocity to limits
    
    % Memory update
    prev_err_h = err_h; % Store current error for next iteration's derivative calculation
    
    %% ERROR CALCULATION FOR PHI -> ROLL
    % err_r = 0; % tmpp
    % int_err_r = 0;
    % tau_p = 0;
    
    % ----------- TO IMPROVE -------------------
    err_r = x(ALPHA) - x(PHI); % alpha - phi
    int_err_r = int_err_r + err_r * Ts; % Accumulate integral error
    der_err_r = (err_r - prev_err_r) / Ts; % Calculate derivative error

    % Anti-windup for the integrator
    int_err_r = max(min(int_err_r, integral_max), -integral_max); % Clamp integral error
    
    % PID
    p_des = (Kp * err_r + Ki * int_err_r + Kd * der_err_r); % Calculate PID output

    % Final saturation q_ref = tau
    p_des = max(min(p_des, des_p_max), -des_p_max); % Clamp pitch velocity to limits

    % Memory update
    prev_err_r = err_r; % Store current error for next iteration's derivative calculation

    %% ERROR CALCULATION FOR THETA -> PITCH
    % err_p = 0; % tmpp
    % int_err_p = 0;
    % tau_q = 0;
    
    % ----------- TO IMPROVE -------------------
    err_p = x(BETA) - x(THETA); % beta - theta
    int_err_p = int_err_p + err_p * Ts; % Accumulate integral error
    der_err_p = (err_p - prev_err_p) / Ts; % Calculate derivative error

    % Anti-windup for the integrator
    int_err_p = max(min(int_err_p, integral_max), -integral_max); % Clamp integral error

    % PID
    des_q = (Kp * err_p + Ki * int_err_p + Kd * der_err_p); % Calculate PID output

    % Final saturation q_ref = tau
    des_q = max(min(des_q, des_q_max), -des_q_max); % Clamp pitch velocity to limits

    % Memory update
    prev_err_p = err_p; % Store current error for next iteration's derivative calculation

    %% FINAL
    % Output control commands
    tau = [u_des, w_des, p_des, des_q]'; 
    % Error update
    p_err = [prev_err_u, prev_err_h, prev_err_r, prev_err_p]';
    int_err = [int_err_u, int_err_h, int_err_r, int_err_p]';

    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err_u, u_des);
    printDebug('Error: %.2f | w_ref: %.3f m/s\n', err_h, w_des);
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err_r), p_des);
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err_p), des_q);
end