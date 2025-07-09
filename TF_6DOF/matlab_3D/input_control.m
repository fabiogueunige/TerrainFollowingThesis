function [tau, p_err, int_err] = input_control(x, Ts, prev_err, int_err, angles, old_input, wRr, wRt)
    %% Definitions
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    % angles               
    PHI = 1;        THETA = 2;      PSI = 3;  
    % input
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;
    C_U = 1;        C_H = 2;        C_ROLL = 3;     C_PITCH = 4;

    global DEBUG
    printDebug('       Input Control\n');
    %% SYSTEM PARAMETERS
    s_u_star = 0.3;       % [m/s] Constant surge velocity
    s_v_star = 0.0;       % Constant sway velocity
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
    
    %% General Computation
    r_speed = [old_input(I_IND_U), 0, old_input(I_IND_W)]';
    s_speed = wRt' * wRr * r_speed;
    
    %% ERROR CALCULATION FOR SURGE
    % u_des = t_u_star;
    % err_u = 0;
    % int_err_u = 0;

    % NON FUNZIONA, Qualche problema!! A SAPERE QUALE BOH
    err_u = s_u_star - s_speed(I_IND_U);
    int_err_u = int_err_u + err_u * Ts; % Accumulate integral error
    der_err_u = (err_u - prev_err_u) / Ts; % Calculate derivative error

    % Anti-windup for the integrator
    int_err_u = max(min(int_err_u, integral_max), -integral_max); % Clamp integral error

    % PID
    pid_u = (Kp * err_u + Ki * int_err_u + Kd * der_err_u); % Calculate PID output
    
    % u_des
    tp_speed = wRr' * wRt * [pid_u; 0; 0];
    u_des = tp_speed(I_IND_U);

    % final saturation
    u_des = max(min(u_des, des_u_max), -des_u_max); % Clamp vertical velocity to limits

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    u_des = s_u_star;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Memory update
    prev_err_u = err_u; % Store current error for next iteration's derivative calculation
    
    %% ERROR CALCULATION FOR SWAY
    pid_v = 0;
    % NON FUNZIONA, Qualche problema!! A SAPERE QUALE BOH
    % err_v = t_v_star - s_speed(I_IND_V);
    % int_err_v = int_err_v + err_v * Ts; % Accumulate integral error
    % der_err_v = (err_v - prev_err_v) / Ts; % Calculate derivative error
    % 
    % % Anti-windup for the integrator
    % int_err_v = max(min(int_err_v, integral_max), -integral_max); % Clamp integral error
    % 
    % % PID
    % pid_v = (Kp * err_v + Ki * int_err_v + Kd * der_err_v); % Calculate PID output
    % 
    % % Memory update
    % prev_err_v = err_v; % Store current error for next iteration's derivative calculation

    %% ERROR CALCULATION FOR H
    % err_h = 0; 
    % int_err_h = 0;
    % tau_w = 0;

    % Pid Computation
    err_h = (h_star - x(IND_H)); % Negative for robot frame velocity (error calculation)
    int_err_h = int_err_h + err_h * Ts; % Accumulate integral error
    der_err_h = (err_h - prev_err_h) / Ts; % Calculate derivative error

    % Anti-windup for the integrator
    int_err_h = max(min(int_err_h, integral_max), -integral_max); % Clamp integral error

    % PID on altitude frame
    pid_h = (Kp * err_h + Ki * int_err_h + Kd * der_err_h);
    
    tp_speed = wRr' * wRt * [0; 0; pid_h];
    w_des = tp_speed(3); % I_IND_W

    w_des = max(min(w_des, des_w_max), -des_w_max); % Clamp vertical velocity to limits

    % Memory update
    prev_err_h = err_h; % Store current error for next iteration's derivative calculation
    
    %% SURGE, SWAY, HEAVE DESIRED VELOCITIES
    % PID on world and robot frame
    % % %% %% %% %% %% %% % %
    % Secondo me dovrebbero lavorare bene tutti quanti insieme, ma il surge
    % non funziona
    % tp_speed (tutti)
    % fare le desiderate di ognuno

    % v_des = max(min(v_des, des_v_max), -des_v_max); % Clamp vertical velocity to limits
    

    %% ERROR CALCULATION FOR PHI -> ROLL
    % err_r = 0; % tmpp
    % int_err_r = 0;
    % tau_p = 0;
    
    % ----------- TO IMPROVE ------------------- %
    err_r = x(ALPHA) - angles(PHI); % alpha - phi
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
    
    % ----------- TO IMPROVE ------------------- %
    err_p = x(BETA) - angles(THETA); % beta - theta
    int_err_p = int_err_p + err_p * Ts; % Accumulate integral error
    der_err_p = (err_p - prev_err_p) / Ts; % Calculate derivative error

    % Anti-windup for the integrator
    int_err_p = max(min(int_err_p, integral_max), -integral_max); % Clamp integral error

    % PID
    q_des = (Kp * err_p + Ki * int_err_p + Kd * der_err_p); % Calculate PID output

    % Final saturation q_ref = tau
    q_des = max(min(q_des, des_q_max), -des_q_max); % Clamp pitch velocity to limits

    % Memory update
    prev_err_p = err_p; % Store current error for next iteration's derivative calculation

    %% FINAL
    % Output control commands
    tau = [u_des, w_des, p_des, q_des]'; 
    % Error update
    p_err = [prev_err_u, prev_err_h, prev_err_r, prev_err_p]';
    int_err = [int_err_u, int_err_h, int_err_r, int_err_p]';

    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err_u, u_des);
    printDebug('Error: %.2f | w_ref: %.3f m/s\n', err_h, w_des);
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err_r), p_des);
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err_p), q_des);
end