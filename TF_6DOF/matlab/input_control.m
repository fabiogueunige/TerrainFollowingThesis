function [u, pid_q, p_err, int_err] = input_control(x, Ts, prev_err, int_err)
    %% SYSTEM PARAMETERS
    u_star = 0.2;        % [m/s] Constant surge velocity
    w_max = 10;        % [m/s] Vertical velocity saturation
    q_max = 5;        % Pitch velocity saturation
    h_star = 7;        % Reference altitude
    prev_err_h = prev_err(1);
    prev_err_p = prev_err(2);
    int_err_h = int_err(1);
    int_err_p = int_err(2);

    %% PID PARAMETERS
    Kp = 0.5;          % Proportional gain
    Ki = 0.1;          % Integral gain
    Kd = 0.05;         % Derivative gain
    
    %% ERROR CALCULATION FOR H
    err_h = -(h_star - x(1)); % Negative for robot frame velocity (error calculation)
    int_err_h = int_err_h + err_h * Ts; % Accumulate integral error
    der_err_h = (err_h - prev_err_h) / Ts; % Calculate derivative error
    
    % Anti-windup for the integrator
    integral_max = 1.0; % Maximum allowed integral error
    int_err_h = max(min(int_err_h, integral_max), -integral_max); % Clamp integral error
    
    % PID CONTROL 
    cos_b = cos(x(2)); % Cosine of sonar angle (works for this case)
    if abs(cos_b) < 1e-3
        cos_b = sign(cos_b) * 1e-3;  % Numerical protection (avoid division by zero)
    end
    
    % PID on altitude frame
    pid_term = (Kp * err_h + Ki * int_err_h + Kd * der_err_h) / cos_b; % Calculate PID output
    
    % VERTICAL VELOCITY COMMAND
    % z = - (u*cos(theta)-w*sin(theta))*tan(beta)) + PID % Calculate reference vertical velocity
    % z = w*cos(theta) - u*sin(theta)

    % ------ u star va sostituito con velocità surge reale ottenuta con controllo ---------
    w_ref = (u_star*(sin(x(3))-cos(x(3))*tan(x(2))) + pid_term) / (cos(x(3))+sin(x(3))*tan(x(2)));
    
    % Final saturation
    w_ref = max(min(w_ref, w_max), -w_max); % Clamp vertical velocity to limits
    
    % Memory update
    prev_err_h = err_h; % Store current error for next iteration's derivative calculation
    
    %% ERROR CALCULATION FOR THETA
    % q_ref = 0;
    % err_p = 0;
    % prev_err_p = 0;
    % int_err_p = 0;
    % ----------- TO IMPROVE -------------------
    err_p = (x(2) - x(3)); % beta - theta
    int_err_p = int_err_p + err_p * Ts; % Accumulate integral error
    der_err_p = (err_p - prev_err_p) / Ts; % Calculate derivative error

    % Anti-windup for the integrator
    int_err_p = max(min(int_err_p, integral_max), -integral_max); % Clamp integral error

    % PID
    pid_term = (Kp * err_p + Ki * int_err_p + Kd * der_err_p); % Calculate PID output
    % Having the PID i can consider it as tau

    q_ref = pid_term;

    % Final saturation q_ref = tau
    q_ref = max(min(q_ref, q_max), -q_max); % Clamp pitch velocity to limits

    % Memory update
    prev_err_p = err_p; % Store current error for next iteration's derivative calculation

    %% FINAL
    u = [u_star, w_ref, q_ref]'; % Output control commands (surge velocity, vertical velocity)
    pid_q = q_ref;
    p_err = [prev_err_h, prev_err_p]';
    int_err = [int_err_h, int_err_p]';

    fprintf('Pred Alt: %.2f m | Error: %.2f | w_ref: %.3f m/s\n', x(1), err_h, w_ref);
    fprintf('Pred Pitch: %.2f | Error: %.2f | q_ref: %.3f m/s\n', rad2deg(x(3)), err_p, q_ref);
end