function [u, prev_err, int_err] = input_control(x, Ts, prev_err, int_err, theta)
    %% SYSTEM PARAMETERS
    u_star = 0.2;        % [m/s] Constant surge velocity
    w_max = 10;        % [m/s] Vertical velocity saturation
    h_star = 7;        % Reference altitude
    
    %% PID PARAMETERS
    Kp = 0.5;          % Proportional gain
    Ki = 0.1;          % Integral gain
    Kd = 0.05;         % Derivative gain
    
    %% ERROR CALCULATION 
    err = -(h_star - x(1)); % Negative for robot frame velocity (error calculation)
    int_err = int_err + err * Ts; % Accumulate integral error
    der_err = (err - prev_err) / Ts; % Calculate derivative error
    
    % Anti-windup for the integrator (optional)
    integral_max = 1.0; % Maximum allowed integral error
    int_err = max(min(int_err, integral_max), -integral_max); % Clamp integral error
    
    % PID CONTROL 
    cos_b = cos(x(2)); % Cosine of sonar angle (works for this case)
    if abs(cos_b) < 1e-3
        cos_b = sign(cos_b) * 1e-3;  % Numerical protection (avoid division by zero)
    end
    
    % PID on altitude frame
    pid_term = (Kp * err + Ki * int_err + Kd * der_err) / cos_b; % Calculate PID output
    
    %% VERTICAL VELOCITY COMMAND
    w_ref = (u_star*(sin(theta)-cos(theta)*tan(x(2))) + pid_term) / (cos(theta)+sin(theta)*tan(x(2)));
    
    % Final saturation
    w_ref = max(min(w_ref, w_max), -w_max); % Clamp vertical velocity to limits
    
    % Memory update
    prev_err = err; % Store current error for next iteration's derivative calculation
    
    u = [u_star, w_ref]'; % Output control commands (surge velocity, vertical velocity)

    fprintf('Predicted Alt: %.2f m | Error: %.2f | w_ref: %.3f m/s\n', x(1), err, w_ref);
end