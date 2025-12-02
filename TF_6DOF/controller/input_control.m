function [pid, int_term, pre_err, err_i, acc, term_sum] = input_control(ggg, x, angles, old_pid, int_term, speed, old_speed, ...
                        acc, old_t_s, wRr, wRt, Ts, dim_i, pre_err, err_i, Kp, Ki, Kd, Kt)
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;

    printDebug('       Input Control\n');

    %% Initialize Control Variables
    max_pid = ones(dim_i, 1);  % Saturation limits for all DOFs
    term_sum = zeros(dim_i, 1);
    pid = zeros(dim_i, 1);

    %% Compute Current Acceleration
    % Numerical differentiation of velocity
    acc = derivator(acc, speed, old_speed, Ts); 
    
    %% Transform Velocities and Accelerations to Terrain Frame
    % Express velocities in terrain-aligned frame for surge/sway/heave control
    s_speed = wRt' * wRr * speed(SURGE:HEAVE);
    s_acc = wRt' * wRr * acc(SURGE:HEAVE);

    %% Compute Control Errors
    err = zeros(dim_i, 1);
    
    % Translational errors (in terrain frame)
    err(SURGE) = (ggg.surge - s_speed(SURGE));
    err(SWAY) = (ggg.sway - s_speed(SWAY));
    err(HEAVE) = (ggg.altitude - x(IND_H));  % Altitude tracking (not velocity)
    
    % Rotational errors (in body frame)
    err(ROLL) = (ggg.roll - angles(PHI));
    err(PITCH) = (ggg.pitch - angles(THETA));
    err(YAW) = (ggg.yaw - angles(PSI));

    %% Controller Implementation
    % ============================================================
    % DELTA FORM WITH ANTI-WINDUP (Recommended)
    % ============================================================
    % This form integrates: Ki*e - Kp*v - Kd*a - Kt*pid_old
    % Provides better anti-windup and smoother control
    
    % Compute PID terms for each DOF
    for j = 1:dim_i
        % Integral term (acts on error)
        i_err = Ki(j) * err(j);
        
        if j == SURGE || j == SWAY
            % PI control for surge/sway (no derivative)
            % Proportional acts on acceleration (terrain frame)
            p_err = (Kp(j) * s_acc(j));
            d_err = 0;
        elseif j == HEAVE
            % PID control for heave (altitude tracking)
            % Proportional acts on vertical velocity
            % Derivative acts on vertical acceleration
            p_err = (Kp(j) * s_speed(j));
            d_err = (Kd(j) * s_acc(j));
        else
            % PID control for roll/pitch/yaw
            % Proportional and derivative act on angular rates
            p_err = (Kp(j) * speed(j));
            d_err = (Kd(j) * acc(j));
        end
        
        % Delta term sum: I - P - D - anti-windup
        term_sum(j) = i_err - p_err - d_err;
    end
    
    % Transform surge/sway/heave terms back to robot frame
    tp_speed = wRr' * wRt * [term_sum(SURGE); term_sum(SWAY); term_sum(HEAVE)];
    
    % Integrate with anti-windup for each DOF
    for j = 1:dim_i
        % Update translational components with transformed values
        if j < ROLL
            term_sum(j) = tp_speed(j);
        end
        
        % Add anti-windup correction term
        term_sum(j) = term_sum(j) - Kt(j)*old_pid(j);
        
        % Integrate the delta term
        int_term(j) = integrator(int_term(j), term_sum(j), old_t_s(j), Ts);
        
        % Apply saturation and compute anti-windup correction
        pid_sat = max(min(int_term(j), max_pid(j)), -max_pid(j));
        pid(j) = int_term(j) - pid_sat;  % Store difference for next iteration
    end

    %% Debug Output
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SURGE), pid(SURGE));
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SWAY), pid(SWAY));
    printDebug('Error: %.2f | w_ref: %.3f m/s\n', err(HEAVE), pid(HEAVE));
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err(ROLL)), rad2deg(pid(ROLL)));
    printDebug('Error: %.2f | q_ref: %.3f rad/s\n', rad2deg(err(PITCH)), rad2deg(pid(PITCH)));
    printDebug('Error: %.2f | r_ref: %.3f rad/s\n', rad2deg(err(YAW)), rad2deg(pid(YAW)));
end