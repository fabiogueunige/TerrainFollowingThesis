function [pid, term_sum] = input_control(ggg, x, angles, old_pid, speed, ...
                        acc, old_t_s, wRr, w_n, Ts, dim_i, Kp, Ki, Kd)
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;

    printDebug('       Input Control\n');

    %% Initialize Control Variables
    max_pid = ones(dim_i, 1) * 10;  % Saturation limits for all DOFs
    term_sum = zeros(dim_i, 1);
    pid = zeros(dim_i, 1);
    err = zeros(dim_i, 1);
    
    h_err = (ggg.altitude - x(IND_H)); 
    % Normal vector in robot frame (how altitude h projects onto robot axes)
    % Transform world normal to robot frame
    r_n = wRr' * w_n;  % Normal in robot frame
    r_n = r_n / norm(r_n);  % Normalize
    h_contribution = h_err * r_n;  % [surge; sway; heave] contribution
    
    % Translational errors (in terrain frame)
    err(SURGE) = (ggg.surge - speed(SURGE) + 0.3 * h_contribution(SURGE));
    err(SWAY) = (ggg.sway - speed(SWAY) + 0.3 * h_contribution(SWAY));
    err(HEAVE) = h_contribution(HEAVE);  % Altitude tracking
    
    % Rotational errors (in body frame)
    err(ROLL) = (ggg.roll - angles(PHI));
    err(PITCH) = (ggg.pitch - angles(THETA));
    err(YAW) = (ggg.yaw - angles(PSI));

    %% Controller Implementation
    % ============================================================
    % DELTA FORM WITH ANTI-WINDUP (Recommended)
    % ============================================================
    % Provides better anti-windup and smoother control
    
    % Compute PID terms for each DOF
    for j = 1:dim_i
        % Integral term (acts on error)
        i_err = Ki(j) * err(j);
        
        if j == SURGE || j == SWAY
            % PI control for surge/sway (no derivative)
            % Proportional acts on acceleration (terrain frame)
            p_err = (Kp(j) * acc(j));
            d_err = 0;
        else
            % PID control for heave/roll/pitch/yaw
            % Proportional and derivative act on angular rates
            p_err = (Kp(j) * speed(j));
            d_err = (Kd(j) * acc(j));
        
        % Delta term sum: I - P - D - anti-windup
        term_sum(j) = i_err - p_err - d_err;
        
        % Integrate the delta term
        pid(j) = integrator(old_pid(j), term_sum(j), old_t_s(j), Ts);
        
        % Apply saturation and compute anti-windup correction
        pid(j) = max(min(pid(j), max_pid(j)), -max_pid(j));
    end

    %% Debug Output
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SURGE), pid(SURGE));
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SWAY), pid(SWAY));
    printDebug('Error: %.2f | w_ref: %.3f m/s\n', err(HEAVE), pid(HEAVE));
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err(ROLL)), rad2deg(pid(ROLL)));
    printDebug('Error: %.2f | q_ref: %.3f rad/s\n', rad2deg(err(PITCH)), rad2deg(pid(PITCH)));
    printDebug('Error: %.2f | r_ref: %.3f rad/s\n', rad2deg(err(YAW)), rad2deg(pid(YAW)));
end