%% INPUT_CONTROL - 6-DOF PID controller with anti-windup for AUV
%
% Implements a multi-DOF PID controller with anti-windup mechanism for
% autonomous underwater vehicle control. Supports both delta form with
% anti-windup ("A+D") and classical PID form.
%
% SYNTAX:
%   [pid, int_term, pre_err, err_i, acc, term_sum] = input_control(ggg, x, ...
%       angles, old_pid, int_term, speed, old_speed, acc, old_t_s, wRr, ...
%       wRt, Ts, dim_i, pre_err, err_i, Kp, Ki, Kd, Kt)
%
% INPUTS:
%   ggg         - Goal structure with setpoints (surge, sway, altitude, roll, pitch, yaw)
%   x           - EKF state vector [h, alpha, beta]
%   angles      - Current Euler angles [phi, theta, psi]
%   old_pid     - Previous PID output [6x1]
%   int_term    - Integral term state [6x1]
%   speed       - Current body velocities [6x1] (u, v, w, p, q, r)
%   old_speed   - Previous body velocities [6x1]
%   acc         - Current body accelerations [6x1]
%   old_t_s     - Previous term sum (for delta form) [6x1]
%   wRr         - World to robot rotation matrix [3x3]
%   wRt         - World to terrain rotation matrix [3x3]
%   Ts          - Sampling time [s]
%   dim_i       - Input dimension (6)
%   pre_err     - Previous error (for classical PID) [6x1]
%   err_i       - Integrated error (for classical PID) [6x1]
%   Kp, Ki, Kd  - PID gains [6x1]
%   Kt          - Anti-windup gains [6x1]
%
% OUTPUTS:
%   pid         - Control output [6x1] (velocity references or corrections)
%   int_term    - Updated integral term state [6x1]
%   pre_err     - Updated previous error [6x1]
%   err_i       - Updated integrated error [6x1]
%   acc         - Updated acceleration estimate [6x1]
%   term_sum    - Current term sum [6x1]
%
% CONTROL TYPES:
%   "A+D" - Delta form with anti-windup (RECOMMENDED)
%           Provides better windup protection and smoother control
%           PID output = integral of (Ki*e - Kp*dx/dt - Kd*d²x/dt² - Kt*pid_old)
%
%   "PID" - Classical PID form
%           Standard form: pid = Kp*e + Ki*∫e*dt + Kd*de/dt
%
% COORDINATE FRAMES:
%   - Errors computed in terrain frame for surge/sway/heave
%   - Angular errors computed in body frame
%   - Control outputs transformed to appropriate frames
%
% ANTI-WINDUP MECHANISM:
%   Prevents integrator windup during saturation by back-calculating
%   the difference between saturated and unsaturated values.
%
% NOTES:
%   - Heave control tracks altitude (h) from EKF, not velocity
%   - Derivative action applied to measured variable (not error) to avoid spikes
%   - Saturation limits: ±1.0 for all DOFs
%
% See also: gainComputation, tau0_values, dynamic_model

function [pid, int_term, acc, term_sum] = input_control(ggg, x, angles, old_pid, int_term, speed, old_speed, ...
                        acc, old_t_s, wRr, w_n, stat, Ts, dim_i, Kp, Ki, Kd, Kt)
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

    %% Compute altitude error
    r_n = wRr' * w_n;  % Transform terrain normal in world frame to robot frame
    r_n = vector_normalization(r_n);  % Normalize terrain normal vector
    % Project altitude error onto each robot axis
    % To correct h, we need velocity along the normal direction
    % Each axis contributes proportionally to its alignment with the normal
    % h_contribution is the velocity command in robot frame [surge; sway; heave]
    err_h = -(ggg.altitude - x(IND_H));  % = h(k-1) - h_star (current - target)

    if (stat == "Following" || stat == "MoveRoll" || stat == "MovePitch" || stat == "ContactSearch")
        h_contribution = err_h * r_n;  % [surge; sway; heave] contribution

        %% Compute Control Errors
        err = zeros(dim_i, 1);
        
        % Translational errors (in terrain frame)
        err(SURGE) = (ggg.surge - speed(SURGE) + 0.3 * h_contribution(SURGE));
        err(SWAY) = (ggg.sway - speed(SWAY) + 0.3 * h_contribution(SWAY));
        err(HEAVE) = h_contribution(HEAVE);  % Heave error based on altitude correction
    else
        err(SURGE) = (ggg.surge - speed(SURGE));
        err(SWAY) = (ggg.sway - speed(SWAY));
        err(HEAVE) = -err_h;  % Direct altitude error for heave
    end
        
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
            p_err = (Kp(j) * acc(j));
            d_err = 0;
        else
            % PID control for heave/roll/pitch/yaw
            % Proportional and derivative act on angular rates
            p_err = (Kp(j) * speed(j));
            d_err = (Kd(j) * acc(j));
        end
        
        % Delta term sum: I - P - D - anti-windup
        term_sum(j) = i_err - p_err - d_err - Kt(j)*old_pid(j);

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