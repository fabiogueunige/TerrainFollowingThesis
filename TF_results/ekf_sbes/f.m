%% F - EKF state transition function (prediction step)
%
% Implements the nonlinear state transition model for the Extended Kalman
% Filter. Predicts the next terrain-relative state (altitude and terrain
% orientation) based on current state and robot velocities.
%
% SYNTAX:
%   [x_next, wRt_p] = f(x, u_input, Ts, wRr)
%
% INPUTS:
%   x       - Current state vector [3x1]: [h, alpha, beta]
%             h: altitude above terrain [m]
%             alpha: terrain roll angle [rad]
%             beta: terrain pitch angle [rad]
%   u_input - Robot body velocities [6x1]: (u, v, w, p, q, r)
%   Ts      - Sampling time [s]
%   wRr     - World to robot rotation matrix [3x3]
%
% OUTPUTS:
%   x_next  - Predicted state vector [3x1]: [h_new, alpha_new, beta_new]
%   wRt_p   - Predicted world to terrain rotation matrix [3x3]
%
% STATE TRANSITION MODEL:
%   h(k+1) = h(k) + (wRt' * wRr * v_body)_z * Ts
%   alpha(k+1) = alpha(k)  [terrain angles assumed quasi-static]
%   beta(k+1) = beta(k)
%
%   where (wRt' * wRr * v_body)_z is the vertical velocity in terrain frame
%
% COORDINATE FRAMES:
%   - Body frame: robot-fixed coordinate system
%   - World frame: NED inertial frame (z down from surface)
%   - Terrain frame: aligned with local terrain (n perpendicular to surface)
%
% ASSUMPTIONS:
%   - Terrain orientation changes slowly (quasi-static terrain)
%   - Robot velocities accurately measured (from DVL/AHRS)
%   - Small angle approximations NOT used (full nonlinear model)
%
% NOTES:
%   - Angles wrapped to [-π, π] for consistency
%   - Terrain rotation: ZYX Euler angles (yaw=0, pitch=beta, roll=alpha)
%   - Additional π rotation accounts for terrain normal pointing down
%
% See also: jacobian_f, h, jacobian_h, doc/EKF_ALGORITHM.md

function [x_next, wRt_p] = f(x, u_input, Ts, wRr)
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global SURGE; global SWAY; global HEAVE;

    printDebug('       f State Prediction\n');
    
    %% Angle Wrapping
    % Ensure terrain angles remain in [-π, π] range
    alpha_new = wrapToPi(x(ALPHA));
    beta_new = wrapToPi(x(BETA));
    
    %% Terrain Rotation Matrix
    % Construct predicted terrain orientation
    % ZYX Euler: Rz(0) * Ry(beta) * Rx(alpha) * Rx(π)
    % Additional Rx(π) accounts for normal pointing downward
    wRt_p = rotz(0)*roty(beta_new)*rotx(alpha_new)*rotx(pi);
    
    %% Velocity Transformation
    % Transform body velocities to world frame
    w_speed = wRr * u_input(SURGE:HEAVE);
    
    % Transform to terrain-aligned frame
    s_speed = (wRt_p)' * w_speed;

    %% Altitude Update
    % Integrate vertical velocity in terrain frame
    h_new = x(IND_H) + s_speed(HEAVE)*Ts;

    % Debug output for monitoring prediction
    printDebug('Predicted h: %.2f m | a: %.2f | b: %.2f \n', h_new, rad2deg(alpha_new), rad2deg(beta_new));
    
    %% Output Predicted State
    x_next = [h_new, alpha_new, beta_new]';
end