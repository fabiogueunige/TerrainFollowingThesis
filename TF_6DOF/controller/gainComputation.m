%% GAINCOMPUTATION - Compute PID gains for 6-DOF AUV control
%
% Computes proportional, integral, derivative, and anti-windup gains for
% the PID controller based on linearized AUV dynamics around an operating point.
% Uses pole placement and desired natural frequency/damping specifications.
%
% SYNTAX:
%   [kp, ki, kd, kt] = gainComputation(speed0, dim_i)
%
% INPUTS:
%   speed0  - Operating point velocity vector [6x1] (u, v, w, p, q, r)
%   dim_i   - Number of input dimensions (6 for full 6-DOF control)
%
% OUTPUTS:
%   kp      - Proportional gains [6x1]
%   ki      - Integral gains [6x1]
%   kd      - Derivative gains [6x1]
%   kt      - Anti-windup gains [6x1]
%
% THEORY:
%   For surge/sway (PI control):
%     Transfer function: G(s) = ki/(s^2 + (2*damp*wn)*s + wn^2)
%     kp = 2*damp*wn*mv - d_linear
%     ki = wn^2 * mv
%
%   For heave/roll/pitch/yaw (PID control):
%     Transfer function includes additional pole at s = -p
%     kp = mv*(wn^2 + 2*damp*p*wn)
%     ki = p*wn^2*mv
%     kd = (p + 2*damp*wn)*mv - d_linear
%     kt = 1/sqrt(Ti*Td) for anti-windup
%
% PARAMETERS:
%   wn   = 0.4   - Natural frequency [rad/s]
%   damp = 0.6   - Damping ratio (critically damped ≈ 1.0)
%   p    = 10    - Additional pole for PID design
%
% REFERENCE:
%   BlueROV2 model parameters from manufacturer specifications
%   See doc/CONTROL_SYSTEM.md for detailed gain computation methodology
%
% See also: input_control, tau0_values, dynamic_model

% function [kp, ki, kd, kt] = gainComputation(speed0, dim_i)
function [kp, ki, kd] = gainComputation(speed0, dim_i)
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;
    
    %% Control Design Parameters
    % Natural frequency and damping for pole placement
    wn = 0.4;       % Natural frequency [rad/s]
    damp = 0.7;     % Damping ratio (0.6 = slightly underdamped)
    p = 8;         % Additional pole for PID controller

    % Initialize gain vectors
    kp = zeros(dim_i, 1);
    ki = zeros(dim_i, 1);
    kd = zeros(dim_i, 1);
    % Ti = zeros(dim_i, 1);   % Integral time constant
    % Td = zeros(dim_i, 1);   % Derivative time constant
    % kt = zeros(dim_i, 1);   % Anti-windup gain

    %% BlueROV2 Physical Parameters
    m = 11.5;   % Total mass [kg]
    I = diag([0.21, 0.245, 0.245]);  % Inertia tensor [kg*m^2]

    % Added mass coefficients (negative of acceleration-induced forces)
    tau_a = -[27.08; 25.952; 29.9081; 1; 1; 1]; 
    
    % Linear damping coefficients (proportional to velocity)
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5];
    
    % Quadratic damping coefficients (proportional to |velocity|*velocity)
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1];
    
    %% Effective Dynamics
    % Virtual mass (total mass + added mass)
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

    % d_linear = -tau_r - 2*tau_d*|v0| (linearization)
    dv_lin = -tau_r - 2 * tau_d .* abs(speed0);
    
    %% Gain Computation
    % Different control strategies for translational vs rotational DOFs
    for l = 1:dim_i
        if l == 1 || l == 2
            % PI control for surge and sway (horizontal motion)
            % No derivative action needed for stable horizontal dynamics
            kp(l) = 2*damp*wn*mv(l) - dv_lin(l);
            ki(l) = wn^2 * mv(l);
            kd(l) = 0;
            % Ti(l) = kp(l)/ki(l);
            % kt(l) = 1/Ti(l);  % Anti-windup gain
        else
            % PID control for heave, roll, pitch, yaw
            % Derivative action helps with stability and faster response
            kp(l) = mv(l)*((wn^2) + 2*damp*p*wn);
            ki(l) = p*(wn^2)*mv(l);
            kd(l) = (p + 2*damp*wn)*mv(l) - dv_lin(l);
            % Ti(l) = kp(l)/ki(l);
            % Td(l) = kd(l)/kp(l);
            % kt(l) = 1 / sqrt(Ti(l)*Td(l));  % Anti-windup gain (geometric mean)
        end
    end
end