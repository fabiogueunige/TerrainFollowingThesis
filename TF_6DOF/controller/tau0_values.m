%% TAU0_VALUES - Compute equilibrium forces/torques for AUV at operating point
%
% Computes the steady-state forces and torques required to maintain a given
% velocity vector, accounting for damping and restoring forces. These values
% are used as feedforward terms in the control system.
%
% SYNTAX:
%   tau0 = tau0_values(sp0, i_dim)
%
% INPUTS:
%   sp0     - Operating point velocity vector [6x1] (u, v, w, p, q, r)
%   i_dim   - Number of input dimensions (6 for full 6-DOF control)
%
% OUTPUTS:
%   tau0    - Equilibrium forces/torques [6x1]
%             [X, Y, Z, K, M, N] in body frame
%
% THEORY:
%   At equilibrium (zero acceleration), forces balance:
%     tau0 = D(v)*v + g(η)
%   where:
%     D(v) = linear damping + quadratic damping*|v|
%     g(η) = restoring forces from buoyancy (roll/pitch coupling)
%
% PHYSICAL MODEL:
%   - BlueROV2 parameters (mass, volume, center of buoyancy)
%   - Seawater density: 1028 kg/m³
%   - Buoyancy arm: z = 0.0420 m (vertical offset to center of buoyancy)
%
% NOTES:
%   - Assumes small roll/pitch angles (sin(θ) ≈ 0, sin(φ) ≈ 0)
%   - Quadratic damping dominates at higher speeds
%   - Used as feedforward term in input_control.m
%
% See also: gainComputation, input_control, dynamic_model

function tau0 = tau0_values(sp0, i_dim)
    global DEBUG
    printDebug('       Tau0 generator\n');
    
    %% State Indices
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;
    
    %% Equilibrium Computation
    % Assumed equilibrium angles (hover condition)
    theta0 = 0;  % Pitch angle at equilibrium [rad]
    phi0 = 0;    % Roll angle at equilibrium [rad]

    %% BlueROV2 Physical Parameters
    % Fluid properties
    rho = 1028;         % Seawater density [kg/m³]
    volume = 0.011054;  % Displaced volume [m³]
    g = 9.81;           % Gravitational acceleration [m/s²]

    % Restoring force parameters
    z = -0.0420;         % Vertical distance to center of buoyancy [m]
    B = rho * volume * g;  % Buoyancy force [N]

    %% Damping Coefficients
    % Linear damping (proportional to velocity)
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5];
    
    % Quadratic damping (proportional to |velocity|*velocity)
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1];
    
    % Total damping forces at operating point sp0
    dv = -tau_r - tau_d .* abs(sp0);

    % Total damping forces at operating point sp0
    dv = -tau_r - tau_d .* abs(sp0);

    %% Equilibrium Forces and Torques
    % Initialize output vector
    tau0 = zeros(i_dim,1);
    
    % Surge: X-force (forward/backward)
    tau0(SURGE) = dv(U)*sp0(U);
    
    % Sway: Y-force (left/right)
    tau0(SWAY) = dv(V)*sp0(V);
    
    % Heave: Z-force (up/down)
    tau0(HEAVE) = dv(W)*sp0(W);
    
    % Roll: K-torque (rotation about x-axis)
    % Includes restoring torque from buoyancy offset
    tau0(ROLL) = dv(P)*sp0(P) + z*B*sin(theta0)*sin(phi0);
    
    % Pitch: M-torque (rotation about y-axis)
    % Includes restoring torque from buoyancy offset
    tau0(PITCH) = dv(Q)*sp0(Q) + z*B*sin(theta0);
    
    % Yaw: N-torque (rotation about z-axis)
    tau0(YAW) = dv(R)*sp0(R);
end