%% DYNAMIC_MODEL - 6-DOF AUV dynamics with linearized hydrodynamics
%
% Implements the linearized equations of motion for an underwater vehicle,
% computing accelerations from applied forces/torques and current velocities.
% Based on BlueROV2 parameters and hydrodynamic coefficients.
%
% SYNTAX:
%   [new_vel] = dynamic_model(tau, tau0, sp0, ang, v_old, Ts, i_dim, old_acc)
%
% INPUTS:
%   tau     - Applied forces/torques [6x1]: [X, Y, Z, K, M, N]
%   tau0    - Equilibrium forces/torques at operating point [6x1]
%   sp0     - Operating point velocities [6x1]: (u0, v0, w0, p0, q0, r0)
%   ang     - Current Euler angles [3x1]: (phi, theta, psi)
%   v_old   - Previous velocities [6x1]: (u, v, w, p, q, r)
%   Ts      - Sampling time [s]
%   i_dim   - Input dimension (6)
%   old_acc - Previous accelerations [6x1]
%
% OUTPUTS:
%   new_vel - Updated velocities [6x1]: (u, v, w, p, q, r)
%
% DYNAMICS EQUATIONS (Linearized):
%   M * dv/dt = tau - tau0 - D_lin * (v - v0)
%
%   where:
%     M = virtual mass matrix (mass + added mass)
%     D_lin = linearized damping around operating point v0
%     tau = applied control forces/torques
%     tau0 = equilibrium forces at v0
%
% HYDRODYNAMIC MODEL:
%   - Virtual mass: M = m - M_A (added mass reduces effective inertia)
%   - Linear damping: proportional to velocity
%   - Quadratic damping: linearized as 2*k*|v0| around operating point
%   - Restoring forces: from buoyancy offset (pitch/roll coupling)
%
% LINEARIZATION:
%   Nonlinear term: D(v)*v = (D_linear + D_quad*|v|)*v
%   Linearization: D(v)*v ≈ D(v0)*v0 + D'(v0)*(v-v0)
%                         = tau0 + D_lin*(v-v0)
%   where D_lin = D_linear + 2*D_quad*|v0|
%
% BLUEROV2 PARAMETERS:
%   Mass:        11.5 kg
%   Volume:      0.011054 m³
%   Inertia:     diag([0.21, 0.245, 0.245]) kg*m²
%   Buoyancy arm: 0.042 m (vertical offset)
%
% NOTES:
%   - Decoupled dynamics (no Coriolis/centripetal terms in linear model)
%   - Linearization valid for small deviations from operating point
%   - Commented nonlinear model available for reference
%   - Integration using Tustin method (trapezoidal)
%
% ALTERNATIVE:
%   Full nonlinear model (commented) includes:
%   - Coriolis and centripetal forces
%   - Nonlinear damping
%   - Full restoring force coupling
%   See commented section for implementation details
%
% See also: tau0_values, gainComputation, integrator

function [new_vel] = dynamic_model(tau, tau0, sp0, ang, v_old, Ts, i_dim, old_acc)
    printDebug('       Dynamic Model\n');
    
    %% State Indices
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;
    global PHI; global THETA; global PSI; 

    global PHI; global THETA; global PSI; 

    %% BlueROV2 Physical Parameters
    m = 11.5;       % Total mass [kg]
    I = diag([0.21, 0.245, 0.245]);  % Inertia tensor [kg*m²]
    
    % Fluid and buoyancy properties
    rho = 1028;         % Seawater density [kg/m³]
    volume = 0.011054;  % Displaced volume [m³]
    g = 9.81;           % Gravitational acceleration [m/s²]
    z = 0.0420;         % Vertical distance to center of buoyancy [m]
    B = rho * volume * g;  % Buoyancy force [N]

    %% Hydrodynamic Coefficients
    % Added mass (negative of acceleration-induced forces)
    tau_a = -[27.08; 25.952; 29.9081; 1; 1; 1];
    
    % Linear damping coefficients
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5];
    
    % Quadratic damping coefficients
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1];
    
    %% Effective System Properties
    % Virtual mass (total mass + added mass)
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;
    
    % Nonlinear damping at current velocity (for reference)
    dv = -tau_r - tau_d .* abs(v_old);
    
    % Linearized damping around operating point sp0
    dv_lin = -tau_r - 2 * tau_d .* abs(sp0);
    
    % Velocity deviation from operating point
    delta = v_old - sp0;
    % Velocity deviation from operating point
    delta = v_old - sp0;
    
    %% Linearized Dynamics - Compute Accelerations
    % Initialize acceleration vector
    s_dotdot = zeros(i_dim,1);
    
    % Linearized equation: M * a = tau - tau0 - D_lin * (v - v0)
    % For each DOF: a = (tau - tau0 - D_lin*delta_v) / M
    
    % Surge: X-axis acceleration (forward/backward)
    s_dotdot(SURGE) = (tau(SURGE) - tau0(SURGE) - dv_lin(U)*delta(SURGE)) / mv(U);

    % Sway: Y-axis acceleration (left/right)
    s_dotdot(SWAY) = (tau(SWAY) - tau0(SWAY) - dv_lin(V)*delta(SWAY)) / mv(V);
 
    % Heave: Z-axis acceleration (up/down)
    s_dotdot(HEAVE) = (tau(HEAVE) - tau0(HEAVE) - dv_lin(W)*delta(HEAVE)) / mv(W);

    % Roll: K-torque acceleration (rotation about x-axis)
    s_dotdot(ROLL) = (tau(ROLL) - tau0(ROLL) - dv_lin(P)*delta(ROLL)) / mv(P);
    
    % Pitch: M-torque acceleration (rotation about y-axis)
    s_dotdot(PITCH) = (tau(PITCH) - tau0(PITCH) - dv_lin(Q)*delta(PITCH)) / mv(Q);

    % Yaw: N-torque acceleration (rotation about z-axis)
    s_dotdot(YAW) = (tau(YAW) - tau0(YAW) - dv_lin(R)*delta(YAW)) / mv(R);

    %% NONLINEAR MODEL (ALTERNATIVE - Currently Commented)
    % Full 6-DOF nonlinear dynamics including:
    % - Coriolis and centripetal forces (coupling between DOFs)
    % - Nonlinear quadratic damping
    % - Full restoring force coupling with orientation
    %
    % Uncomment to use full nonlinear model instead of linearized version
    %
    % Surge: includes lateral/vertical velocity coupling
    % s_dotdot(U) = (tau(U) + mv(V)*v_old(V)*v_old(R) - mv(W)*v_old(W)*v_old(Q) ...
    %                - dv(U)*v_old(U)) / mv(U);
    %
    % Sway: includes longitudinal/vertical velocity coupling
    % s_dotdot(V) = (tau(V) - mv(U)*v_old(U)*v_old(R) + mv(W)*v_old(W)*v_old(P) ...
    %                - dv(V)*v_old(V)) / mv(V);
    %
    % Heave: includes longitudinal/lateral velocity coupling
    % s_dotdot(W) = (tau(W) + mv(U)*v_old(U)*v_old(Q) - mv(V)*v_old(V)*v_old(P) ...
    %                - dv(W)*v_old(W)) / mv(W);
    %
    % Roll: includes sway/heave and pitch/yaw rate coupling + restoring moment
    % s_dotdot(P) = (tau(P) + (mv(V)-mv(W))*v_old(V)*v_old(W) ...
    %                + (mv(Q)-mv(R))*v_old(Q)*v_old(R) - dv(P)*v_old(P) ...
    %                - z*B*cos(ang(THETA))*sin(ang(PHI))) / mv(P);
    %
    % Pitch: includes surge/heave and roll/yaw rate coupling + restoring moment
    % s_dotdot(Q) = (tau(Q) - (mv(U)-mv(W))*v_old(U)*v_old(W) ...
    %                - (mv(P)-mv(R))*v_old(P)*v_old(R) - dv(Q)*v_old(Q) ...
    %                + z*B*sin(ang(THETA))) / mv(Q);
    %
    % Yaw: includes surge/sway and roll/pitch rate coupling
    % s_dotdot(R) = (tau(R) + (mv(U)-mv(V))*v_old(U)*v_old(V) ...
    %                + (mv(P)-mv(Q))*v_old(P)*v_old(Q) - dv(R)*v_old(R)) / mv(R);

    %% Velocity Integration
    % Integrate accelerations to get new velocities using Tustin method
    new_vel = integrator(v_old, s_dotdot, old_acc, Ts);

    %% Debug Output
    printDebug('surge: %.2f | sway: %.2f | heave: %.2f ', new_vel(SURGE), new_vel(SWAY), new_vel(HEAVE));
    printDebug('| p: %.3f | q: %.3f | r: %.3f\n', new_vel(ROLL), new_vel(PITCH), new_vel(YAW));
    printDebug('a_surge: %.2f | a_sway: %.2f | a_heave: %.2f ', s_dotdot(SURGE), s_dotdot(SWAY), s_dotdot(HEAVE));
    printDebug('| a_p: %.3f | a_q: %.3f | a_r: %.3f\n', s_dotdot(ROLL), s_dotdot(PITCH), s_dotdot(YAW));
end