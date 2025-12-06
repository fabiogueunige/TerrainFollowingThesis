function [s_dotdot, new_vel] = dynamic_model(tau, ang, v_old, Ts, i_dim, old_acc)
    
    %% State Indices
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    
    %% Angle Indices (for ang parameter)
    PHI = 1;    THETA = 2;  PSI = 3;

    %% BlueROV2 Physical Parameters
    m = 11.5;       % Total mass [kg]
    I = diag([0.21, 0.245, 0.245]);  % Inertia tensor [kg*m²]
    
    % Fluid and buoyancy properties
    rho = 1028;         % Seawater density [kg/m³]
    volume = 0.011054;  % Displaced volume [m³]
    g = 9.81;           % Gravitational acceleration [m/s²]
    z = -0.0420;         % Vertical distance to center of buoyancy [m]
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
    
    %% Linearized Dynamics - Compute Accelerations
    % Initialize acceleration vector
    s_dotdot = zeros(i_dim,1);
    
    % Linearized equation: M * a = tau - tau0 - D_lin * (v - v0)
    % For each DOF: a = (tau - tau0 - D_lin*delta_v) / M
    %% NONLINEAR MODEL (ALTERNATIVE - Currently Commented)
    % Full 6-DOF nonlinear dynamics including:
    % - Coriolis and centripetal forces (coupling between DOFs)
    % - Nonlinear quadratic damping
    % - Full restoring force coupling with orientation

    % Surge: includes lateral/vertical velocity coupling
    s_dotdot(U) = (tau(U) + mv(V)*v_old(V)*v_old(R) - mv(W)*v_old(W)*v_old(Q) ...
                - dv(U)*v_old(U)) / mv(U);

    % Sway: includes longitudinal/vertical velocity coupling
    s_dotdot(V) = (tau(V) - mv(U)*v_old(U)*v_old(R) + mv(W)*v_old(W)*v_old(P) ...
                - dv(V)*v_old(V)) / mv(V);

    % Heave: includes longitudinal/lateral velocity coupling
    s_dotdot(W) = (tau(W) + mv(U)*v_old(U)*v_old(Q) - mv(V)*v_old(V)*v_old(P) ...
                - dv(W)*v_old(W)) / mv(W);

    % Roll: includes sway/heave and pitch/yaw rate coupling + restoring moment
    s_dotdot(P) = (tau(P) + (mv(V)-mv(W))*v_old(V)*v_old(W) ...
                + (mv(Q)-mv(R))*v_old(Q)*v_old(R) - dv(P)*v_old(P) ...
                - z*B*cos(ang(THETA))*sin(ang(PHI))) / mv(P);

    % Pitch: includes surge/heave and roll/yaw rate coupling + restoring moment
    s_dotdot(Q) = (tau(Q) - (mv(U)-mv(W))*v_old(U)*v_old(W) ...
                - (mv(P)-mv(R))*v_old(P)*v_old(R) - dv(Q)*v_old(Q) ...
                + z*B*sin(ang(THETA))) / mv(Q);

    % Yaw: includes surge/sway and roll/pitch rate coupling
    s_dotdot(R) = (tau(R) + (mv(U)-mv(V))*v_old(U)*v_old(V) ...
                + (mv(P)-mv(Q))*v_old(P)*v_old(Q) - dv(R)*v_old(R)) / mv(R);
    
    %% Velocity Integration
    % Integrate accelerations to get new velocities using Tustin method
    new_vel = integrator(v_old, s_dotdot, old_acc, Ts);
end