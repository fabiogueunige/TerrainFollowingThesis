function [s_dotdot, new_vel] = dynamic_model(tau, tau0, sp0, ang, v_old, Ts, i_dim, old_acc, choice)
    
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
    z = 0.005;          % Vertical distance to center of buoyancy [m] (reduced for stability)
    B = rho * volume * g;  % Buoyancy force [N]

    %% Hydrodynamic Coefficients (Tuned for BlueROV2)
    % Added mass (negative of acceleration-induced forces)
    tau_a = -[27.08; 25.952; 29.9081; 0.15; 0.18; 0.18];
    
    % Linear damping coefficients (increased for large angle stability)
    tau_r = [-0.1213; -1.1732; -1.1130; -4.0; -4.0; -3.5];
    
    % Quadratic damping coefficients (increased for large angle stability)
    tau_d = [-23.9000; -46.2700; -50.2780; -8.0; -8.0; -7.0];
    
    %% Effective System Properties
    % Virtual mass (total mass + added mass)
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;
    
    % Nonlinear damping at current velocity (for reference)
    dv = -tau_r - tau_d .* abs(v_old);
    
    % Linearized damping around operating point sp0
    dv_lin = -tau_r - 2 * tau_d .* abs(sp0);
    
    % Velocity deviation from operating point
    delta = v_old - sp0;
    
    %% Linearized Dynamics - Compute Accelerations
    % Initialize acceleration vector
    s_dotdot = zeros(i_dim,1);
    
    % Linearized equation: M * a = tau - tau0 - D_lin * (v - v0)
    % For each DOF: a = (tau - tau0 - D_lin*delta_v) / M
    
    if choice == -1 || choice == -2
        %% LINEARIZED DYNAMICS
        for l = 1:i_dim
            s_dotdot(l) = (tau(l) - tau0(l) - dv_lin(l)*delta(l)) / mv(l);
        end
    else
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
    end
    
    %% Velocity Integration
    % Integrate accelerations to get new velocities using Tustin method
    new_vel = integrator(v_old, s_dotdot, old_acc, Ts);
end