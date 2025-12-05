function [x_pred] = f_position(x_old, tau, wRr_old, Q_loc, dim_f, Ts)
    x_predict = zeros(dim_f, 1);

    %% dynamic parameters%% State Indices
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    
    %% Angle Indices (for ang parameter)
    PHI = 4;    THETA = 5; 

    eta = x_old(1:6);
    nu = x_old(7:12);

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
    dv = -tau_r - tau_d .* abs(nu);

    %% position
    x_predict(1:3) = wRr_old * nu(1:3) * Ts;
    T = transformationT(eta(4:6));
    x_predict(4:6) = T * nu(4:6) * Ts;

    %% velocity
    x_predict(7) = (tau(U) + mv(V)*nu(V)*nu(R) - mv(W)*nu(W)*nu(Q) ...
                - dv(U)*nu(U)) * Ts / mv(U);

    % Sway: includes longitudinal/vertical velocity coupling
    x_predict(8) = (tau(V) - mv(U)*nu(U)*nu(R) + mv(W)*nu(W)*nu(P) ...
                - dv(V)*nu(V)) * Ts / mv(V);

    % Heave: includes longitudinal/lateral velocity coupling
    x_predict(9) = (tau(W) + mv(U)*nu(U)*nu(Q) - mv(V)*nu(V)*nu(P) ...
                - dv(W)*nu(W)) * Ts / mv(W);

    % Roll: includes sway/heave and pitch/yaw rate coupling + restoring moment
    x_predict(10) = (tau(P) + (mv(V)-mv(W))*nu(V)*nu(W) ...
                + (mv(Q)-mv(R))*nu(Q)*nu(R) - dv(P)*nu(P) ...
                - z*B*cos(eta(THETA))*sin(eta(PHI))) * Ts / mv(P);

    % Pitch: includes surge/heave and roll/yaw rate coupling + restoring moment
    x_predict(11) = (tau(Q) - (mv(U)-mv(W))*nu(U)*nu(W) ...
                - (mv(P)-mv(R))*nu(P)*nu(R) - dv(Q)*nu(Q) ...
                + z*B*sin(eta(THETA))) * Ts / mv(Q);

    % Yaw: includes surge/sway and roll/pitch rate coupling
    x_predict(12) = (tau(R) + (mv(U)-mv(V))*nu(U)*nu(V) ...
                + (mv(P)-mv(Q))*nu(P)*nu(Q) - dv(R)*nu(R)) * Ts / mv(R);

    %% bias
    % bias is 0 with the error

    %% process update
    x_pred = x_old + x_predict;
end
