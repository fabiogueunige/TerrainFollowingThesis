function tau0 = tau0_values(sp0, i_dim)
    
    %% State Indices
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    SURGE = U;  SWAY = V;   HEAVE = W;
    ROLL = P;   PITCH = Q;  YAW = R;
    
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
    z = 0.0420;         % Vertical distance to center of buoyancy [m]
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