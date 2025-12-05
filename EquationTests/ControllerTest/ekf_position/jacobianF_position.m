function [F_d] = jacobianF_position(x_old, wRr_old, dim_f, Ts)
    F = zeros(dim_f, dim_f);

    %% State Indices
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;

    eta = x_old(1:6);
    nu = x_old(7:12);
    phi = eta(4);
    theta = eta(5);
    psi = eta(6);
    u = nu(1);    v = nu(2);    w = nu(3);
    p = nu(4);    q = nu(5);    r = nu(6);
    
    %% dynamic parameters
    m = 11.5;       % Total mass [kg]
    I = diag([0.21, 0.245, 0.245]);  % Inertia tensor [kg*m²]
    rho = 1028;         % Seawater density [kg/m³]
    volume = 0.011054;  % Displaced volume [m³]
    g = 9.81;
    z_B = -0.0420;         % Vertical distance to center of buoyancy [m]
    B = rho * volume * g;  % Buoyancy force [N]

    W_rest = z_B * B;

    tau_a = -[27.08; 25.952; 29.9081; 1; 1; 1];
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5];
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1];
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;
    dv = -tau_r - tau_d .* abs(nu);

    %% position 
    % position 
    % F(1:3,4:6) = 0;
    % orientation
    F(1:3,4) = rotz(psi) * roty(theta) * d_rotx(phi) * nu(4:6);
    F(1:3,5) = rotz(psi) * d_roty(theta) * rotx(phi) * nu(4:6);
    F(1:3,6) = d_rotz(psi) * roty(theta) * rotx(phi) * nu(4:6);
    % velocity
    F(1:3,7:9) = wRr_old;
    % bias = 0

    %% Orientation
    T = transformationT(eta(4:6));
    % position = 0
    % orientation
    F(4:6,4) = dT_dphi(phi, theta) * nu(1:3);
    F(4:6,5) = dT_dtheta(phi, theta) * nu(4:6);
    F(4:6,6) = zeros(3,1);
    % velocity
    F(4:6,10:12) = T;
    F(4:6,13:15) = -T;

    %% velocity linear
    % --- SURGE (u_dot) ---
    % Eq: (Tau + mv(V)*v*r - mv(W)*w*q - dv(U)*u) / mv(U)
    F(7,7)  = -dv(U) / mv(U);          % d/du
    F(7,8)  = (mv(V) * r) / mv(U);     % d/dv
    F(7,9)  = -(mv(W) * q) / mv(U);    % d/dw
    F(7,11) = -(mv(W) * w) / mv(U);    % d/dq
    F(7,12) = (mv(V) * v) / mv(U);     % d/dr

    % --- SWAY (v_dot) ---
    % Eq: (Tau - mv(U)*u*r + mv(W)*w*p - dv(V)*v) / mv(V)
    F(8,7)  = -(mv(U) * r) / mv(V);    % d/du
    F(8,8)  = -dv(V) / mv(V);          % d/dv
    F(8,9)  = (mv(W) * p) / mv(V);     % d/dw
    F(8,10) = (mv(W) * w) / mv(V);     % d/dp
    F(8,12) = -(mv(U) * u) / mv(V);    % d/dr

    % --- HEAVE (w_dot) ---
    % Eq: (Tau + mv(U)*u*q - mv(V)*v*p - dv(W)*w) / mv(W)
    F(9,7)  = (mv(U) * q) / mv(W);     % d/du
    F(9,8)  = -(mv(V) * p) / mv(W);    % d/dv
    F(9,9)  = -dv(W) / mv(W);          % d/dw
    F(9,10) = -(mv(V) * v) / mv(W);    % d/dp
    F(9,11) = (mv(U) * u) / mv(W);     % d/dq

    %% velocity angular
    % Termini comuni momento raddrizzante
    % Roll_restoring:  -z*B * cth * sphi
    % Pitch_restoring: +z*B * sth
    
    % --- ROLL RATE (p_dot) ---
    % Eq: (Tau + (mv(V)-mv(W))vw + (mv(Q)-mv(R))qr - dv(P)*p - zB*cth*sphi) / mv(P)
    
    % Derivate Attitudine (Restoring)
    F(10,4) = (-W_rest * cos(theta) * cos(phi)) / mv(P);    % d/dphi
    F(10,5) = (W_rest * sin(theta) * sin(phi)) / mv(P);     % d/dtheta (derivata di cos è -sin, col meno davanti diventa +)
    
    % Derivate Stati Dinamici
    F(10,8)  = ((mv(V) - mv(W)) * w) / mv(P);          % d/dv
    F(10,9)  = ((mv(V) - mv(W)) * v) / mv(P);          % d/dw
    F(10,10) = -dv(P) / mv(P);                        % d/dp
    F(10,11) = ((mv(Q) - mv(R)) * r) / mv(P);          % d/dq
    F(10,12) = ((mv(Q) - mv(R)) * q) / mv(P);          % d/dr
    % --- PITCH RATE (q_dot) ---
    % Eq: (Tau - (mv(U)-mv(W))uw - (mv(P)-mv(R))pr - dv(Q)*q + zB*sth) / mv(Q)
    
    % Derivate Attitudine
    F(11,5) = (W_rest * cos(theta)) / mv(Q);            % d/dtheta
    
    % Derivate Stati Dinamici
    F(11,7)  = -((mv(U) - mv(W)) * w) / mv(Q);         % d/du
    F(11,9)  = -((mv(U) - mv(W)) * u) / mv(Q);         % d/dw
    F(11,10) = -((mv(P) - mv(R)) * r) / mv(Q);         % d/dp
    F(11,11) = -dv(Q) / mv(Q);                         % d/dq
    F(11,12) = -((mv(P) - mv(R)) * p) / mv(Q);         % d/dr

    % --- YAW RATE (r_dot) ---
    % Eq: (Tau + (mv(U)-mv(V))uv + (mv(P)-mv(Q))pq - dv(R)*r) / mv(R)
    F(12,7)  = ((mv(U) - mv(V)) * v) / mv(R);          % d/du
    F(12,8)  = ((mv(U) - mv(V)) * u) / mv(R);          % d/dv
    F(12,10) = ((mv(P) - mv(Q)) * q) / mv(R);          % d/dp
    F(12,11) = ((mv(P) - mv(Q)) * p) / mv(R);          % d/dq
    F(12,12) = -dv(R) / mv(R);                      % d/dr

    % discretizzazione
    F_d = eye(dim_f) + F * Ts;
end
