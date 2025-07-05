function tau0 = tau0_values(speed0, i_dim)
    % updated to bluerov model
    fprintf('       Tau0 generator\n');
    %% Definition
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;
    
    %% Tau 0 computation
    theta0 = 0;
    phi0 = 0;

    % m = 11.5; % massa totale [kg]
    % I = diag([0.21, 0.245, 0.245]);
    rho = 1028;
    volume = 0.011054;
    g = 9.81;

    % restoring
    z = 0.0420;
    B = rho * volume * g;

    % Added mass
    % tau_a = [27.08; 25.952; 29.9081; 1; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    % mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

    % Dissipative forces (con v0 = 0.1 solo nel surge)
    sp0 = [speed0(I_IND_U), 0, speed0(I_IND_W), speed0(I_IND_P), speed0(I_IND_Q), 0];
    dv = -tau_r - 2 * tau_d .* abs(sp0);

    % tau0 computation
    tau0 = zeros(i_dim,1);

    % surge
    tau0(I_IND_U) = dv(U)*sp0(U);
    % sway ...
    % heave
    tau0(I_IND_W) = dv(W)*sp0(W);
    % roll
    tau0(I_IND_P) = dv(P)*sp0(P) + z*B*sin(theta0)*sin(phi0);
    % pitch
    tau0(I_IND_Q) = dv(Q)*sp0(Q) + z*B*sin(theta0);
    % yaw ...
end