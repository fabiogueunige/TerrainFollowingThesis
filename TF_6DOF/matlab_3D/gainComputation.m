function [kp, ki, kd, kt] = gainComputation(speed0, dim_i)
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;
    %% Linearization Parameters
    wn = 0.4;
    damp = 0.6;
    p = 10;

    kp = zeros(dim_i, 1);
    ki = zeros(dim_i, 1);
    kd = zeros(dim_i, 1);
    Ti = zeros(dim_i, 1);
    Td = zeros(dim_i, 1);
    kt = zeros(dim_i, 1);

    %% Model
    m = 11.5; % massa totale [kg]
    I = diag([0.21, 0.245, 0.245]);

    % Added mass
    tau_a = -[27.08; 25.952; 29.9081; 1; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

    % Dissipative forces (con v0 = 0.1 solo nel surge)
    dv = -tau_r - tau_d .* abs(speed0);
    dv_lin = -tau_r - 2 * tau_d .* abs(speed0);
    
    %% Computation
    for l = 1:dim_i
        if l == SURGE || l == SWAY
            kp(l) = 2*damp*wn*mv(l) - dv_lin(l);
            ki(l) = wn^2 * mv(l);
            kd(l) = 0;
            Ti(l) = kp(l)/ki(l);
            kt(l) = 1/Ti(l);
        else
            kp(l) = mv(l)*((wn^2) + 2*damp*p*wn);
            ki(l) = p*(wn^2)*mv(l);
            kd(l) = (p + 2*damp*wn)*mv(l) - dv_lin(l);
            Ti(l) = kp(l)/ki(l);
            Td(l) = kd(l)/kp(l);
            kt(l) = 1 / sqrt(Ti(l)*Td(l));
        end
    end
end