function [tau] = tau_generator(Ts, old_speed, input_star, tau0, speed0)
    a_target = (input_star - old_speed) / Ts;
    
    tau = zeros(3,1);

    % updated to bluerov model
    m = 11.5; % massa totale [kg]
    I = diag([0.21, 0.245, 0.245]);

    % [surge, heave, pitch]
    % Added mass
    tau_a = [27.08; 29.9081; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1130; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -50.2780; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; I(2,2)] - tau_a;
    
    % Dissipative forces (con v0 = 0.1 solo nel surge)
    dv = -tau_r - 2 * tau_d .* abs(speed0);

    delta = old_speed - speed0;

    % control generator
    for l = 1:3
        tau(l) = mv(l) * a_target(l) + tau0(l) - dv(l) * delta(l);
    end
end