function [tau] = tau_generator(Ts, old_speed, input_star, tau0, speed0)
    a_target = (input_star - old_speed) / Ts;
    tau = zeros(2,1);

    % medusa model
    m = 30;
    
    % Added mass
    tau_a = [-25; -19.4311]; % kl_dot
    
    % Restoring forces
    tau_r = [-0.2; 33.6804]; % kl
    
    % Hydrodynamic forces
    tau_d = [-19.5;  -89.4105]; % kl_modl
    
    % Virtual mass
    mv = [m; m] - tau_a;

    % Dissatipative forces
    dv = -tau_r - 2*tau_d .* abs(speed0);

    % soon removing point of equilibrium
    delta = old_speed - speed0;

    % control generator
    for l = 1:2
        tau(l) = mv(l) * a_target(l) + tau0(l) - dv(l) * delta(l);
    end
end