function new_vel = dynamic_model(tau, tau0, speed0, old_vel, Ts)
    % deve darmi la velocità w
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

    % speed0 = [surge, heave]';
    delta = old_vel - speed0;

    s_dotdot = zeros(2,1);
    
    % surge
    s_dotdot(1) = (tau(1) - tau0(1) - dv(1)*delta(1)) / mv(1); % + mv(2)*speed0(2)*delta(4) + mv(2)*speed0(4)*delta(2)

    % heave
    s_dotdot(2) = (tau(2) - tau0(2) + dv(2)*delta(2)) / mv(2);
  
    new_vel = old_vel + Ts * s_dotdot;
end