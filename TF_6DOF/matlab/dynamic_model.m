function [new_vel] = dynamic_model(tau, tau0, speed0, old_vel, Ts)
    % deve darmi la velocità w
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

    delta = old_vel - speed0;

    % Accelerations
    % 6 DOF MODEL Linearized
    s_dotdot = zeros(3,1);
    
    % surge
    s_dotdot(1) = (tau(1) - tau0(1) - dv(1)*delta(1)) / mv(1);
 
    % heave
    s_dotdot(2) = (tau(2) - tau0(2) - dv(2)*delta(2)) / mv(2);
    
    % pitch
    s_dotdot(3) = (tau(3) - tau0(3) - dv(3)*delta(3)) / mv(3);
  
    new_vel = old_vel + Ts * s_dotdot;
end