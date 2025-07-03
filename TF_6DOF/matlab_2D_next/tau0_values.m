function tau0 = tau0_values(speed0)
    % updated to bluerov model
    theta0 = 0;
    rho = 1028;
    volume = 0.011054;
    g = 9.81;

    % restoring
    z = 0.0420;
    B = rho * volume * g;

    % [surge, heave, pitch]
    
    % Linear damping
    tau_r = [-0.1213; -1.1130; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -50.2780; -1]; % quadratic_damping (kl_modl)
    

    % Dissatipative forces
    dv = -tau_r - 2*tau_d .* abs(speed0);
    
    % tau0 computation
    tau0 = zeros(3,1);
    tau0(1) = dv(1)*speed0(1);
    % heave
    tau0(2) = dv(2)*speed0(2);
    % pitch
    tau0(3) = dv(3)*speed0(3) + z*B*sin(theta0);
end