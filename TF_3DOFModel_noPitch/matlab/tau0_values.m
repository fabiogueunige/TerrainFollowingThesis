function tau0 = tau0_values(speed0)
    % updated to medusa model
    % m = 30;
    % I = 4.14;
    % g = 9.81;
    WB = -0.8339; % Buoyancy

    % speed0 = [surge, heave]';
    
    % Restoring forces
    tau_r = [-0.2; 33.6804]; % kl
    
    % Hydrodynamic forces
    tau_d = [-19.5;  -89.4105]; % kl_modl
    

    % Dissatipative forces
    dv = -tau_r - 2*tau_d .* abs(speed0);
    
    % tau0 computation
    tau0 = zeros(2,1);
    tau0(1) = dv(1)*speed0(1); 
    % Attenzione tau0(2) e speed0(2) solo per questo caso
    tau0(2) = -dv(2)*speed0(2) - WB;
        
end