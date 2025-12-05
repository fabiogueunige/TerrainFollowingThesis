function [eta_dot, eta] = kinematic_model(eta, nu, wRr, old_eta_dot, dim_i, Ts)
    printDebug('       Kinematic Model\n');
    
    eta_dot = zeros(dim_i,1);

    eta_dot(1:3) = wRr * nu(1:3);
    
    T = transformationT(eta(4:6));
    
    % Transform angular rates
    eta_dot(4:6) = T * nu(4:6);

    for j = 1:dim_i
        eta(j) = integrator(eta(j), eta_dot(j), old_eta_dot(j), Ts);
    end
end
