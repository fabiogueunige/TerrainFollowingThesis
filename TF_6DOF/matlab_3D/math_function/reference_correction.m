function [a, b] = reference_correction(n_cap, alph, bet)
    %% Reference correction for terrain normal vector
    % The terrain normal should point downward (negative z in world frame)
    % If n_cap(3) > 0, the normal is inverted and needs correction
    
    epsilon = 1e-6;  % Tolerance for numerical stability
    
    %% Check if normal is pointing in the wrong direction
    if n_cap(3) > epsilon
        % Normal points upward -> invert it
        printDebug('Reference correction: Inverting normal vector (n_z = %.4f)\n', n_cap(3));
        
        % Invert and normalize the normal
        new_n_cap = -n_cap;
        new_n = vector_normalization(new_n_cap);
        
        % Recompute angles from corrected normal
        % Using ZYX Euler angles convention
        bet_part = sqrt(new_n(2)^2 + new_n(3)^2);
        
        % Handle gimbal lock case
        if bet_part < epsilon
            % Near gimbal lock: beta = ±π/2
            beta_corr = sign(-new_n(1)) * pi/2;
            alpha_corr = 0;  % Alpha is undefined, set to 0
        else
            beta_corr  = atan2(-new_n(1), bet_part);
            alpha_corr = atan2(new_n(2), new_n(3));
        end
        
        % Wrap angles to [-π, π]
        b = wrapToPi(beta_corr);
        a = wrapToPi(alpha_corr);
        
        printDebug('  Corrected angles: alpha = %.4f rad, beta = %.4f rad\n', a, b);
    else
        % Normal already points downward -> keep original angles
        a = alph;
        b = bet;
    end
end