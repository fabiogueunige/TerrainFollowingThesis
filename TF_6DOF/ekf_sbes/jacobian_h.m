function H = jacobian_h(x, s, num_m, num_n, num_s, n, n0)
    printDebug('       Jacobian H covariance\n');
    
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;  

    %% Initialize Jacobian
    H = zeros(num_m, num_n);
    
    %% Normal Vector Derivatives
    % Account for π rotation in terrain frame definition
    new_n0 = rotx(pi)*n0;
    
    % ∂n/∂α: partial derivative of normal with respect to roll
    dn_alpha_t = (roty(x(BETA)) * (d_rotx(x(ALPHA))) * new_n0)';
    
    % ∂n/∂β: partial derivative of normal with respect to pitch
    dn_beta_t = (d_roty(x(BETA)) * (rotx(x(ALPHA))) * new_n0)';

    %% Compute Jacobian for Each Sensor
    for j = 1:num_s
        % Denominator term (n' * s_j)
        n_dot_s = n' * s(:, j);
        
        % ∂z_j/∂h: derivative with respect to altitude
        H(j, IND_H) = -1 / n_dot_s;
        
        % ∂z_j/∂α: derivative with respect to terrain roll
        H(j, ALPHA) = (x(IND_H) * dn_alpha_t * s(:, j)) / (n_dot_s)^2;
        
        % ∂z_j/∂β: derivative with respect to terrain pitch
        H(j, BETA) = (x(IND_H) * dn_beta_t * s(:, j)) / (n_dot_s)^2;
    end
end