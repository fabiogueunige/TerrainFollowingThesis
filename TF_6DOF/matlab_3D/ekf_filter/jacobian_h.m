function H = jacobian_h(x, s, num_m, num_n, num_s, n, n0)
    % Jacobian of the observation function with respect to x
    printDebug('       Jacobian H covariance\n');
    
    %% Definition
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  

    %% Computations
    H = zeros(num_m, num_n);
    new_n0 = rotx(pi)*n0;
    dn_alpha_t = (roty(x(BETA)) * (d_rotx(x(ALPHA))) * new_n0)';
    dn_beta_t = (d_roty(x(BETA)) * (rotx(x(ALPHA))) * new_n0)';

    for j = 1:num_s
        % Compute the observation Jacobian for each sensor
        H(j, IND_H) = -1 / (n' * s(:, j)); % Derivative with respect to h
        H(j, ALPHA) = (x(IND_H) * dn_alpha_t * s(:, j)) / (n' * s(:, j))^2;
        H(j, BETA) = (x(IND_H) * dn_beta_t * s(:, j)) / (n' * s(:, j))^2;
    end
end