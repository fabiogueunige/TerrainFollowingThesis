function H = jacobian_h(x, s, num_m, num_n, num_s, n, n0, r_s, psi)
    % Jacobian of the observation function with respect to x
    fprintf('       Jacobian H covariance\n');
    %% Definition
    IND_H = 1;                  
    ALPHA = 2;                  
    BETA = 3;
    PHI = 4;        M_PHI = 5;    
    THETA = 5;      M_THETA = 6;  
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;   

    %% Computations
    H = zeros(num_m, num_n);
    new_n0 = rotx(pi)'*n0;
    dn_alpha_t = (roty(x(BETA)) * (d_rotx(x(ALPHA))) * new_n0)';
    dn_beta_t = (d_roty(x(BETA)) * (rotx(x(ALPHA))) * new_n0)';
    ds_phi = rotz(psi) * roty(x(THETA)) * d_rotx(x(PHI));%*x(IND_P)
    ds_theta = rotz(psi) * d_roty(x(THETA)) * rotx(x(PHI));%*x(IND_Q)

    for j = 1:num_s
        % Compute the observation Jacobian for each sensor
        H(j, IND_H) = 1 / (n' * s(:, j)); % Derivative with respect to h
        H(j, ALPHA) = -(x(IND_H) * dn_alpha_t * s(:, j)) / (n' * s(:, j))^2;
        H(j, BETA) = -(x(IND_H) * dn_beta_t * s(:, j)) / (n' * s(:, j))^2;
        H(j,PHI) = -(x(IND_H) * n' * (ds_phi*r_s(:, j))) / (n' * s(:, j))^2;
        H(j,THETA) = -(x(IND_H) * n' * (ds_theta*r_s(:, j))) / (n' * s(:, j))^2;
    end
    H(M_PHI, PHI) = 1;     % Derivative with respect to phi
    H(M_THETA, THETA) = 1; % Derivative with respect to theta
    % H(M_IND_P, IND_P) = 1; % Derivative with respect to p
    % H(M_IND_Q, IND_Q) = 1; % Derivative with respect to q  
end