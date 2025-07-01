function H = jacobian_h(x, s, num_m, num_n, num_s, n, n0, Gamma, Lambda, Eta, Zeta)
    % Jacobian of the observation function with respect to x
    
    %% Definition
    IND_H = 1;                  I_IND_U = 1;
    ALPHA = 2;                  I_IND_W = 2;
    BETA = 3;
    PHI = 4;      M_PHI = 5;
    THETA = 5;    M_THETA = 6;
    IND_P = 6;    M_IND_P = 7;  I_IND_P = 3;
    IND_Q = 7;    M_IND_Q = 8;  I_IND_Q = 4;    

    %% Computations
    H = zeros(num_m, num_n);
    dn_alpha_t = (roty(x(BETA)) * (d_rotx(x(ALPHA)))' * n0)';
    dn_beta_t = (d_roty(x(BETA))' * (rotx(x(ALPHA))) * n0)';
    d_s(:, 1) = [-cos(Gamma + x(THETA,1)), 0, -sin(Gamma + x(THETA,1))]';
    d_s(:, 2) = [-cos(Lambda + x(THETA,1)), 0, -sin(Lambda + x(THETA,1))]';
    d_s(:, 3) = [0, -cos(Eta + x(PHI,1)), -sin(Eta + x(PHI,1))]';
    d_s(:, 4) = [0, -cos(Zeta + x(PHI,1)), -sin(Zeta + x(PHI,1))]';


    for j = 1:num_s
        % Compute the observation Jacobian for each sensor
        H(j, IND_H) = 1 / (n' * s(:, j)); % Derivative with respect to h
        H(j, ALPHA) = -(x(IND_H) * dn_alpha_t * s(:, j)) / (n' * s(:, j))^2;
        H(j, BETA) = -(x(IND_H) * dn_beta_t * s(:, j)) / (n' * s(:, j))^2;
        if j <= 2
            H(j, M_PHI) = 0;
            H(j, M_THETA) = -(x(IND_H) * n' * d_s(:, j)) / (n' * s(:, j))^2;
            
        else
            H(j, M_PHI) = -(x(IND_H) * n' * d_s(:, j)) / (n' * s(:, j))^2;
            H(j, M_THETA) = 0;
        end
    end
    H(M_PHI, PHI) = 1; % Derivative with respect to phi
    H(M_THETA, THETA) = 1; % Derivative with respect to theta
    H(M_IND_P, IND_P) = 1; % Derivative with respect to p
    H(M_IND_Q, IND_Q) = 1; % Derivative with respect to q   
end