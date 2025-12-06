%% JACOBIAN_H - Compute Jacobian of measurement function
%
% Computes the Jacobian matrix of the measurement function h with respect
% to the state vector x. Used in EKF update step to compute innovation
% covariance: S = H*P*H' + R and Kalman gain: K = P*H'/S
%
% SYNTAX:
%   H = jacobian_h(x, s, num_m, num_n, num_s, n, n0)
%
% INPUTS:
%   x       - Current state vector [3x1]: [h, alpha, beta]
%   s       - Sensor direction vectors in body frame [3 x num_s]
%   num_m   - Number of measurements (typically 4)
%   num_n   - Number of states (3)
%   num_s   - Number of sensors (typically 4)
%   n       - Current terrain normal vector in world frame [3x1]
%   n0      - Reference normal vector [3x1] (typically [0, 0, 1]')
%
% OUTPUTS:
%   H       - Measurement Jacobian [num_m x num_n]
%             H(i,j) = ∂h_i/∂x_j evaluated at current state
%
% JACOBIAN STRUCTURE:
%   For each sensor i:
%     H(i, :) = [∂z_i/∂h, ∂z_i/∂α, ∂z_i/∂β]
%
% PARTIAL DERIVATIVES:
%   ∂z_i/∂h = -1 / (n' * s_i)
%   
%   ∂z_i/∂α = h * (∂n/∂α)' * s_i / (n' * s_i)²
%   
%   ∂z_i/∂β = h * (∂n/∂β)' * s_i / (n' * s_i)²
%
% THEORY:
%   The measurement model is:
%     z_i = -h / (n' * s_i)
%
%   where n = Ry(β) * Rx(α) * Rx(π) * n0
%
%   Normal vector derivatives:
%     ∂n/∂α = Ry(β) * ∂Rx(α)/∂α * Rx(π) * n0
%     ∂n/∂β = ∂Ry(β)/∂β * Rx(α) * Rx(π) * n0
%
% NOTES:
%   - Measurement sensitive to both altitude and terrain orientation
%   - Singularity when sensor parallel to terrain (n' * s_i = 0)
%   - Higher altitude → larger sensitivity to angle errors
%   - Uses chain rule for rotation matrix derivatives
%
% See also: h, jacobian_f, d_rotx, d_roty, doc/EKF_ALGORITHM.md

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