%% JACOBIAN_F - Compute Jacobian of state transition function
%
% Computes the Jacobian matrix of the state transition function f with
% respect to the state vector x. Used in EKF prediction step to propagate
% state covariance: P(k+1|k) = F*P(k|k)*F' + Q
%
% SYNTAX:
%   F = jacobian_f(x, u_input, Ts, num_n, wRr)
%
% INPUTS:
%   x       - Current state vector [3x1]: [h, alpha, beta]
%   u_input - Robot body velocities [6x1]: (u, v, w, p, q, r)
%   Ts      - Sampling time [s]
%   num_n   - Number of states (3)
%   wRr     - World to robot rotation matrix [3x3]
%
% OUTPUTS:
%   F       - State transition Jacobian [3x3]
%             F(i,j) = ∂f_i/∂x_j evaluated at current state
%
% JACOBIAN STRUCTURE:
%   F = [1,  ∂h/∂α,  ∂h/∂β ]
%       [0,    1,      0    ]
%       [0,    0,      1    ]
%
% PARTIAL DERIVATIVES:
%   ∂h/∂α = [(∂Ry/∂β * Rx(α) * Rx(π))' * wRr * v]_z * Ts
%   ∂h/∂β = [(Ry(β) * ∂Rx/∂α * Rx(π))' * wRr * v]_z * Ts
%
%   where v = [u, v, w]' is body velocity
%
% THEORY:
%   The state transition is:
%     h(k+1) = h(k) + [wRt' * wRr * v]_z * Ts
%     α(k+1) = α(k)
%     β(k+1) = β(k)
%
%   where wRt = Rz(0) * Ry(β) * Rx(α) * Rx(π)
%
%   Since α and β are assumed constant, their Jacobian rows are identity.
%   The altitude Jacobian requires derivatives of rotation matrices.
%
% NOTES:
%   - Uses rotation matrix derivatives: d_rotx, d_roty
%   - Only third component (heave) of rotated velocity affects altitude
%   - Jacobian evaluated at current state (local linearization)
%
% See also: f, jacobian_h, d_rotx, d_roty, doc/EKF_ALGORITHM.md

function F = jacobian_f(x, u_input, Ts, num_n, wRr)
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global SURGE; global SWAY; global HEAVE;
    
    printDebug('       jacobian F Covariance\n')

    %% Initialize Jacobian
    % Start with identity (α and β rows remain unchanged)
    F = eye(num_n);
    
    % Velocity-time product for efficiency
    r_speedTs = u_input(SURGE:HEAVE) * Ts;

    %% Altitude Partial Derivatives
    % ∂h/∂α: derivative with respect to terrain roll angle
    h_alpha = (roty(x(BETA)) * d_rotx(x(ALPHA)) * rotx(pi))' * wRr*r_speedTs;
    
    % ∂h/∂β: derivative with respect to terrain pitch angle
    h_beta = (d_roty(x(BETA)) * rotx(x(ALPHA)) * rotx(pi))' * wRr*r_speedTs;
    
    % Populate first row of Jacobian (altitude derivatives)
    F(IND_H, ALPHA) = h_alpha(3);  % Extract z-component
    F(IND_H, BETA) = h_beta(3);    % Extract z-component
end