%% REFERENCE_CORRECTION - Correct terrain normal orientation and angles
%
% Ensures terrain normal vector points downward (negative z-component in
% world frame) and computes corresponding terrain orientation angles.
% Handles gimbal lock and ensures angle consistency.
%
% SYNTAX:
%   [a, b] = reference_correction(n_cap, alph, bet)
%
% INPUTS:
%   n_cap - Computed terrain normal vector [3x1] (may need inversion)
%   alph  - Current alpha angle estimate [rad] (terrain roll)
%   bet   - Current beta angle estimate [rad] (terrain pitch)
%
% OUTPUTS:
%   a - Corrected alpha angle [rad] (terrain roll, wrapped to [-π, π])
%   b - Corrected beta angle [rad] (terrain pitch, wrapped to [-π, π])
%
% COORDINATE CONVENTION:
%   In NED (North-East-Down) world frame:
%   - Terrain normal should point downward: n_z < 0
%   - If n_z > 0, normal points upward → invert and recompute angles
%
% ANGLE COMPUTATION:
%   From corrected normal n = [n_x, n_y, n_z]':
%   
%   ZYX Euler angles:
%     beta  = atan2(-n_x, √(n_y² + n_z²))
%     alpha = atan2(n_y, n_z)
%
%   Handles gimbal lock when √(n_y² + n_z²) ≈ 0:
%     beta = ±π/2 (depending on sign of n_x)
%     alpha = 0 (undefined, set to zero by convention)
%
% ALGORITHM:
%   1. Check if n_z > epsilon (normal pointing upward)
%   2. If yes: invert normal and recompute angles
%   3. Handle gimbal lock case (beta ≈ ±π/2)
%   4. Wrap angles to [-π, π] for consistency
%   5. If no correction needed: return original angles
%
% TOLERANCE:
%   epsilon = 1e-6 for numerical stability in comparisons
%
% NOTES:
%   - Critical for EKF consistency (maintains normal direction convention)
%   - Prevents 180° ambiguity in terrain orientation
%   - Debug output available when correction applied
%
% See also: plane_computation, SBES_measurament, vector_normalization

function [a, b] = reference_correction(n_cap, alph, bet)
    %% Numerical Tolerance
    epsilon = 1e-6;  % Tolerance for zero comparisons
    
    %% Check Normal Direction
    % In world frame (NED), terrain normal should point upward and be negative (n_z < 0)
    if n_cap(3) > epsilon
        % Normal points downward → needs correction
        % fprintf('Reference correction: Inverting normal vector (n_z = %.4f)\n', n_cap(3));
        
        %% Invert and Normalize
        new_n_cap = -n_cap;
        new_n = vector_normalization(new_n_cap);
        
        %% Recompute Angles from Corrected Normal
        % Using ZYX Euler angles convention: Rz(ψ) * Ry(β) * Rx(α)
        % For terrain: ψ = 0, so normal depends only on α and β
        
        % Magnitude in yz-plane
        bet_part = sqrt(new_n(2)^2 + new_n(3)^2);
        
        %% Handle Gimbal Lock
        if bet_part < epsilon
            % Near gimbal lock: beta ≈ ±π/2
            % Alpha becomes undefined, set to 0 by convention
            beta_corr = sign(-new_n(1)) * pi/2;
            alpha_corr = 0;  % Alpha undefined at gimbal lock
        else
            % Standard angle extraction
            % beta: pitch angle (rotation about y-axis)
            beta_corr  = atan2(-new_n(1), bet_part);
            % alpha: roll angle (rotation about x-axis)
            alpha_corr = atan2(new_n(2), new_n(3));
        end
        
        %% Wrap Angles to [-π, π]
        b = wrapToPi(beta_corr);
        a = wrapToPi(alpha_corr);
        
        printDebug('  Corrected angles: alpha = %.4f rad, beta = %.4f rad\n', a, b);
    else
        % Normal already points upward → no correction needed
        a = alph;
        b = bet;
    end
end