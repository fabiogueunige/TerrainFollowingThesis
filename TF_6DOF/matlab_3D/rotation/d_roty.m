function dRy = d_roty(angle)
    % D_ROTY Derivative of rotation matrix around Y-axis with respect to angle
    %
    % Input:
    %   angle - Rotation angle [rad]
    %
    % Output:
    %   dRy   - 3x3 derivative matrix (∂Ry/∂angle)
    %
    % Used in Jacobian computations for EKF
    
    dRy = [-sin(angle),  0,  cos(angle);
                     0,  0,           0;
           -cos(angle),  0, -sin(angle)];
end