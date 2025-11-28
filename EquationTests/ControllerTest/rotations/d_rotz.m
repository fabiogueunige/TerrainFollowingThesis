function dRz = d_rotz(angle)
    % D_ROTZ Derivative of rotation matrix around Z-axis with respect to angle
    %
    % Input:
    %   angle - Rotation angle [rad]
    %
    % Output:
    %   dRz   - 3x3 derivative matrix (∂Rz/∂angle)
    %
    % Used in Jacobian computations for EKF
    
    dRz = [-sin(angle), -cos(angle),  0;
            cos(angle), -sin(angle),  0;
                     0,           0,  0];
end