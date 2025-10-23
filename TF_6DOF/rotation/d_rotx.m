function dRx = d_rotx(angle)
    % D_ROTX Derivative of rotation matrix around X-axis with respect to angle
    %
    % Input:
    %   angle - Rotation angle [rad]
    %
    % Output:
    %   dRx   - 3x3 derivative matrix (∂Rx/∂angle)
    %
    % Used in Jacobian computations for EKF
    
    dRx = [0,           0,            0;
           0, -sin(angle),  -cos(angle);
           0,  cos(angle),  -sin(angle)];
end