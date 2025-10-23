function Rx = rotx(angle)
    % ROTX Rotation matrix around X-axis
    %
    % Input:
    %   angle - Rotation angle [rad]
    %
    % Output:
    %   Rx    - 3x3 rotation matrix
    %
    % Convention: Right-hand rule, counterclockwise positive
    
    Rx = [1,          0,           0;
          0,  cos(angle), -sin(angle);
          0,  sin(angle),  cos(angle)];
end