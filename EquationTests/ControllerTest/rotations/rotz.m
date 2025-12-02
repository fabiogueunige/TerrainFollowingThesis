function Rz = rotz(angle)
    % ROTZ Rotation matrix around Z-axis
    %
    % Input:
    %   angle - Rotation angle [rad]
    %
    % Output:
    %   Rz    - 3x3 rotation matrix
    %
    % Convention: Right-hand rule, counterclockwise positive
    
    Rz = [cos(angle), -sin(angle),  0;
          sin(angle),  cos(angle),  0;
                   0,           0,  1];
end