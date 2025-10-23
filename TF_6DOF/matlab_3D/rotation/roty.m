function Ry = roty(angle)
    % ROTY Rotation matrix around Y-axis
    %
    % Input:
    %   angle - Rotation angle [rad]
    %
    % Output:
    %   Ry    - 3x3 rotation matrix
    %
    % Convention: Right-hand rule, counterclockwise positive
    
    Ry = [ cos(angle),  0,  sin(angle);
                    0,  1,           0;
          -sin(angle),  0,  cos(angle)];
end