%% H - EKF measurement prediction function (update step)
%
% Implements the nonlinear measurement model for the Extended Kalman Filter.
% Predicts sensor measurements (ranges) from the current state estimate,
% assuming sensors measure distance to a planar terrain.
%
% SYNTAX:
%   z = h(x, s, num_s, num_m, n)
%
% INPUTS:
%   x       - Current state vector [3x1]: [h, alpha, beta]
%   s       - Sensor direction vectors in body frame [3 x num_s]
%             Each column is a unit vector pointing along sensor beam
%   num_s   - Number of sensors (typically 4 for SBES)
%   num_m   - Number of measurements (equal to num_s)
%   n       - Terrain normal vector in world frame [3x1]
%
% OUTPUTS:
%   z       - Predicted measurements [num_m x 1]
%             Each element is predicted range from sensor to terrain [m]
%
% MEASUREMENT MODEL:
%   For each sensor i:
%     z_i = -h / (n' * s_i)
%
%   where:
%     h   = altitude above terrain (perpendicular distance)
%     n   = terrain normal vector (unit vector perpendicular to plane)
%     s_i = sensor direction vector (unit vector along beam)
%
% GEOMETRY:
%   The measurement model assumes a planar terrain described by:
%     n' * (p - p0) = 0
%
%   For a sensor at position p_robot with direction s_i, the intersection
%   distance d_i satisfies:
%     n' * (p_robot + d_i*s_i - p0) = 0
%
%   Solving for d_i:
%     d_i = -h / (n' * s_i)
%
%   where h = n' * (p_robot - p0) is the signed altitude
%
% ERROR HANDLING:
%   Throws error if sensor beam is parallel to terrain (n' * s_i = 0)
%   This condition should be prevented by state machine logic
%
% NOTES:
%   - Assumes all sensors are at robot origin (no lever arm)
%   - Negative altitude means robot below terrain plane
%   - Measurements always positive (physical range)
%
% See also: f, jacobian_h, jacobian_f, SBES_measurament, doc/EKF_ALGORITHM.md

function z = h(x, s, num_s, num_m, n)
    printDebug('       h output function\n');
    
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;   
    
    %% Measurement Prediction
    % Initialize measurement vector
    z = zeros(num_m, 1);
    
    % Compute predicted range for each sensor
    for j = 1:num_s
        % Check for parallel condition (singularity)
        if (n'*s(:,j)) == 0
            error('n and sensor %.0f are parallel', j);
        end
        
        % Range measurement model: d = -h / (n' * s)
        % Negative sign because altitude h is measured from robot down to terrain
        z(j) = -(x(IND_H)) / (n' * s(:,j));
    end

    % Debug output for monitoring measurements
    printDebug('y1 = %.2f | y2 = %.2f | y3 = %.2f | y4 = %.2f\n', z(1), z(2), z(3), z(4));
end