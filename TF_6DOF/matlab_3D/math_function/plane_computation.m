%% PLANE_COMPUTATION - Compute plane normal from sensor intersection points
%
% Estimates terrain plane orientation from multiple sensor-terrain intersection
% points. Handles sensor failures gracefully by working with partial data.
%
% SYNTAX:
%   [n, p1] = plane_computation(t, p_points)
%
% INPUTS:
%   t        - Range measurements [num_sensors x 1]
%              Positive values indicate valid measurements
%   p_points - Intersection points in world frame [3 x num_sensors]
%              Each column corresponds to a sensor beam intersection
%
% OUTPUTS:
%   n  - Estimated terrain normal vector [3x1] (unit vector)
%   p1 - Reference point on the plane [3x1]
%
% ALGORITHM:
%   1. Filter valid measurements (t > 0)
%   2. If ≥3 valid points: compute normal via cross product
%      - Use first 3 valid points: p1, p2, p3
%      - Compute vectors: v1 = p2-p1, v2 = p3-p1
%      - Normal: n = (v1 × v2) / ||v1 × v2||
%   3. If <3 valid points: encode failure mode in normal
%      - Stores valid sensor indices for diagnostic purposes
%
% FAILURE MODES:
%   - 0 valid sensors: n = [0; 0; 0], warning issued
%   - 1-2 valid sensors: n = [sensor_index; 0; 10], encoded diagnostic
%   - ≥3 valid sensors: normal computed from geometry
%
% PLANE EQUATION:
%   The plane is defined by: n' * (p - p1) = 0
%   where p1 is a point on the plane and n is the normal vector
%
% NOTES:
%   - Assumes coplanar intersection points (valid for local terrain patches)
%   - Uses first 3 valid points (not least-squares fit)
%   - Normal direction depends on point ordering (right-hand rule)
%   - For 4 SBES sensors, typically all 4 are valid in normal operation
%
% ERROR HANDLING:
%   - Warns if no valid sensors
%   - Returns diagnostic codes for partial sensor failures
%   - Normalizes result to unit vector
%
% See also: SBES_measurament, reference_correction, vector_normalization

function [n, p1] = plane_computation(t, p_points)
    %% Filter Valid Measurements
    % Only consider points with positive t values (valid range measurements)
    valid_idx = t > 0;
    valid_t = t(valid_idx);
    valid_points = p_points(:, valid_idx);
    num_valid = sum(valid_idx);
    
    %% Check if Enough Points Available
    % Need at least 3 points to define a plane uniquely
    if num_valid < 3
        % Insufficient points: encode failure mode in normal vector
        
        % Get indices of valid sensors for diagnostics
        s_index = find(valid_idx);
        
        if num_valid == 0
            % Complete sensor failure
            n = [0; 0; 0];
            p1 = p_points(:,1);
            warning('No valid points to compute plane normal. All sensors failed.');
        else
            % Partial failure: encode first valid sensor index
            % Format: [sensor_id; 0; 10] to distinguish from geometric normal
            n = [s_index(1); 0; 10];
            
            % Use the first valid point as reference
            p1 = valid_points(:, 1);
        end
        return;
    end
    
    %% Compute Plane from Three Points
    % Select first three valid points
    p1 = valid_points(:, 1);
    p2 = valid_points(:, 2);
    p3 = valid_points(:, 3);
    
    % Construct two vectors lying in the plane
    v1 = p2 - p1;
    v2 = p3 - p1;
    
    % Normal vector is perpendicular to both v1 and v2
    % Direction determined by right-hand rule: n = v1 × v2
    n = cross(v1, v2);
    
    % Normalize to unit vector
    n = vector_normalization(n);
end