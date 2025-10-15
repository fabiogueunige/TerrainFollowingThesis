function [n, p1] = plane_computation(t, p_points)
    %% Compute plane normal from intersection points
    % Only consider points with positive t values
    
    % Filter valid points (t > 0)
    valid_idx = t > 0;
    valid_points = p_points(:, valid_idx);
    num_valid = sum(valid_idx);
    
    % Need at least 3 points to define a plane
    if num_valid < 3
        % Return a default normal vector (pointing up)
        n = [0; 0; 0];
        warning('Not enough valid points to compute plane normal. Using default [0; 0; 1]');
        return;
    end
    
    % Use the first 3 valid points to compute the plane
    p1 = valid_points(:, 1);
    p2 = valid_points(:, 2);
    p3 = valid_points(:, 3);
    
    % Compute two vectors in the plane
    v1 = p2 - p1;
    v2 = p3 - p1;
    
    % Normal is the cross product
    n = cross(v1, v2);
    n = vector_normalization(n);
end