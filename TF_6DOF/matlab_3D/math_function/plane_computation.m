function [n, p1] = plane_computation(t, p_points)
    %% Compute plane normal from intersection points
    % Only consider points with positive t values
    
    % Filter valid points (t > 0)
    valid_idx = t > 0;
    valid_t = t(valid_idx);
    valid_points = p_points(:, valid_idx);
    num_valid = sum(valid_idx);
    
    % Need at least 3 points to define a plane
    if num_valid < 3
        % Not enough points: encode which sensors are valid in the normal
        
        % Get indices of valid sensors
        s_index = find(valid_idx);
        
        if num_valid == 0
            % No valid sensors at all
            n = [0; 0; 0];
            p1 = p_points(:,1);
            warning('No valid points to compute plane normal. All sensors failed.');
        else
            n = [s_index(1); 0; 10];
            
            % Use the first valid point as reference
            p1 = valid_points(:, 1);
        end
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