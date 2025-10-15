function [is_intersect] = intersection_check(p_end, p_start, p_int, dir)
    %% Check if intersection point belongs to the plane segment
    % The point must lie between p_start and p_end along the direction dir
    
    % Vector from p_start to p_int
    v_start_int = p_int - p_start;
    
    % Vector from p_start to p_end
    v_start_end = p_end - p_start;
    
    % Project v_start_int onto the direction
    proj_int = dot(v_start_int, dir);
    
    % Project v_start_end onto the direction
    proj_end = dot(v_start_end, dir);
    
    % Check if the intersection point is between start and end
    % The projection of p_int should be between 0 and proj_end
    if proj_int >= 0 && proj_int <= proj_end
        is_intersect = true;
    else
        is_intersect = false;
    end
    
end
    
