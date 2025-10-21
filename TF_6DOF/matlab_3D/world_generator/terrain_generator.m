function [planes, current_idx] = terrain_generator(planes, p_robot, vel_w, current_idx, step_length, max_planes, l_ite)
    %% Dynamic terrain generation with circular buffer
    % Generates new plane when robot moves forward
    % Uses circular buffer of size max_planes
    
    delta_limit = pi/4;
    if l_ite < 30
        angle_range = [-pi/6, pi/6];
    else
        angle_range = [-pi/5, pi/5];
    end
    n0 = [0; 0; 1]; 
    rate_of_change = 3;
    min_distance = 15;
    
    %% Check if we need to generate a new plane
    % Calculate distance from robot to furthest plane
    furthest_distance = planes(current_idx).point_w - p_robot;
    
    % Generate new planes until we have at least 20m ahead
    while furthest_distance(1) < min_distance &&  furthest_distance(2) < min_distance   % Keep at least 10m ahead
        % Increment index (circular)
        current_idx = current_idx + 1;
        if current_idx > max_planes
            current_idx = 1;  % Wrap around to beginning
        end
        
        % Get previous plane index
        prev_idx = current_idx - 1;
        if prev_idx < 1
            prev_idx = max_planes;
        end
        
        %% Generate new plane angles
        if mod(current_idx, rate_of_change) == 0
            valid = false;
            while ~valid
                new_alpha = (angle_range(2)-angle_range(1))*rand + angle_range(1);
                new_beta = (angle_range(2)-angle_range(1))*rand + angle_range(1);
                if abs(new_alpha - planes(prev_idx).alpha) <= delta_limit && ...
                   abs(new_beta - planes(prev_idx).beta) <= delta_limit
                    planes(current_idx).alpha = new_alpha;
                    planes(current_idx).beta = new_beta;
                    valid = true;
                end
            end
        else   % small angle changes
            planes(current_idx).alpha = planes(prev_idx).alpha + 0.1 * sin(0.1 * current_idx) + 0.1 * randn;
            planes(current_idx).beta = planes(prev_idx).beta + 0.1 * sin(0.1 * current_idx) + 0.1 * randn;
        end
        
        %% Definition in inertial frame
        wRs = (rotz(0)*roty(planes(current_idx).beta)*rotx(planes(current_idx).alpha))*rotx(pi);
        planes(current_idx).n_w = wRs * n0;
        if (norm(planes(current_idx).n_w) ~= 1)
            planes(current_idx).n_w = vector_normalization(planes(current_idx).n_w);
            printDebug('n: [%.4f; %.4f; %.4f]\n', planes(current_idx).n_w(1), ...
                       planes(current_idx).n_w(2), planes(current_idx).n_w(3));
        end
        
        %% Plane point definition
        % Direction based on velocity projected on plane
        dir_w = vel_w - (vel_w' * planes(current_idx).n_w) * planes(current_idx).n_w;
        if norm(dir_w) > 0
            dir_w = vector_normalization(dir_w);
        else
            dir_w = planes(prev_idx).dir_w;  % Keep previous direction
        end
        planes(current_idx).dir_w = dir_w;
        
        % New point at step_length distance from previous
        planes(current_idx).point_w = planes(prev_idx).point_w + step_length * dir_w;
        
        % Update distance for next iteration
        furthest_distance = planes(current_idx).point_w - p_robot;
    end
end 