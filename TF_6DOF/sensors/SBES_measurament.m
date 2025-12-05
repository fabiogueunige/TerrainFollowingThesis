function [ymes, h_real, n_new, Rm, command, a_new, b_new] = SBES_measurament(planes, num_s, pr, wRr, ...
                    t_idx, Rm, command, ite) 
    %% Definition
    printDebug('       SBES Measurament:\n');

    s = SBES_definition(wRr);

    %% Init
    plane_contact_idx = zeros(1, num_s);
    h_real = inf;  % Use inf for minimum search
    t_star = zeros(1, num_s);
    p_int = zeros(3, num_s);
    y = zeros(1, num_s);
    command.sensor_fail = 0;
    
    %% Search range - only check recently generated planes
    % Search backwards from current index for efficiency
    % search_range = 200;  % Number of planes to check
    max_planes = length(planes);

    %% noise sensors
    v_sbes = mvnrnd(zeros(num_s,1), Rm)'; 
    
    
    %% Computation - for each sensor
    for j = 1:num_s
        t_star(j) = inf;  % Initialize with inf for minimum search
        
        % Search through planes in circular buffer
        for ii = 1:max_planes
            
            % Check if sensor ray intersects with plane
            ray_plane_dot = dot(s(:,j), planes(ii).n_w);
            
            if abs(ray_plane_dot) > 1e-10  % Not parallel (with tolerance)
                % Calculate intersection parameter t
                t_temp = -dot((pr - planes(ii).point_w), planes(ii).n_w) / ray_plane_dot;
                
                % Only consider positive t
                if t_temp > 0
                    % Calculate intersection point
                    int_p = pr + t_temp * s(:, j);
                    
                    % Get next plane index for boundary check
                    next_ii = ii + 1;
                    if next_ii > max_planes
                        next_ii = 1;
                    end
                    
                    % Check if intersection is within plane segment
                    if intersection_check(planes(next_ii).point_w, planes(ii).point_w, int_p, planes(ii).dir_w)
                        % Keep the closest intersection
                        if t_temp < t_star(j)
                            t_star(j) = t_temp;
                            plane_contact_idx(j) = ii;
                        end
                    end
                end
            end
        end

        %% Value check
        if isinf(t_star(j)) || t_star(j) <= 0
            Rm(:,:) = Rm(:,:) * 150;
            printDebug('Error: No valid intersection for sensor %d at iteration %d\n', j, ite);
            command.contact(j) = false;
            command.sensor_fail = command.sensor_fail + 1;
        else
            command.contact(j) = true;
            % Calculate final intersection point and measurement
            p_int(:, j) = pr + t_star(j) * s(:, j);
            y(j) = t_star(j);
        end
    end
    
    % Calculate h_real only from contacted planes (most efficient and accurate)
    h_real = inf;
    for j = 1:num_s
        if command.contact(j) && plane_contact_idx(j) > 0
            % Calculate altitude for this contacted plane
            ii = plane_contact_idx(j);
            h_tmp = abs(planes(ii).n_w' * (pr - planes(ii).point_w)) / norm(planes(ii).n_w);
            if h_tmp < h_real
                h_real = h_tmp;
            end
        end
    end
    
    % Handle case where no valid altitude was found
    if isinf(h_real)
        h_real = 100;
    end

    %% Plane of the 4 measurements generation
    [n_new, gen_point, a_new, b_new] = plane_computation(t_star, p_int);
    if norm(n_new) ~= 1 && n_new(3) == 10
        n_new = planes(plane_contact_idx(n_new(1))).n_w;  % Use normal of first contacted plane
        gen_point = planes(plane_contact_idx(n_new(1))).point_w;
        a_new = planes(plane_contact_idx(n_new(1))).alpha;
        b_new = planes(plane_contact_idx(n_new(1))).beta;
    end

    %% Plot visualization
    if mod(ite, 2000) == 0 || ite == 30
        m_visualization(pr, gen_point, n_new, num_s, p_int, wRr, ite, planes, plane_contact_idx, command.contact);
    end

    %% Sending Info's
    ymes = [y(1); y(2); y(3); y(4)];

    %% Additive noise
    y_mes = ymes + v_sbes;
    
    printDebug('h real: %.3f | y1: %.3f | y2: %.3f | y3: %.3f | y4: %.3f\n', ...
               h_real, y(1), y(2), y(3), y(4));

end