function [ymes, h_real, n_new, Rm, command] = SBES_measurament(planes, num_s, pr, wRr, ...
                    k, Rm, command,e_mem) 
    %% Definition
    printDebug('       SBES Measurament:\n');
    check = [command.contact1, command.contact2, command.contact3, command.contact4];

    s = SBES_definition(wRr);

    %% Init
    plane_contact_idx = zeros(1, num_s);
    h_real = 0;
    t_star = zeros(1, num_s);
    p_int = zeros(3, num_s);
    y = zeros(1, num_s);
    command.sensor_fail = 0;

    %% Computation 
    for j = 1:num_s
        for ii = 1 : (k + e_mem - 1)
            % fare condizione if con punti pre e successicìvi
            if dot(s(:,j),planes(ii).n_w) ~= 0
                t_temp = -(dot((pr - planes(ii).point_w),planes(ii).n_w))/(dot(s(:,j),planes(ii).n_w));
                int_p = pr + t_temp*s(:, j);
                if intersection_check(planes(ii + 1).point_w, planes(ii).point_w, int_p, planes(ii).dir_w)
                    if t_temp > 0 && (t_temp < t_star(:,j) || t_star(:,j) <= 0)
                        t_star(:, j) = t_temp;
                        plane_contact_idx(j) = ii;
                    end
                    h_tmp = (planes(ii).n_w'*(pr - planes(ii).point_w))/(norm(planes(ii).n_w));
                    if h_tmp > 0 && (h_tmp < h_real || h_real == 0)
                        h_real = h_tmp;
                    end
                else 
                    if t_star(:,j) <= 0
                        t_star(:,j) = -3; % identification code
                    end
                end
            end   
        end

        % Value check
        if t_star(:,j) <= 0
            Rm(:,:) = Rm(:,:)*150;
            fprintf('Error with t value = %.3f ', t_star(:,j));
            error('Error for sensor %.0f\n',j);
            check(j) = false;
            command.sensor_fail = command.sensor_fail + 1;
        else
            check(j) = true;
        end

        % Intersection point and measure
        p_int(:, j) = pr + t_star(:, j)*s(:, j);
        y(j) = norm(t_star(:, j));
    end

    %% Plane of the 4 mes generation
    [n_new, gen_point] = plane_computation(t_star(:,:), p_int(:,:));

    %% Plot visualization
    if k == 2 || mod(k, 2000) == 0
        m_visualization(pr, gen_point, n_new, num_s, p_int, wRr, k);
    end

    command.contact1 = check(1);
    command.contact2 = check(2);
    command.contact3 = check(3);
    command.contact4 = check(4);

    %% Sending Info's
    ymes = [y(1); y(2); y(3); y(4)];
    printDebug('h reale: %.3f | y1m: %.3f | y2m: %.3f | y3m: %.3f | y4m: %.3f\n', h_real, y(1), y(2), y(3), y(4));

end


