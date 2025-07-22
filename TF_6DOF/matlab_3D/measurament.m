function [ymes, h_real, pr, Rm, command] = measurament(alpha, beta, pplane, n0, r_s , num_s, ...
                                    input, Ts, pr_old, wRr, k, Rm, command) 
    %% Definition
    global SURGE; global SWAY; global HEAVE;
    printDebug('       Measurament:\n');
    check = [command.contact1, command.contact2, command.contact3, command.contact4];

    %% Terrain Definition
    wRs = (rotz(0)*roty(beta)*rotx(alpha))*rotx(pi);
    n = wRs*n0; % in world frame    
    if (norm(n) ~= 1)
        n = vector_normalization(n);
        printDebug('n: [%.4f; %.4f; %.4f]\n', n(1), n(2), n(3));
    end
    
    %% Real Robot angle and position update
    % Trasformation update
    w_speed = wRr * input(SURGE:HEAVE);
    pr = pr_old + w_speed*Ts;
    
    %% Sensor Definition
    % Calcolando quello reale del robot per avere misure affidabili
    s = zeros(3,num_s);
    % check of consistency for the sensors
    for j = 1:num_s
        s(:,j) = wRr*r_s(:,j);
        if (norm(s(:,j)) ~= 1)
            printDebug('!! ALERT !!: norm Measured sensor %.0f has been normalized\n', j);
            s(:,j) = vector_normalization(s(:,j));
        end
    end

    %% Sensor Values
    t_star = zeros(1, num_s);
    p_int = zeros(3, num_s);
    y = zeros(1, num_s);
    for j = 1:num_s
        if dot(s(:,j),n) == 0
            error('The line s and the plane n are parallel in measurament');
        end
        t_star(:, j) = -(dot((pr - pplane),n))/(dot(s(:,j),n));
        if t_star(:,j) < 0
            Rm(:,:) = Rm(:,:)*150;
            fprintf('Negative value for sensor %.0f\n',j);
            check(j) = false;
        else
            check(j) = true;
        end
        p_int(:, j) = pr + t_star(:, j)*s(:, j);
        y(j) = norm(t_star(:, j));
    end

    %% Plot visualization
    if k == 2 || mod(k, 2000) == 0
        m_visualization(pr, pplane, n, num_s, p_int, wRr, k);
    end

    command.contact1 = check(1);
    command.contact2 = check(2);
    command.contact3 = check(3);
    command.contact4 = check(4);

    %% Sending Info's
    ymes = [y(1); y(2); y(3); y(4)];
    h_real = (n'*(pr - pplane))/(norm(n));
    printDebug('h reale: %.3f | y1m: %.3f | y2m: %.3f | y3m: %.3f | y4m: %.3f\n', h_real, y(1), y(2), y(3), y(4));
    printDebug('Punto x: %.2f | y: %.2f | z: %.2f\n', pr(1), pr(2), pr(3));
end