function [ymes, h_real, pr, Rm] = measurament(alpha, beta, pplane, n0, r_s , num_s, ...
                                    input, Ts, pr_old, wRr, k, Rm) 
    %% Definition
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    % angles               
    PHI = 1;        THETA = 2;      PSI = 3;  
    % input
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;  
    
    global DEBUG
    printDebug('       Measurament:\n');

    %% Terrain Definition
    wRt = (rotz(0)*roty(beta)*rotx(alpha))*rotx(pi);
    n = wRt*n0; % in world frame
    if (norm(n) ~= 1)
        n = vector_normalization(n);
        printDebug('n: [%.4f; %.4f; %.4f]\n', n(1), n(2), n(3));
    end
    
    %% Real Robot angle and position update
    % Valid because no noise
    
    % Trasformation update
    w_speed = wRr * [input(I_IND_U); 0; input(I_IND_W)];
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
            error('The line s and the plane n are parallel');
        end
        t_star(:, j) = -(dot((pr - pplane),n))/(dot(s(:,j),n));
        if t_star(:,j) < 0
            Rm(j,j) = Rm(j,j)*50;
            fprintf('Negative value for sensor %.0f\n',j);
            pause(0.5);
        end
        p_int(:, j) = pr + t_star(:, j)*s(:, j);
        y(j) = norm(t_star(:, j));

        % % Second check visibility
        % z_r(:,j) = (wRr)*z_r(:,j);
        % v_p = p_int(:, j) - pr; % world frame
        % visibile = dot(z_r(:,j), v_p) > 0;
        % if ~visibile
        %     fprintf('!! Error in visibility for the sensor %0.f !!', j);
        %     pause(0.05);
        % end
    end

        %% Plot visualization
    if k == 2 || mod(k, 2000) == 0
        m_visualization(pr, pplane, n, num_s, p_int, wRr, k);
    end

    %% Sending Info's
    ymes = [y(1); y(2); y(3); y(4)];
    h_real = (n'*(pr - pplane))/(norm(n));
    printDebug('h reale: %.3f | y1m: %.3f | y2m: %.3f | y3m: %.3f | y4m: %.3f\n', h_real, y(1), y(2), y(3), y(4));
    printDebug('Punto x: %.2f | y: %.2f | z: %.2f\n', pr(1), pr(2), pr(3));
end