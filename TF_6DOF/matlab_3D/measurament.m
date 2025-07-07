function [ymes, h_real, pr] = measurament(alpha, beta, pplane, n0, r_s , num_s, ...
                                    v, Ts, pr_old, ph_o, th_o, k, psi, z_r) 
    %% Definitions
    IND_H = 1;                  I_IND_U = 1;
    ALPHA = 2;                  I_IND_W = 2;
    BETA = 3;
    PHI = 4;      M_PHI = 5;
    THETA = 5;    M_THETA = 6;
    IND_P = 6;    M_IND_P = 7;  I_IND_P = 3;
    IND_Q = 7;    M_IND_Q = 8;  I_IND_Q = 4;  
    fprintf('       Measurament:\n');

    %% Terrain Definition
    wRt = (rotz(0)*roty(beta)*rotx(alpha))*rotx(pi);
    n = wRt*n0; % in world frame
    if (norm(n) ~= 1)
        n = vector_normalization(n);
        fprintf('n: [%.4f; %.4f; %.4f]\n', n(1), n(2), n(3));
    end
    
    %% Real Robot angle and position update
    % Valid because no noise
    phm = ph_o + v(I_IND_P)*Ts;
    thm = th_o + v(I_IND_Q)*Ts;
    p_gyr = v(I_IND_P);
    q_gyr = v(I_IND_Q);
    
    % Trasformation update
    wRr = rotz(psi)*roty(thm)*rotx(phm);
    w_speed = wRr * [v(I_IND_U); 0; v(I_IND_W)];
    pr = pr_old + w_speed*Ts;
    
    %% Sensor Definition
    % Calcolando quello reale del robot per avere misure affidabili
    s = zeros(3,num_s);
    % check of consistency for the sensors
    for j = 1:num_s
        s(:,j) = wRr*r_s(:,j);
        if (norm(s(:,j)) ~= 1)
            fprintf('!! ALERT !!: norm Measured sensor %.0f has been normalized\n', j);
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
            error('Negative value for sensor %.0f\n',j);
        end
        p_int(:, j) = pr + t_star(:, j)*s(:, j);
        y(j) = norm(t_star(:, j));
        
        % Second check visibility
        z_r(:,j) = (wRr)*z_r(:,j);
        v_p = p_int(:, j) - pr; % world frame
        visibile = dot(z_r(:,j), v_p) > 0;
        if ~visibile
            error('!! Error in visibility for the sensor %0.f !!', j);
        end
    end

        %% Plot visualization
    if k == 2 || mod(k, 5000) == 0
        m_visualization(pr, pplane, n, num_s, p_int, wRr, k);
    end

    %% Sending Info's
    ymes = [y(1); y(2); y(3); y(4); phm; thm]; %  p_gyr; q_gyr
    h_real = (n'*(pr - pplane))/(norm(n));
    fprintf('h reale: %.3f | y1m: %.3f | y2m: %.3f | y3m: %.3f | y4m: %.3f\n', h_real, y(1), y(2), y(3), y(4));
    fprintf('phi mes new: %.2f | p_gyr: %.2f \n', rad2deg(phm), p_gyr);
    fprintf('theta mes new: %.2f | q_gyr: %.2f \n', rad2deg(thm), q_gyr);
    fprintf('Punto x: %.2f | y: %.2f | z: %.2f\n', pr(1), pr(2), pr(3));
end