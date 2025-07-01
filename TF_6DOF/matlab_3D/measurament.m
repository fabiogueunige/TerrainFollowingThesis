function [ymes, h_real, pr] = measurament(alpha, beta, pplane, n0, Gamma, Lambda, Eta, Zeta, num_s, ...
                                    v, Ts, pr_old, ph_o, th_o) 
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
    wRt = (rotz(0)*roty(beta)*rotx(alpha))*(rotx(pi))';
    n = wRt*n0; % in world frame
    if (norm(n) ~= 1)
        fprintf('ALERT: norm n is not 1\n');
        fprintf('n: [%.4f; %.4f; %.4f]\n', n(1), n(2), n(3));
    end

    %% AUV Parameter
    psi = 0;
    
    %% Real Robot angle and position update
    % Valid because no noise
    phm = ph_o + v(I_IND_P)*Ts;
    thm = th_o + v(I_IND_Q)*Ts;
    p_gyr = v(I_IND_P);
    q_gyr = v(I_IND_Q);
    
    % Trasformation update
    wRr = rotz(psi)*roty(thm)*rotx(phm);
    w_speed = wRr * [v(1); 0; v(2)];
    pr = pr_old + w_speed*Ts;
    
    %% Sensor Definition
    % Calcolando quello reale del robot per avere misure affidabili
    s = zeros(3, num_s);
    s(:, 1) = [-sin(Gamma + thm), 0, cos(Gamma + thm)]';
    s(:, 2) = [-sin(Lambda + thm), 0, cos(Lambda + thm)]';
    s(:, 3) = [0, -sin(Eta + phm), cos(Eta + phm)]';
    s(:, 4) = [0, -sin(Zeta + phm), cos(Zeta + phm)]';
    % check of consistency for the sensors
    for j = 1:num_s
        if (norm(s(:,j)) ~= 1)
            fprintf('ERROR: norm sensor k = %.0f is not 1\n', j);
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
        p_int(:, j) = pr + t_star(:, j)*s(:, j);
        y(j) = norm(t_star(:, j));
    end
    
    % Check of visibility
    z_r = [0, 0, 1]'; 
    z_r = (wRr)'*z_r; % Adjust z_r based on angles
    for j = 1:num_s
        v_p = p_int(:, j) - pr;
        visibile = dot(z_r, v_p) > 0;
        if ~visibile
            error('Error in visibility for the sensor %0.f', j);
        end
    end

    %% Sending Info's
    ymes = [y(1); y(2); y(3); y(4); phm; thm; p_gyr; q_gyr];
    h_real = (n'*(pr - pplane))/(norm(n));
    fprintf('h reale: %.2f | y1m: %.2f | y2m: %.3f | y3m: %.2f | y4m: %.3f\n', h_real, y(1), y(2), y(3), y(4));
    fprintf('phi mes new: %.2f | p_gyr: %.2f \n', rad2deg(phm), p_gyr);
    fprintf('theta mes new: %.2f | q_gyr: %.2f \n', rad2deg(thm), q_gyr);
    fprintf('Punto x: %.2f | y: %.2f | z: %.2f\n', pr(1), pr(2), pr(3));
end