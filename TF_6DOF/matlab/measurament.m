function [ymes, h, pr] = measurament(x, beta, qt, Gamma, Lambda, v, Ts, pr_old) 
    % terrain construction
    mt = tan(beta);
    m1_g = tan((3*pi/2 + Gamma + x(3)));  % also added pitch infos
    m2_l = tan((3*pi/2 + Lambda + x(3))); % also added pitch infos

    % robot position 
    dx = v(1)*cos(x(3)) + v(2)*sin(x(3));
    dz = v(1)*sin(x(3)) - v(2)*cos(x(3));
    pr = pr_old + [dx*Ts, dz*Ts]'; 
    q1 = pr(2) - m1_g*pr(1);
    q2 = pr(2) - m2_l*pr(1);

    % z computation
    z_r = [0, -1]';
    Ry_2D = [cos(x(3)), -sin(x(3));
          sin(x(3)),  cos(x(3))];
    z_r = Ry_2D*z_r;

    % computation for y1 
    xc1 = (q1 - qt) / (mt - m1_g);
    zc1 = mt * xc1 + qt;
    y1m = norm([pr(1) - xc1; pr(2) - zc1]);
    % visibility
    v_p1 = [xc1 - pr(1); zc1 - pr(2)];
    visibile_y1 = dot(z_r, v_p1) > 0;
    if ~visibile_y1
        fprintf('Errore in visibilità y1 \n');
        % y1m = 500000;
    end

    % Computation for y2
    xc2 = (q2 - qt) / (mt - m2_l);
    zc2 = mt * xc2 + qt;
    y2m = norm([pr(1) - xc2; pr(2) - zc2]);
    % visibility
    v_p2 = [xc2 - pr(1); zc2 - pr(2)];
    visibile_y2 = dot(z_r, v_p2) > 0;
    if ~visibile_y2
        fprintf('Errore in visibilità y2 \n');
        % y2m = 500000;
    end
    
    ymes = [y1m; y2m];
    % value of h to obtain
    h = (abs(mt*pr(1) - pr(2) + qt))/ (sqrt(mt^2 + 1));

    fprintf('h reale: %.2f m | y1m: %.2f | y2m: %.3f m/s \n', h, y1m, y2m);
    fprintf('Punto x: %.2f m | Punto y: %.2f\n', pr(1), pr(2));

% %%  plot
% % Intervallo di x per il plot
% x_min = min([pr(1), xc1, xc2]) - 5;
% x_max = max([pr(1), xc1, xc2]) + 5;
% x_values = linspace(x_min, x_max, 100);
% 
% % Equazioni delle rette
% y_mt = mt * x_values + qt;
% y_m1g = m1_g * x_values + q1; 
% y_m2l = m2_l * x_values + q2;
% % y_mp = mp * x_values + qp;
% 
% % plot per h
% m_perp = -1/mt; 
% b_perp = pr(2) - m_perp * pr(1); 
% x_hm = (b_perp - qt) / (mt - m_perp);
% y_hm = mt * x_hm + qt;
% 
% hold on;
% grid on;
% box on;
% 
% plot(x_values, y_mt, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Retta terrain (mt)');
% plot(x_values, y_m1g, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Retta y1 (m1_g)');
% plot(x_values, y_m2l, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Retta y2 (m2_l)');
% % plot(x_values, y_mp, 'p-', 'LineWidth', 1.5, 'DisplayName', 'Retta pitch (mp)');
% 
% % Plot dei punti noti
% plot(pr(1), pr(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Punto Pr (0,0)');
% plot(xc1, zc1, 'cx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Intersezione xc1,zc1');
% plot(xc2, zc2, 'yx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Intersezione xc2,zc2');
% 
% plot(x_hm, y_hm, 'kd', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Punto hm sul Terreno'); % Punto hm
% plot([pr(1), x_hm], [pr(2), y_hm], 'k:', 'LineWidth', 1, 'DisplayName', 'Segmento hm'); % Retta congiungente
% 
% % Aggiungi etichette e legenda
% xlabel('Asse X');
% ylabel('Asse Y');
% title('Plot delle Rette nel Piano');
% legend('Location', 'best');
% axis equal; % Assicura che le unità sugli assi siano uguali per evitare distorsioni angolari
% ylim_auto = [min([min(y_mt), min(y_m1g), min(y_m2l), qt, zc1, zc2])-5, max([max(y_mt), max(y_m1g), max(y_m2l), qt, zc1, zc2])+5];
% if isfinite(ylim_auto(1)) && isfinite(ylim_auto(2))
%     ylim(ylim_auto);
% end
% hold off;


end

function oRr = rotation(theta)
    roll = 0;
    yaw = 0;
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw), 0;
          0, 0, 1];
    Ry = [cos(theta), 0, sin(theta);
          0, 1, 0;
          -sin(theta), 0, cos(theta)];
    Rx = [1 , 0, 0
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];
    oRr = Rz*Ry*Rx;    
end