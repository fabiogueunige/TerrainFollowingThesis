function m_visualization(pr, pplane, n, num_s, p_int, wRr, j)
    global DEBUG    
    figure;
    hold on;
    grid on;
    box on;
    % for plot
    p_proj = pr - dot(n, pr - pplane) * n; % h projection
    colors = lines(num_s);
    
    % Plane plot
    [xp, yp] = meshgrid(-17:0.5:17, -17:0.5:17);
    if abs(n(3)) > 1e-6
        printDebug('\n!!!Attivazione if disegno del piano!!!\n');
        zp = (-n(1)*(xp-pplane(1)) - n(2)*(yp-pplane(2)))/n(3) + pplane(3);
        surf(xp, yp, zp, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'DisplayName', 'Piano');
    end
    
    % Robot frame
    t_tmp = 3;
    x_dir = wRr * [1; 0; 0];
    y_dir = wRr * [0; 1; 0];
    z_dir = wRr * [0; 0; 1];
    p_x_dir = pr + t_tmp * x_dir;
    p_y_dir = pr + t_tmp * y_dir;
    p_z_dir = pr + t_tmp * z_dir;
    % Robot frame plot
    plot3([pr(1), p_x_dir(1)], [pr(2), p_x_dir(2)], [pr(3), p_x_dir(3)], 'r--', 'LineWidth', 2, 'DisplayName', 'Asse X rob');
    plot3([pr(1), p_y_dir(1)], [pr(2), p_y_dir(2)], [pr(3), p_y_dir(3)], 'g--', 'LineWidth', 2, 'DisplayName', 'Asse Y rob');
    plot3([pr(1), p_z_dir(1)], [pr(2), p_z_dir(2)], [pr(3), p_z_dir(3)], 'b--', 'LineWidth', 2, 'DisplayName', 'Asse Z rob');
    
    % Sensor plot
    for k = 1:num_s
        plot3([pr(1), p_int(1, k)], [pr(2), p_int(2, k)], [pr(3), p_int(3, k)], ...
              'LineWidth', 2, 'Color', colors(k, :), ...
              'DisplayName', sprintf('y%d', k));
        plot3(p_int(1, k), p_int(2, k), p_int(3, k), 'x', 'MarkerFaceColor', colors(k, :), ...
                'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', sprintf('Intersezione y%d', k));
    end
    
    % Punto iniziale
    plot3(pr(1), pr(2), pr(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Punto iniziale');
    plot3([pr(1), p_proj(1)], [pr(2), p_proj(2)], [pr(3), p_proj(3)], ...
          'k--', 'LineWidth', 2, 'DisplayName', 'Altitude h)');
    plot3(p_proj(1), p_proj(2), p_proj(3), 'kd', 'MarkerSize', 6, 'LineWidth', 2, 'DisplayName', 'Point of proj h new');
      
    % Normal to the plane
    quiver3(p_proj(1), p_proj(2), p_proj(3), n(1), n(2), n(3), 'r', 'LineWidth', 3, 'MaxHeadSize', 5);
    
    xlabel('Asse X');
    ylabel('Asse Y');
    zlabel('Asse Z');
    
    set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
    
    title('AUV Situation with Sensors %d' , j/1000);
    % legend('Location', 'best');
    axis equal; 
    hold off;
end