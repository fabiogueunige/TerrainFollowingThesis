function plot_results(time, N, h_ref, x_true, x_est, rob_rot, clean_rot, goal, u, ...
                      n_est, n_mes, wRr, prob, n_dim, d_dim, i_dim)
    %% PLOT_RESULTS - Visualizzazione completa dei risultati della simulazione
    % 
    % Inputs:
    %   time: vettore temporale
    %   N: numero di iterazioni
    %   h_ref: altitudine di riferimento
    %   x_true: stati veri
    %   x_est: stati stimati
    %   rob_rot: angoli del robot con rumore
    %   clean_rot: angoli del robot senza rumore
    %   goal: struttura con i goal
    %   u: input di controllo
    %   n_est: normali stimate
    %   n_mes: normali misurate
    %   wRr: matrici di rotazione del robot
    %   prob: posizioni del robot
    %   n_dim, d_dim, i_dim: dimensioni
    
    global HEAVE;
    
    %% States
    fprintf('Plotting states...\n');
    ttl = {'altitude', 'alpha', 'beta'};
    for i = 1:n_dim
        figure('Name', sprintf('State: %s', ttl{i}));
        if (i == 1)
            plot(time, h_ref(:), 'b', 'LineWidth', 3.5, 'DisplayName', 'Desired')
            hold on;
            plot(time, x_true(i,:), 'r', 'DisplayName', 'True');
            plot(time, x_est(i,:), 'g', 'DisplayName', 'Estimated');
        else
            plot(time, rad2deg(x_true(i,:)), 'b', 'LineWidth', 3.5, 'DisplayName', 'True (desired) Terrain');
            hold on;
            plot(time, rad2deg(rob_rot(i-1,:)), 'r', 'DisplayName', 'Rob angle');
            plot(time, rad2deg(x_est(i,:)), 'g', 'DisplayName', 'Estimated');
        end
        xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
        legend; grid on;
        title(ttl{i})
        hold off;
    end
    
    %% Robot angles
    fprintf('Plotting robot angles...\n');
    ttl = {'roll', 'pitch', 'yaw'};
    for i = 1:d_dim
        figure('Name', sprintf('Robot angle: %s', ttl{i}));
        plot(time, rad2deg(clean_rot(i,:)), 'r', 'DisplayName', 'True no noise');
        hold on;
        if i == 1
            plot(time, rad2deg([goal.roll]), 'b', 'DisplayName', 'Goal');
        end
        if i == 2
            plot(time, rad2deg([goal.pitch]), 'b', 'DisplayName', 'Goal');
        end
        plot(time, rad2deg(rob_rot(i,:)), 'g', 'DisplayName', 'Rotation with Noise');
        xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
        legend; grid on;
        title(ttl{i})
        hold off;
    end
    
    %% Inputs
    fprintf('Plotting control inputs...\n');
    ttl = {'u input', 'v input', 'w input', 'p input', 'q input', 'r input'};
    for i = 1:i_dim
        figure('Name', sprintf('Input: %s', ttl{i}));
        if i <= HEAVE
            plot(time, u(i,:), 'b', 'DisplayName', 'u');
        else
            plot(time, rad2deg(u(i,:)), 'b', 'DisplayName', ttl{i});
        end
        hold on
        xlabel('Time [s]'); ylabel('Space [m]');
        grid on;
        title(ttl{i})
        hold off;
    end

    %% Normal z-sign analysis
    figure('Name', 'z-sign'); 
    plot(time, n_est(3,:), 'r', 'LineWidth', 1.5, 'DisplayName', 'z-sign estimated');
    hold on;
    plot(time, n_mes(3,:), 'g', 'LineWidth', 1.5, 'DisplayName', 'z-sign measured (t)');
    xlabel('Tempo [s]');
    ylabel('z value');
    title('Sign of the z of the normal plane');
    legend('Location', 'best');
    hold off;
    
    %% Normal vectors analysis: parallelism and z-component alignment
    fprintf('Analyzing normal vectors...\n');
    % Verifica sia il parallelismo (angolo tra normali) che la concordanza della componente z
    % Se z-component è simile e angolo piccolo -> robot e terreno hanno stessa inclinazione
    
    % Preallocazione solo per parallelismo
    aa12 = zeros(1,N);  % Angolo tra n_est e n_mes
    aa13 = zeros(1,N);  % Angolo tra n_est e z_robot
    aa23 = zeros(1,N);  % Angolo tra n_mes e z_robot

    for i = 1:N
        % Angoli tra normali (parallelismo, indipendentemente dal segno)
        aa12(i) = acosd(abs(dot(n_est(:,i)', n_mes(:,i)'))); % tra n estimato e n da misure
        aa13(i) = acosd(abs(dot(n_est(:,i)', wRr(:,3,i)'))); % tra n estimato e n rob
        aa23(i) = acosd(abs(dot(n_mes(:,i)', wRr(:,3,i)'))); % tra n da misure e n rob
    end
    
    % Plot: Angoli tra normali (parallelismo)
    figure('Name', 'Parallelismo tra normali'); 
    hold on; grid on;
    plot(time, aa12, 'r', 'LineWidth', 1.5, 'DisplayName', 'estimato Vs misure');
    plot(time, aa13, 'g', 'LineWidth', 1.5, 'DisplayName', 'estimato Vs robot');
    plot(time, aa23, 'b', 'LineWidth', 1.5, 'DisplayName', 'misure Vs robot');
    yline(5, 'k--', 'LineWidth', 1, 'DisplayName', 'Soglia 5°');
    scatter(time(aa12 > 5), aa12(aa12 > 5), 30, 'r', 'filled', 'DisplayName', 'Non paralleli (est-mis)');
    scatter(time(aa13 > 5), aa13(aa13 > 5), 30, 'g', 'filled', 'DisplayName', 'Non paralleli (est-rob)');
    scatter(time(aa23 > 5), aa23(aa23 > 5), 30, 'b', 'filled', 'DisplayName', 'Non paralleli (mis-rob)');
    xlabel('Tempo [s]');
    ylabel('Angolo tra normali [°]');
    title('Parallelismo tra le normali dei piani nel tempo');
    legend('Location', 'best');
    hold off;
    
    %% Statistiche finali (solo parallelismo)
    fprintf('\n=== ANALISI NORMALI: STATISTICHE (PARALLELISMO) ===\n');

    % Campioni validi (senza NaN sugli angoli)
    valid_12 = ~isnan(aa12);
    valid_13 = ~isnan(aa13);
    valid_23 = ~isnan(aa23);

    fprintf('\nCampioni validi (senza NaN):\n');
    fprintf('  Est vs Mes: %d / %d (%.1f%%)\n', sum(valid_12), N, 100*sum(valid_12)/N);
    fprintf('  Est vs Rob: %d / %d (%.1f%%)\n', sum(valid_13), N, 100*sum(valid_13)/N);
    fprintf('  Mes vs Rob: %d / %d (%.1f%%)\n', sum(valid_23), N, 100*sum(valid_23)/N);

    fprintf('\nAngolo medio (solo campioni validi):\n');
    if any(valid_12), fprintf('  Est vs Mes: %.2f° (max: %.2f°, min: %.2f°)\n', mean(aa12(valid_12)), max(aa12(valid_12)), min(aa12(valid_12))); else, fprintf('  Est vs Mes: N/A\n'); end
    if any(valid_13), fprintf('  Est vs Rob: %.2f° (max: %.2f°, min: %.2f°)\n', mean(aa13(valid_13)), max(aa13(valid_13)), min(aa13(valid_13))); else, fprintf('  Est vs Rob: N/A\n'); end
    if any(valid_23), fprintf('  Mes vs Rob: %.2f° (max: %.2f°, min: %.2f°)\n', mean(aa23(valid_23)), max(aa23(valid_23)), min(aa23(valid_23))); else, fprintf('  Mes vs Rob: N/A\n'); end

    fprintf('\nPercentuale tempo PARALLELI (angolo < 5°):\n');
    if any(valid_12), good_12 = sum(aa12 < 5 & valid_12); fprintf('  Est vs Mes: %.1f%% (%d / %d campioni validi)\n', 100*good_12/sum(valid_12), good_12, sum(valid_12)); else, fprintf('  Est vs Mes: N/A\n'); end
    if any(valid_13), good_13 = sum(aa13 < 5 & valid_13); fprintf('  Est vs Rob: %.1f%% (%d / %d campioni validi)\n', 100*good_13/sum(valid_13), good_13, sum(valid_13)); else, fprintf('  Est vs Rob: N/A\n'); end
    if any(valid_23), good_23 = sum(aa23 < 5 & valid_23); fprintf('  Mes vs Rob: %.1f%% (%d / %d campioni validi)\n', 100*good_23/sum(valid_23), good_23, sum(valid_23)); else, fprintf('  Mes vs Rob: N/A\n'); end

    fprintf('\n⚠️  ATTENZIONE - Problemi rilevati:\n');
    % NaN nelle normali di input
    nan_mes = any(isnan(n_mes));
    nan_est = any(isnan(n_est));
    if any(nan_mes)
        fprintf('  - n_mes contiene NaN in %d campioni (%.1f%%) -> possibili fallimenti SBES\n', sum(nan_mes), 100*sum(nan_mes)/N);
    end
    if any(nan_est)
        fprintf('  - n_est contiene NaN in %d campioni (%.1f%%) -> problemi nella stima EKF\n', sum(nan_est), 100*sum(nan_est)/N);
    end
    not_parallel_12 = sum(aa12 >= 5 & valid_12);
    not_parallel_13 = sum(aa13 >= 5 & valid_13);
    not_parallel_23 = sum(aa23 >= 5 & valid_23);
    fprintf('  - Non paralleli (>=5°): est-mes=%d, est-rob=%d, mes-rob=%d\n', not_parallel_12, not_parallel_13, not_parallel_23);
    
    fprintf('=====================================\n\n');
    
    %% Trajectory 3D
    fprintf('Plotting 3D trajectory...\n');
    figure('Name', 'Robot Trajectory');
    scatter3(prob(1,:), prob(2,:), prob(3,:), [], time);
    colorbar; 
    colormap(jet);
    xlabel('On X');
    ylabel('On Y');
    zlabel('On Z');
    title('Trajectory XYZ (Color = time)');
    grid on;
    
    fprintf('All plots completed!\n');
end
