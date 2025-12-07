function plot_results(time, N, h_ref, x_true, x_est, rob_rot, clean_rot, goal, u, ...
                      n_est, n_mes, wRr, prob, n_dim, d_dim, i_dim, ...
                      x_loc, eta_gt, nu_gt, wRr_gt)
    %% PLOT_RESULTS - Visualizzazione completa dei risultati della simulazione
    % 
    % Inputs:
    %   time: vettore temporale
    %   N: numero di iterazioni
    %   h_ref: altitudine di riferimento
    %   x_true: stati veri (SBES EKF)
    %   x_est: stati stimati (SBES EKF)
    %   rob_rot: angoli del robot (from EKF position filter)
    %   clean_rot: angoli del robot ground truth
    %   goal: struttura con i goal
    %   u: body velocities (from EKF position filter)
    %   n_est: normali stimate
    %   n_mes: normali misurate
    %   wRr: matrici di rotazione del robot (from EKF)
    %   prob: posizioni del robot (from EKF)
    %   n_dim, d_dim, i_dim: dimensioni
    %   x_loc: EKF position filter full state [15 x N]
    %   eta_gt: ground truth position & orientation [6 x N]
    %   nu_gt: ground truth body velocities [6 x N]
    %   wRr_gt: ground truth rotation matrices [3 x 3 x N]
    
    global HEAVE;
    
    %% States
    fprintf('Plotting states...\n');
    ttl = {'altitude', 'alpha', 'beta'};
    tags = {'state_altitude', 'state_alpha', 'state_beta'};
    for i = 1:n_dim
        figure('Name', sprintf('State: %s', ttl{i}), 'Tag', tags{i}, 'NumberTitle', 'off');
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
    tags = {'robot_roll', 'robot_pitch', 'robot_yaw'};
    for i = 1:d_dim
        figure('Name', sprintf('Robot angle: %s', ttl{i}), 'Tag', tags{i}, 'NumberTitle', 'off');
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
    tags = {'input_surge', 'input_sway', 'input_heave', 'input_p', 'input_q', 'input_r'};
    for i = 1:i_dim
        figure('Name', sprintf('Input: %s', ttl{i}), 'Tag', tags{i}, 'NumberTitle', 'off');
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
    figure('Name', 'z-sign', 'Tag', 'normal_zsign', 'NumberTitle', 'off'); 
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
    figure('Name', 'Parallelismo tra normali', 'Tag', 'normal_parallelism', 'NumberTitle', 'off'); 
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
    figure('Name', 'Robot Trajectory', 'Tag', 'trajectory_3d', 'NumberTitle', 'off');
    scatter3(prob(1,:), prob(2,:), -prob(3,:), [], time);  % -prob(3,:) per NED convention
    colorbar; 
    colormap(jet);
    xlabel('X (North)');
    ylabel('Y (East)');
    zlabel('Z (Down) - NED');
    title('Trajectory XYZ - NED Convention (Color = time)');
    grid on;
    set(gca, 'ZDir', 'reverse');  % Inverte la direzione dell'asse Z per visualizzazione NED
    
    %% ========================================================================
    %% EKF POSITION FILTER STATES - Comparison with Ground Truth
    %% ========================================================================
    fprintf('Plotting EKF Position Filter states vs Ground Truth...\n');
    
    %% Position States (x, y, z)
    figure('Name', 'EKF Position: XYZ vs Ground Truth', 'Tag', 'ekf_pos_xyz', 'NumberTitle', 'off');
    pos_labels = {'X (North)', 'Y (East)', 'Z (Down)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, eta_gt(i,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
        hold on;
        plot(time, x_loc(i,:), 'r--', 'LineWidth', 1.2, 'DisplayName', 'EKF Estimate');
        xlabel('Time [s]');
        ylabel([pos_labels{i} ' [m]']);
        title(['Position ' pos_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
    sgtitle('EKF Position Filter: Position States');
    
    %% Orientation States (phi, theta, psi)
    figure('Name', 'EKF Position: Angles vs Ground Truth', 'Tag', 'ekf_pos_angles', 'NumberTitle', 'off');
    ang_labels = {'Roll (φ)', 'Pitch (θ)', 'Yaw (ψ)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, rad2deg(eta_gt(3+i,:)), 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
        hold on;
        plot(time, rad2deg(x_loc(3+i,:)), 'r--', 'LineWidth', 1.2, 'DisplayName', 'EKF Estimate');
        xlabel('Time [s]');
        ylabel([ang_labels{i} ' [deg]']);
        title(['Orientation ' ang_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
    sgtitle('EKF Position Filter: Orientation States');
    
    %% Linear Velocity States (u, v, w)
    figure('Name', 'EKF Position: Linear Velocities vs Ground Truth', 'Tag', 'ekf_pos_velocities', 'NumberTitle', 'off');
    vel_labels = {'Surge (u)', 'Sway (v)', 'Heave (w)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, nu_gt(i,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
        hold on;
        plot(time, x_loc(6+i,:), 'r--', 'LineWidth', 1.2, 'DisplayName', 'EKF Estimate');
        xlabel('Time [s]');
        ylabel([vel_labels{i} ' [m/s]']);
        title(['Velocity ' vel_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
    sgtitle('EKF Position Filter: Linear Velocity States');
    
    %% Angular Velocity States (p, q, r)
    figure('Name', 'EKF Position: Angular Velocities vs Ground Truth', 'Tag', 'ekf_pos_angular_vel', 'NumberTitle', 'off');
    rate_labels = {'Roll rate (p)', 'Pitch rate (q)', 'Yaw rate (r)'};
    for i = 1:3
        subplot(3,1,i);
        plot(time, rad2deg(nu_gt(3+i,:)), 'b', 'LineWidth', 1.5, 'DisplayName', 'Ground Truth');
        hold on;
        plot(time, rad2deg(x_loc(9+i,:)), 'r--', 'LineWidth', 1.2, 'DisplayName', 'EKF Estimate');
        xlabel('Time [s]');
        ylabel([rate_labels{i} ' [deg/s]']);
        title(['Angular Rate ' rate_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
    sgtitle('EKF Position Filter: Angular Velocity States');
    
    %% Gyro Bias States (if present in x_loc indices 13-15)
    if size(x_loc, 1) >= 15
        figure('Name', 'EKF Position: Gyro Bias Estimates', 'Tag', 'ekf_pos_gyro_bias', 'NumberTitle', 'off');
        bias_labels = {'Gyro Bias X', 'Gyro Bias Y', 'Gyro Bias Z'};
        for i = 1:3
            subplot(3,1,i);
            plot(time, rad2deg(x_loc(12+i,:)), 'r', 'LineWidth', 1.2, 'DisplayName', 'Bias Estimate');
            xlabel('Time [s]');
            ylabel([bias_labels{i} ' [deg/s]']);
            title(bias_labels{i});
            legend('Location', 'best');
            grid on;
        end
        sgtitle('EKF Position Filter: Gyro Bias Estimates');
    end
    
    %% Position Estimation Errors
    figure('Name', 'EKF Position: Estimation Errors', 'Tag', 'ekf_pos_errors', 'NumberTitle', 'off');
    
    % Position errors
    subplot(2,2,1);
    pos_err = x_loc(1:3,:) - eta_gt(1:3,:);
    plot(time, pos_err(1,:), 'r', 'LineWidth', 1.2, 'DisplayName', 'X error');
    hold on;
    plot(time, pos_err(2,:), 'g', 'LineWidth', 1.2, 'DisplayName', 'Y error');
    plot(time, pos_err(3,:), 'b', 'LineWidth', 1.2, 'DisplayName', 'Z error');
    xlabel('Time [s]');
    ylabel('Position Error [m]');
    title('Position Estimation Errors');
    legend('Location', 'best');
    grid on;
    
    % Orientation errors  
    subplot(2,2,2);
    ang_err = rad2deg(x_loc(4:6,:) - eta_gt(4:6,:));
    plot(time, ang_err(1,:), 'r', 'LineWidth', 1.2, 'DisplayName', 'φ error');
    hold on;
    plot(time, ang_err(2,:), 'g', 'LineWidth', 1.2, 'DisplayName', 'θ error');
    plot(time, ang_err(3,:), 'b', 'LineWidth', 1.2, 'DisplayName', 'ψ error');
    xlabel('Time [s]');
    ylabel('Orientation Error [deg]');
    title('Orientation Estimation Errors');
    legend('Location', 'best');
    grid on;
    
    % Velocity errors
    subplot(2,2,3);
    vel_err = x_loc(7:9,:) - nu_gt(1:3,:);
    plot(time, vel_err(1,:), 'r', 'LineWidth', 1.2, 'DisplayName', 'u error');
    hold on;
    plot(time, vel_err(2,:), 'g', 'LineWidth', 1.2, 'DisplayName', 'v error');
    plot(time, vel_err(3,:), 'b', 'LineWidth', 1.2, 'DisplayName', 'w error');
    xlabel('Time [s]');
    ylabel('Velocity Error [m/s]');
    title('Linear Velocity Estimation Errors');
    legend('Location', 'best');
    grid on;
    
    % Angular rate errors
    subplot(2,2,4);
    rate_err = rad2deg(x_loc(10:12,:) - nu_gt(4:6,:));
    plot(time, rate_err(1,:), 'r', 'LineWidth', 1.2, 'DisplayName', 'p error');
    hold on;
    plot(time, rate_err(2,:), 'g', 'LineWidth', 1.2, 'DisplayName', 'q error');
    plot(time, rate_err(3,:), 'b', 'LineWidth', 1.2, 'DisplayName', 'r error');
    xlabel('Time [s]');
    ylabel('Angular Rate Error [deg/s]');
    title('Angular Rate Estimation Errors');
    legend('Location', 'best');
    grid on;
    
    sgtitle('EKF Position Filter: Estimation Errors');
    
    %% RMSE Summary for EKF Position Filter
    fprintf('\n=== EKF POSITION FILTER: ERROR STATISTICS ===\n');
    
    % Position RMSE
    pos_rmse = sqrt(mean(pos_err.^2, 2));
    fprintf('Position RMSE:\n');
    fprintf('  X: %.4f m\n', pos_rmse(1));
    fprintf('  Y: %.4f m\n', pos_rmse(2));
    fprintf('  Z: %.4f m\n', pos_rmse(3));
    fprintf('  Total: %.4f m\n', norm(pos_rmse));
    
    % Orientation RMSE
    ang_rmse = sqrt(mean(ang_err.^2, 2));
    fprintf('\nOrientation RMSE:\n');
    fprintf('  Roll:  %.4f deg\n', ang_rmse(1));
    fprintf('  Pitch: %.4f deg\n', ang_rmse(2));
    fprintf('  Yaw:   %.4f deg\n', ang_rmse(3));
    
    % Velocity RMSE
    vel_rmse = sqrt(mean(vel_err.^2, 2));
    fprintf('\nLinear Velocity RMSE:\n');
    fprintf('  Surge: %.4f m/s\n', vel_rmse(1));
    fprintf('  Sway:  %.4f m/s\n', vel_rmse(2));
    fprintf('  Heave: %.4f m/s\n', vel_rmse(3));
    
    % Angular rate RMSE
    rate_rmse = sqrt(mean(rate_err.^2, 2));
    fprintf('\nAngular Rate RMSE:\n');
    fprintf('  p: %.4f deg/s\n', rate_rmse(1));
    fprintf('  q: %.4f deg/s\n', rate_rmse(2));
    fprintf('  r: %.4f deg/s\n', rate_rmse(3));
    
    fprintf('=====================================\n\n');
    
    fprintf('All plots completed!\n');
end
