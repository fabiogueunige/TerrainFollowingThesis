function plot_results(time, N, h_ref, x_true, x_est, rob_rot, clean_rot, goal, u, ...
                      n_est, n_mes, wRr, prob, n_dim, d_dim, i_dim, ...
                      x_loc, eta_gt, nu_gt)
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
    
    %% States - Ogni stato in una figura separata
    fprintf('Plotting states...\n');
    ttl = {'Altitude', 'Terrain Angle \alpha', 'Terrain Angle \beta'};
    ylabels = {'Altitude [m]', 'Alpha [deg]', 'Beta [deg]'};
    for i = 1:n_dim
        figure('Name', sprintf('State: %s', ttl{i}), 'Tag', sprintf('state_%d', i), 'NumberTitle', 'off');
        if (i == 1)
            plot(time, x_est(i,:), 'g', 'LineWidth', 2.2, 'DisplayName', 'Estimated'); hold on;
            plot(time, x_true(i,:), 'r', 'LineWidth', 2, 'DisplayName', 'True');
            plot(time, h_ref(:), 'b', 'LineWidth', 3.5, 'DisplayName', 'Desired');
            ylabel(ylabels{i});
        else
            plot(time, rad2deg(x_est(i,:)), 'g', 'LineWidth', 2.2, 'DisplayName', 'Estimated'); hold on;
            plot(time, rad2deg(rob_rot(i-1,:)), 'r', 'LineWidth', 2, 'DisplayName', 'Robot angle');
            plot(time, rad2deg(x_true(i,:)), 'b', 'LineWidth', 3.5, 'DisplayName', 'True (desired) Terrain');
            ylabel(ylabels{i});
        end
        xlabel('Time [s]');
        legend('Location','best'); grid on;
        title(ttl{i});
        hold off;
    end
    
    %% Robot angles - Ogni angolo in una figura separata
    fprintf('Plotting robot angles...\n');
    ttl = {'Roll', 'Pitch', 'Yaw'};
    ylabels = {'Roll [deg]', 'Pitch [deg]', 'Yaw [deg]'};
    for i = 1:d_dim
        figure('Name', sprintf('Robot angle: %s', ttl{i}), 'Tag', sprintf('robot_angle_%d', i), 'NumberTitle', 'off');
        if i == 1
            plot(time, rad2deg([goal.roll]), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Goal'); hold on;
        end
        if i == 2
            plot(time, rad2deg([goal.pitch]), 'b--', 'LineWidth', 1.5, 'DisplayName', 'Goal'); hold on;
        end
        plot(time, rad2deg(rob_rot(i,:)), 'g', 'LineWidth', 2.3, 'DisplayName', 'EKF Estimate'); hold on;
        plot(time, rad2deg(clean_rot(i,:)), 'r', 'LineWidth', 2, 'DisplayName', 'True (no noise)');
        xlabel('Time [s]'); ylabel(ylabels{i});
        legend('Location','best'); grid on;
        title(ttl{i});
        hold off;
    end
    
    %% Inputs - Ogni input in una figura separata
    fprintf('Plotting control inputs...\n');
    ttl = {'Surge (u)', 'Sway (v)', 'Heave (w)', 'Roll rate (p)', 'Pitch rate (q)', 'Yaw rate (r)'};
    ylabels = {'u [m/s]', 'v [m/s]', 'w [m/s]', 'p [deg/s]', 'q [deg/s]', 'r [deg/s]'};
    for i = 1:i_dim
        figure('Name', sprintf('Input: %s', ttl{i}), 'Tag', sprintf('input_%d', i), 'NumberTitle', 'off');
        if i <= HEAVE
            plot(time, u(i,:), 'b', 'LineWidth', 2, 'DisplayName', ttl{i});
        else
            plot(time, rad2deg(u(i,:)), 'b', 'LineWidth', 2, 'DisplayName', ttl{i});
        end
        xlabel('Time [s]'); ylabel(ylabels{i});
        legend('Location','best'); grid on;
        title(ttl{i});
        hold off;
    end
    
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
    
    %% Position States (x, y, z) - Figure separate
    pos_labels = {'X (North)', 'Y (East)', 'Z (Down)'};
    for i = 1:3
        figure('Name', sprintf('EKF Position: %s vs Ground Truth', pos_labels{i}), 'Tag', sprintf('ekf_pos_xyz_%d', i), 'NumberTitle', 'off');
        plot(time, x_loc(i,:), 'r--', 'LineWidth', 2.7, 'DisplayName', 'EKF Estimate'); hold on;
        plot(time, eta_gt(i,:), 'b', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
        xlabel('Time [s]');
        ylabel([pos_labels{i} ' [m]']);
        title(['Position ' pos_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
    
    %% Orientation States (phi, theta, psi) - Figure separate
    ang_labels = {'Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)'};
    for i = 1:3
        figure('Name', sprintf('EKF Position: %s vs Ground Truth', ang_labels{i}), 'Tag', sprintf('ekf_pos_angle_%d', i), 'NumberTitle', 'off');
        plot(time, rad2deg(x_loc(3+i,:)), 'r--', 'LineWidth', 2.7, 'DisplayName', 'EKF Estimate'); hold on;
        plot(time, rad2deg(eta_gt(3+i,:)), 'b', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
        xlabel('Time [s]');
        ylabel([ang_labels{i} ' [deg]']);
        title(['Orientation ' ang_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
    
    %% Linear Velocity States (u, v, w) - Figure separate
    vel_labels = {'Surge (u)', 'Sway (v)', 'Heave (w)'};
    for i = 1:3
        figure('Name', sprintf('EKF Position: %s vs Ground Truth', vel_labels{i}), 'Tag', sprintf('ekf_pos_vel_%d', i), 'NumberTitle', 'off');
        plot(time, x_loc(6+i,:), 'r--', 'LineWidth', 2.7, 'DisplayName', 'EKF Estimate'); hold on;
        plot(time, nu_gt(i,:), 'b', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
        xlabel('Time [s]');
        ylabel([vel_labels{i} ' [m/s]']);
        title(['Velocity ' vel_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
    
    %% Angular Velocity States (p, q, r) - Figure separate
    rate_labels = {'Roll rate (p)', 'Pitch rate (q)', 'Yaw rate (r)'};
    for i = 1:3
        figure('Name', sprintf('EKF Position: %s vs Ground Truth', rate_labels{i}), 'Tag', sprintf('ekf_pos_angvel_%d', i), 'NumberTitle', 'off');
        plot(time, rad2deg(x_loc(9+i,:)), 'r--', 'LineWidth', 2.7, 'DisplayName', 'EKF Estimate'); hold on;
        plot(time, rad2deg(nu_gt(3+i,:)), 'b', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
        xlabel('Time [s]');
        ylabel([rate_labels{i} ' [deg/s]']);
        title(['Angular Rate ' rate_labels{i}]);
        legend('Location', 'best');
        grid on;
        hold off;
    end
    
    %% Gyro Bias States (if present in x_loc indices 13-15) - Figure separate
    if size(x_loc, 1) >= 15
        bias_labels = {'Gyro Bias X', 'Gyro Bias Y', 'Gyro Bias Z'};
        for i = 1:3
            figure('Name', sprintf('EKF Position: %s', bias_labels{i}), 'Tag', sprintf('ekf_pos_gyro_bias_%d', i), 'NumberTitle', 'off');
            plot(time, rad2deg(x_loc(12+i,:)), 'r', 'LineWidth', 2.0, 'DisplayName', 'Bias Estimate');
            xlabel('Time [s]');
            ylabel([bias_labels{i} ' [deg/s]']);
            title(bias_labels{i});
            legend('Location', 'best');
            grid on;
        end
    end
    
    %% Position Estimation Errors - Figure separate for each error type
    % Position errors
    pos_err = x_loc(1:3,:) - eta_gt(1:3,:);
    figure('Name', 'EKF Position Error: X', 'Tag', 'ekf_pos_err_x', 'NumberTitle', 'off');
    plot(time, pos_err(1,:), 'r', 'LineWidth', 2.5, 'DisplayName', 'X error');
    xlabel('Time [s]'); ylabel('X Error [m]');
    title('EKF Position Error: X'); legend('X error', 'Location', 'best'); grid on;
    
    figure('Name', 'EKF Position Error: Y', 'Tag', 'ekf_pos_err_y', 'NumberTitle', 'off');
    plot(time, pos_err(2,:), 'g', 'LineWidth', 2.5, 'DisplayName', 'Y error');
    xlabel('Time [s]'); ylabel('Y Error [m]');
    title('EKF Position Error: Y'); legend('Y error', 'Location', 'best'); grid on;
    
    figure('Name', 'EKF Position Error: Z', 'Tag', 'ekf_pos_err_z', 'NumberTitle', 'off');
    plot(time, pos_err(3,:), 'b', 'LineWidth', 2.5, 'DisplayName', 'Z error');
    xlabel('Time [s]'); ylabel('Z Error [m]');
    title('EKF Position Error: Z'); legend('Z error', 'Location', 'best'); grid on;
    
    % Orientation errors
    ang_err = rad2deg(x_loc(4:6,:) - eta_gt(4:6,:));
    figure('Name', 'EKF Orientation Error: Roll (φ)', 'Tag', 'ekf_ang_err_phi', 'NumberTitle', 'off');
    plot(time, ang_err(1,:), 'r', 'LineWidth', 2.5, 'DisplayName', 'Roll (φ) error');
    xlabel('Time [s]'); ylabel('Roll Error [deg]');
    title('EKF Orientation Error: Roll (φ)'); legend('Roll (φ) error', 'Location', 'best'); grid on;
    
    figure('Name', 'EKF Orientation Error: Pitch (θ)', 'Tag', 'ekf_ang_err_theta', 'NumberTitle', 'off');
    plot(time, ang_err(2,:), 'g', 'LineWidth', 2.5, 'DisplayName', 'Pitch (θ) error');
    xlabel('Time [s]'); ylabel('Pitch Error [deg]');
    title('EKF Orientation Error: Pitch (θ)'); legend('Pitch (θ) error', 'Location', 'best'); grid on;
    
    figure('Name', 'EKF Orientation Error: Yaw (ψ)', 'Tag', 'ekf_ang_err_psi', 'NumberTitle', 'off');
    plot(time, ang_err(3,:), 'b', 'LineWidth', 2.5, 'DisplayName', 'Yaw (ψ) error');
    xlabel('Time [s]'); ylabel('Yaw Error [deg]');
    title('EKF Orientation Error: Yaw (ψ)'); legend('Yaw (ψ) error', 'Location', 'best'); grid on;
    
    % Velocity errors
    vel_err = x_loc(7:9,:) - nu_gt(1:3,:);
    figure('Name', 'EKF Linear Velocity Error: u', 'Tag', 'ekf_vel_err_u', 'NumberTitle', 'off');
    plot(time, vel_err(1,:), 'r', 'LineWidth', 2.5, 'DisplayName', 'u error');
    xlabel('Time [s]'); ylabel('u Error [m/s]');
    title('EKF Linear Velocity Error: u'); legend('u error', 'Location', 'best'); grid on;
    
    figure('Name', 'EKF Linear Velocity Error: v', 'Tag', 'ekf_vel_err_v', 'NumberTitle', 'off');
    plot(time, vel_err(2,:), 'g', 'LineWidth', 2.5, 'DisplayName', 'v error');
    xlabel('Time [s]'); ylabel('v Error [m/s]');
    title('EKF Linear Velocity Error: v'); legend('v error', 'Location', 'best'); grid on;
    
    figure('Name', 'EKF Linear Velocity Error: w', 'Tag', 'ekf_vel_err_w', 'NumberTitle', 'off');
    plot(time, vel_err(3,:), 'b', 'LineWidth', 2.5, 'DisplayName', 'w error');
    xlabel('Time [s]'); ylabel('w Error [m/s]');
    title('EKF Linear Velocity Error: w'); legend('w error', 'Location', 'best'); grid on;
    
    % Angular rate errors
    rate_err = rad2deg(x_loc(10:12,:) - nu_gt(4:6,:));
    figure('Name', 'EKF Angular Rate Error: p', 'Tag', 'ekf_rate_err_p', 'NumberTitle', 'off');
    plot(time, rate_err(1,:), 'r', 'LineWidth', 2.5, 'DisplayName', 'p error');
    xlabel('Time [s]'); ylabel('p Error [deg/s]');
    title('EKF Angular Rate Error: p'); legend('p error', 'Location', 'best'); grid on;
    
    figure('Name', 'EKF Angular Rate Error: q', 'Tag', 'ekf_rate_err_q', 'NumberTitle', 'off');
    plot(time, rate_err(2,:), 'g', 'LineWidth', 2.5, 'DisplayName', 'q error');
    xlabel('Time [s]'); ylabel('q Error [deg/s]');
    title('EKF Angular Rate Error: q'); legend('q error', 'Location', 'best'); grid on;
    
    figure('Name', 'EKF Angular Rate Error: r', 'Tag', 'ekf_rate_err_r', 'NumberTitle', 'off');
    plot(time, rate_err(3,:), 'b', 'LineWidth', 2.5, 'DisplayName', 'r error');
    xlabel('Time [s]'); ylabel('r Error [deg/s]');
    title('EKF Angular Rate Error: r'); legend('r error', 'Location', 'best'); grid on;
    
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
