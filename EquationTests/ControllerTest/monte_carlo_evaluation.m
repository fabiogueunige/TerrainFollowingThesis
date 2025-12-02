function monte_carlo_evaluation()
    clc; clear; close all;
    addpath('for_controller');
    addpath('rotations');
    addpath('sensors');
    addpath('model');
    addpath('ekf_position');

    %% Analysis Configuration
    noise_scales = [0.5, 1.0, 2.0]; % Test Low, Normal, and High noise conditions
    N_runs_per_scale = 5;           % Runs per condition
    
    % Storage for results
    results = struct();

    fprintf('Starting Monte Carlo Analysis (Focus: Position Z & Altitude h)...\n');

    for s_idx = 1:length(noise_scales)
        scale = noise_scales(s_idx);
        fprintf('\n--- Testing Noise Scale: %.1fx ---\n', scale);
        
        % Metrics Storage
        rmse_z_raw = zeros(N_runs_per_scale, 1);
        rmse_z_filt = zeros(N_runs_per_scale, 1);
        rmse_h_raw = zeros(N_runs_per_scale, 1);
        rmse_h_filt = zeros(N_runs_per_scale, 1);
        
        for run = 1:N_runs_per_scale
            fprintf('Run %d/%d... ', run, N_runs_per_scale);
            
            %% Simulation Setup
            Ts = 0.001; Tf = 50; time = 0:Ts:Tf; N = length(time);
            i_dim = 6; d_dim = 3;
            
            % Initial State
            eta = zeros(i_dim,N); eta(1:3,1) = [2; 0; 0]; eta(4:6,1) = [0; 0; 0];
            wRr = zeros(d_dim, d_dim, N);
            nu = zeros(i_dim, N); a = zeros(i_dim,N);
            
            % EKF Setup with SCALED Noise
            P_loc = zeros(i_dim,i_dim,N);
            [Q_loc, R_loc, P_loc(:,:,1)] = setup_loc();
            Q_loc = Q_loc * scale;
            R_loc = R_loc * scale;
            
            wRr(:,:,1) = rotz(eta(6,1)) * roty(eta(5,1)) * rotx(eta(4,1));
            
            % Ground Truth
            pos = zeros(i_dim,N); pos(:,1) = eta(:,1);
            rot = zeros(d_dim,d_dim,N); rot(:,:,1) = wRr(:,:,1);
            
            % Terrain & Angles
            p_seafloor_NED = [-3; 0; 5];
            alpha = zeros(1,N); beta = zeros(1,N);
            for k = 1:N
                t = time(k);
                if t < 10, alpha(k)=0; elseif t<25, alpha(k)=deg2rad(20); elseif t<40, alpha(k)=deg2rad(-45); else, alpha(k)=deg2rad(15); end
                if t < 5, beta(k)=0; elseif t<15, beta(k)=deg2rad(-20); elseif t<30, beta(k)=deg2rad(50); else, beta(k)=deg2rad(20); end
            end
            
            % Controller Params
            choice = 1; % Nonlinear + Noise
            speed0 = zeros(6,1); u_star = 0.3; v_star = 0; h_star = 3;
            err = zeros(i_dim, N); pid = zeros(i_dim, N); int_err = zeros(i_dim, N);
            h_err_int = 0; h_err_int_max = 10.0;
            max_pid = 4.0; max_pid_W = 15.0;
            
            Kp = [3.5; 3.5; 6.0; 2.5; 2.5; 2.0];
            Ki = [0.7; 0.7; 2.0; 0.3; 0.3; 0.4];
            Kd = [1.2; 1.2; 3.0; 2.0; 2.0; 1.5];
            
            % Filter Setup
            alpha_filt = 0.80;
            eta_filt = zeros(i_dim, N); eta_filt(:,1) = eta(:,1);
            
            % Altitude Storage
            h_clean = zeros(1,N);
            h_raw = zeros(1,N);
            h_filt_val = zeros(1,N);
            
            %% Main Loop
            for k = 2:N
                % Control Logic
                wRs = rotz(0) * roty(beta(k)) * rotx(alpha(k)) * rotx(pi);
                n0 = [0;0;1]; n_k = wRs * n0;
                nz = n_k(3) / max(norm(n_k), 1e-9);
                z_target = p_seafloor_NED(3) - h_star * nz;
                
                % Errors
                z_err = z_target - pos(3,k-1);
                h_err_int = max(min(h_err_int + z_err*Ts, h_err_int_max), -h_err_int_max);
                w_des = 0.5*z_err + 0.2*h_err_int;
                w_des = max(min(w_des, 0.5), -0.5);
                err(3,k) = w_des - nu(3,k-1);
                err(4,k) = alpha(k) - eta_filt(4,k-1);
                err(5,k) = beta(k) - eta_filt(5,k-1);
                err(6,k) = 0 - eta_filt(6,k-1);
                
                % PID
                for l=1:i_dim
                    if k==2, int_err(l,k)=err(l,k)*Ts; else, int_err(l,k)=int_err(l,k-1)+0.5*(err(l,k)+err(l,k-1))*Ts; end
                    int_err(l,k) = max(min(int_err(l,k), 5), -5);
                    d_err = (err(l,k)-err(l,k-1))/Ts;
                    u_pid = Kp(l)*err(l,k) + Ki(l)*int_err(l,k) + Kd(l)*d_err;
                    lim = max_pid; if l==3, lim=max_pid_W; end
                    pid(l,k) = max(min(u_pid, lim), -lim);
                end
                
                % Dynamics & EKF
                [a(:,k), nu(:,k)] = dynamic_model(pid(:,k), zeros(i_dim,1), speed0, eta(4:6,k-1), nu(:,k-1), Ts, i_dim, a(:,k-1), choice);
                [eta(:,k), wRr(:,:,k), P_loc(:,:,k)] = ekf_position(eta(:,k-1), nu(:,k), wRr(:,:,k-1), P_loc(:,:,k-1), Q_loc, R_loc, Ts, choice);
                
                % Filter
                eta_filt(:,k) = alpha_filt * eta_filt(:,k-1) + (1 - alpha_filt) * eta(:,k);
                
                % Ground Truth
                pos(1:3,k) = pos(1:3,k-1) + rot(:,:,k-1)*nu(1:3,k)*Ts;
                pos(4:6,k) = eta(4:6,k-1) + nu(4:6,k)*Ts;
                rot(:,:,k) = rotz(eta(6,k)) * roty(eta(5,k)) * rotx(eta(4,k));
                
                % Calculate Altitudes for Analysis
                % h = | n' * (pos - p_seafloor) | / |n|
                % n is normal to seafloor, derived from wRs
                % wRs was calculated at start of loop for control
                
                % Ground Truth Altitude
                h_clean(k) = abs((n_k'*(pos(1:3,k) - p_seafloor_NED))/norm(n_k));
                
                % Raw EKF Altitude
                h_raw(k) = abs((n_k'*(eta(1:3,k) - p_seafloor_NED))/norm(n_k));
                
                % Filtered Altitude
                h_filt_val(k) = abs((n_k'*(eta_filt(1:3,k) - p_seafloor_NED))/norm(n_k));
            end
            
            %% Metrics Calculation
            % 1. Depth (Z) RMSE
            err_z_raw = eta(3,:) - pos(3,:);
            err_z_filt = eta_filt(3,:) - pos(3,:);
            rmse_z_raw(run) = sqrt(mean(err_z_raw.^2));
            rmse_z_filt(run) = sqrt(mean(err_z_filt.^2));
            
            % 2. Altitude (h) RMSE
            err_h_raw = h_raw - h_clean;
            err_h_filt = h_filt_val - h_clean;
            rmse_h_raw(run) = sqrt(mean(err_h_raw.^2));
            rmse_h_filt(run) = sqrt(mean(err_h_filt.^2));
            
            fprintf('Done.\n');
        end
        
        results(s_idx).scale = scale;
        results(s_idx).z_raw_mean = mean(rmse_z_raw);
        results(s_idx).z_filt_mean = mean(rmse_z_filt);
        results(s_idx).h_raw_mean = mean(rmse_h_raw);
        results(s_idx).h_filt_mean = mean(rmse_h_filt);
    end

    %% Final Report Generation
    fprintf('\n================================================================\n');
    fprintf('                  THESIS RESULTS SUMMARY                        \n');
    fprintf('================================================================\n');

    for i = 1:length(results)
        r = results(i);
        fprintf('\n[Noise Scale: %.1fx]\n', r.scale);
        
        fprintf('  1. DEPTH (Z) ESTIMATION ACCURACY (RMSE):\n');
        fprintf('     Raw EKF:      %.4f [m]\n', r.z_raw_mean);
        fprintf('     Filtered:     %.4f [m]\n', r.z_filt_mean);
        improv_z = (1 - r.z_filt_mean / r.z_raw_mean) * 100;
        fprintf('     Improvement:  %+.1f%%\n', improv_z);
        
        fprintf('  2. ALTITUDE (h) ESTIMATION ACCURACY (RMSE):\n');
        fprintf('     Raw EKF:      %.4f [m]\n', r.h_raw_mean);
        fprintf('     Filtered:     %.4f [m]\n', r.h_filt_mean);
        improv_h = (1 - r.h_filt_mean / r.h_raw_mean) * 100;
        fprintf('     Improvement:  %+.1f%%\n', improv_h);
    end

    %% Visualization for Thesis
    figure('Name', 'Thesis Results: Filter Efficacy (Z & h)', 'Color', 'w');
    scales = [results.scale];
    
    subplot(1,2,1);
    y_z_raw = [results.z_raw_mean];
    y_z_filt = [results.z_filt_mean];
    bar(scales, [y_z_raw; y_z_filt]');
    legend('Raw EKF', 'Filtered');
    xlabel('Noise Scale Factor');
    ylabel('Depth RMSE [m]');
    title('Depth (Z) Estimation Error');
    grid on;
    
    subplot(1,2,2);
    y_h_raw = [results.h_raw_mean];
    y_h_filt = [results.h_filt_mean];
    bar(scales, [y_h_raw; y_h_filt]');
    legend('Raw EKF', 'Filtered');
    xlabel('Noise Scale Factor');
    ylabel('Altitude RMSE [m]');
    title('Altitude (h) Estimation Error');
    grid on;
end
