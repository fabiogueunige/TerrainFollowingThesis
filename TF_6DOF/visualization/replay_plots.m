%% REPLAY_PLOTS - Rigenera tutti i grafici da dati salvati
%
% Questo script carica i dati di una simulazione salvata e rigenera
% tutti i grafici utilizzando la funzione plot_results.
%
% UTILIZZO:
%   1. Esegui questo script
%   2. Seleziona il run da visualizzare dalla lista
%   3. Tutti i grafici verranno rigenerati
%
% SINTASSI:
%   replay_plots                  % Mostra lista e seleziona interattivamente
%   replay_plots('run_name')      % Carica run specifico
%
% ESEMPIO:
%   replay_plots('run_20251201_105813')
%
% OUTPUT:
%   - Tutti i grafici della simulazione (stati, angoli, input, normali, traiettoria)
%   - Statistiche di analisi delle normali
%
% See also: load_simulation_data, plot_results, save_simulation_data

function replay_plots(run_name)
    %% Setup paths
    addpath('data_management');
    addpath('visualization');
    
    %% Load simulation data
    fprintf('\n========================================\n');
    fprintf('   REPLAY PLOTS - Data Visualization\n');
    fprintf('========================================\n\n');
    
    if nargin < 1
        % Interactive selection
        sim_data = load_simulation_data();
    else
        % Load specific run
        sim_data = load_simulation_data(run_name);
    end
    
    %% Extract dimensions from data
    n_dim = 3;  % Number of states [h, alpha, beta]
    d_dim = 3;  % World dimensions [roll, pitch, yaw]
    i_dim = 6;  % Number of inputs [u, v, w, p, q, r]
    
    %% Set global variable needed by plot_results
    global HEAVE;
    HEAVE = 3;
    
    %% Close all existing figures
    close all;
    
    %% Call plot_results with loaded data
    fprintf('\n========================================\n');
    fprintf('   Generating Plots...\n');
    fprintf('========================================\n\n');
    
    % Check if new EKF position data exists
    if isfield(sim_data, 'x_loc') && isfield(sim_data, 'eta_gt') && ...
       isfield(sim_data, 'nu_gt') && isfield(sim_data, 'wRr_gt')
        % New format with EKF position filter data
        plot_results(sim_data.time, sim_data.N, sim_data.h_ref, ...
                     sim_data.x_true, sim_data.x_est, ...
                     sim_data.rob_rot, sim_data.clean_rot, ...
                     sim_data.goal, sim_data.u, ...
                     sim_data.n_est, sim_data.n_mes, ...
                     sim_data.wRr, sim_data.prob, ...
                     n_dim, d_dim, i_dim, ...
                     sim_data.x_loc, sim_data.eta_gt, ...
                     sim_data.nu_gt, sim_data.wRr_gt);
    else
        % Legacy format without EKF position filter data
        warning('Legacy data format detected. EKF position plots will not be shown.');
        % Create dummy data for backward compatibility
        N = sim_data.N;
        x_loc_dummy = zeros(15, N);
        eta_gt_dummy = zeros(6, N);
        nu_gt_dummy = zeros(6, N);
        wRr_gt_dummy = zeros(3, 3, N);
        
        plot_results(sim_data.time, sim_data.N, sim_data.h_ref, ...
                     sim_data.x_true, sim_data.x_est, ...
                     sim_data.rob_rot, sim_data.clean_rot, ...
                     sim_data.goal, sim_data.u, ...
                     sim_data.n_est, sim_data.n_mes, ...
                     sim_data.wRr, sim_data.prob, ...
                     n_dim, d_dim, i_dim, ...
                     x_loc_dummy, eta_gt_dummy, ...
                     nu_gt_dummy, wRr_gt_dummy);
    end
    
    %% Compute and display comprehensive performance metrics
    % Following RESULTS_GUIDE_2.pdf specifications
    fprintf('\n========================================\n');
    fprintf('   Computing Performance Metrics...\n');
    fprintf('========================================\n\n');
    
    try
        % Compute all metrics from RESULTS_GUIDE
        metrics = compute_performance_metrics(sim_data);
        
        % Display formatted results
        display_performance_metrics(metrics, false);  % false = summary mode
        
    catch ME
        % Fallback to basic statistics if new functions fail
        warning('Could not compute full metrics: %s\nUsing basic statistics...', ME.message);
        
        fprintf('\n========================================\n');
        fprintf('   Performance Summary (Basic)\n');
        fprintf('========================================\n\n');
        
        % Altitude tracking performance
        h_error = sim_data.h_ref - sim_data.x_est(1,:);
        fprintf('ALTITUDE TRACKING:\n');
        fprintf('  Mean error:     %.4f m\n', mean(h_error));
        fprintf('  RMS error:      %.4f m\n', rms(h_error));
        fprintf('  Max error:      %.4f m\n', max(abs(h_error)));
        fprintf('  Std deviation:  %.4f m\n', std(h_error));
        
        % Angle tracking (alpha, beta)
        if size(sim_data.x_est, 1) >= 3
            alpha_error = sim_data.x_true(2,:) - sim_data.x_est(2,:);
            beta_error = sim_data.x_true(3,:) - sim_data.x_est(3,:);
            
            fprintf('\nALPHA ANGLE TRACKING:\n');
            fprintf('  Mean error:     %.4f rad (%.2f°)\n', mean(alpha_error), rad2deg(mean(alpha_error)));
            fprintf('  RMS error:      %.4f rad (%.2f°)\n', rms(alpha_error), rad2deg(rms(alpha_error)));
            fprintf('  Max error:      %.4f rad (%.2f°)\n', max(abs(alpha_error)), rad2deg(max(abs(alpha_error))));
            
            fprintf('\nBETA ANGLE TRACKING:\n');
            fprintf('  Mean error:     %.4f rad (%.2f°)\n', mean(beta_error), rad2deg(mean(beta_error)));
            fprintf('  RMS error:      %.4f rad (%.2f°)\n', rms(beta_error), rad2deg(rms(beta_error)));
            fprintf('  Max error:      %.4f rad (%.2f°)\n', max(abs(beta_error)), rad2deg(max(abs(beta_error))));
        end
        
        % Control effort
        if isfield(sim_data, 'u')
            fprintf('\nCONTROL EFFORT:\n');
            control_labels = {'surge (u)', 'sway (v)', 'heave (w)', 'roll rate (p)', 'pitch rate (q)', 'yaw rate (r)'};
            for i = 1:min(6, size(sim_data.u, 1))
                fprintf('  %s: mean=%.4f, max=%.4f, std=%.4f\n', ...
                    control_labels{i}, mean(abs(sim_data.u(i,:))), ...
                    max(abs(sim_data.u(i,:))), std(sim_data.u(i,:)));
            end
        end
        
        % Covariance trace evolution
        if isfield(sim_data, 'P_final')
            fprintf('\nFINAL ESTIMATION COVARIANCE (trace):\n');
            fprintf('  %.6f\n', trace(sim_data.P_final));
        end
        
        fprintf('\n========================================\n');
    end
    
    fprintf('   Replay Complete!\n');
    fprintf('========================================\n\n');
    
    fprintf('TIP: Use figure windows toolbar to zoom, pan, and save plots\n\n');
end
