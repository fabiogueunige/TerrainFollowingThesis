%% COLLECT_SIMULATION_DATA - Collect all simulation variables into structure
%
% Collects all important simulation data into a single structure for saving.
% This function should be called at the end of the simulation loop.
%
% SYNTAX:
%   sim_data = collect_simulation_data(time, Ts, Tf, N, h_ref, ...
%       x_true, x_est, x_pred, ni, S, P, P0, ...
%       z_meas, z_pred, n_mes, n_est, n_pre, rob_rot, clean_rot, R, ...
%       pid, u, u_dot, goal, integral_err, p_err, i_err, t_sum, ...
%       prob, wRr, wRt, wRt_pre, state, ...
%       Q, R_tp, R_a, Kp, Ki, Kd, Kt, speed0, tau0, x0, x0_est)
%
% INPUTS:
%   All simulation variables (see main_6DOF_3D.m for definitions)
%
% OUTPUTS:
%   sim_data - Structure containing all simulation data organized by category:
%              .time, .Ts, .Tf, .N              - Time parameters
%              .x_true, .x_est, .x_pred, .h_ref - EKF states
%              .ni, .S, .P_final, .P0           - EKF covariance
%              .z_meas, .z_pred, .n_mes, ...    - Sensor data
%              .pid, .u, .u_dot, .goal, ...     - Control data
%              .prob, .wRr, .wRt, .state        - Trajectory data
%              .Q, .R_tp, .R_a, .Kp, ...        - Parameters
%
% EXAMPLE:
%   % At end of simulation
%   sim_data = collect_simulation_data(time, Ts, Tf, N, h_ref, ...
%       x_true, x_est, x_pred, ni, S, P, P0, ...);
%   save_simulation_data(sim_data);
%
% See also: save_simulation_data, load_simulation_data

function sim_data = collect_simulation_data(time, Ts, Tf, N, h_ref, ...
    x_true, x_est, x_pred, ni, S, P, P0, ...
    z_meas, z_pred, n_mes, n_est, n_pre, rob_rot, clean_rot, R, ...
    pid, u, u_dot, goal, integral_err, p_err, i_err, t_sum, ...
    prob, wRr, wRt, wRt_pre, state, ...
    Q, R_SBES, Kp, Ki, Kd, speed0, x0, x0_est, ...
    max_planes, step_length, angle_range, rate_of_change, delta_limit, pp_init_w, n0, ...
    x_loc, eta_gt, nu_gt, wRr_gt)

    fprintf('\n=== COLLECTING SIMULATION DATA ===\n');
    
    %% Time Parameters
    sim_data.time = time;
    sim_data.Ts = Ts;
    sim_data.Tf = Tf;
    sim_data.N = N;
    
    %% EKF States
    fprintf('Collecting EKF states...\n');
    sim_data.h_ref = h_ref;
    sim_data.x_true = x_true;
    sim_data.x_est = x_est;
    sim_data.x_pred = x_pred;
    
    %% EKF Covariance
    fprintf('Collecting EKF covariance data...\n');
    sim_data.ni = ni;
    sim_data.S = S;
    sim_data.P_final = P;
    sim_data.P0 = P0;
    
    %% Sensor Data
    fprintf('Collecting sensor data...\n');
    sim_data.z_meas = z_meas;
    sim_data.z_pred = z_pred;
    sim_data.n_mes = n_mes;
    sim_data.n_est = n_est;
    sim_data.n_pre = n_pre;
    sim_data.rob_rot = rob_rot;
    sim_data.clean_rot = clean_rot;
    sim_data.R = R;
    
    %% Control Data
    fprintf('Collecting control data...\n');
    sim_data.pid = pid;
    sim_data.u = u;
    sim_data.u_dot = u_dot;
    sim_data.goal = goal;
    sim_data.integral_err = integral_err;
    sim_data.p_err = p_err;
    sim_data.i_err = i_err;
    sim_data.t_sum = t_sum;
    
    %% Trajectory Data
    fprintf('Collecting trajectory data...\n');
    sim_data.prob = prob;
    sim_data.wRr = wRr;
    sim_data.wRt = wRt;
    sim_data.wRt_pre = wRt_pre;
    sim_data.state = state;
    
    %% Simulation Parameters
    fprintf('Collecting simulation parameters...\n');
    sim_data.Q = Q;
    sim_data.R_SBES = R_SBES;
    sim_data.Kp = Kp;
    sim_data.Ki = Ki;
    sim_data.Kd = Kd;
    sim_data.speed0 = speed0;
    sim_data.x0 = x0;
    sim_data.x0_est = x0_est;
    
    %% Terrain Generation Parameters
    fprintf('Collecting terrain parameters...\n');
    sim_data.max_planes = max_planes;
    sim_data.step_length = step_length;
    sim_data.angle_range = angle_range;
    sim_data.rate_of_change = rate_of_change;
    sim_data.delta_limit = delta_limit;
    sim_data.pp_init_w = pp_init_w;
    sim_data.n0 = n0;
    
    %% EKF Position Filter Data (Ground Truth comparison)
    fprintf('Collecting EKF position filter data...\n');
    sim_data.x_loc = x_loc;           % Full EKF position state [15 x N]
    sim_data.eta_gt = eta_gt;         % Ground truth position & orientation [6 x N]
    sim_data.nu_gt = nu_gt;           % Ground truth body velocities [6 x N]
    sim_data.wRr_gt = wRr_gt;         % Ground truth rotation matrices [3 x 3 x N]
    
    fprintf('Collection complete!\n');
    fprintf('Total data fields: %d\n\n', length(fieldnames(sim_data)));
end
