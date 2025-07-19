clc; clear; close all;
addpath('for_controller');
%% Filter Parameters
Ts = 0.001;       % Sampling time [s]
Tf = 1000;          % Final time [s]
time = 0:Ts:Tf;   % Time vector
N = length(time); % Number of iterations
%% Dimensions
i_dim = 1;          % Number of inputs 
d_dim = 3;          % world space total dimensions
a_dim = 3;          % number of angles

%% Robot 
pr = zeros(d_dim,N);        % AUV initial position
u = zeros(i_dim, N);        % Known inputs
u_dot = zeros(i_dim,N);     % AUV acceleration
a = zeros(i_dim,N);         % Real acceleration

%% Controller choice
controller = 'A+D';
model = 'B';
speed0 = 0.4;

%% Dynamic
if model == 'B'
    % massa totale [kg]
    m = 11.5; 
    % added mass = X_udot / (rho*V)
    tau_a = -27.08;
    % Linear damping (restoring)
    tau_r = -0.1213;
    % Quadratic damping (hydrodinamic)
    tau_d = -23.9000; % -18.18;
end
if model == 'M'
    % massa totale [kg]
    m = 30;
    % added mass
    tau_a = -25;
    % Linear damping
    tau_r = -0.2;
    % Quadratic damping
    tau_d = -19.5;
end

% Virtual mass
mv = m - tau_a;
% Dissipative forces (con v0 = 0.1 solo nel surge)
dv = -tau_r - tau_d.*abs(speed0);
dv_lin = -tau_r - 2*tau_d.*abs(speed0);

%% Controller Parameters
u_star = 0.3;
err = zeros(i_dim, N);
pid = zeros(i_dim, N);          % PID for Dynamics
int_term = zeros(i_dim, N);     % integral error
term_sum = zeros(i_dim, N);     % integral error
max_pid = 1;
integral_max = 1;

% errori pid semplice
i_err = zeros(i_dim, N);            % integral error

% errori pid classico
err_i = 0;
% errore pid delta

%% tau0
tau0 = dv*speed0;

%% Gain Computation
[Kp, Ki, Kt] = gainComputation(speed0, model);

%% Dynamic implementation
if controller == '1'
    u = ones(i_dim,N)*5;
end

%% Loop Cycle
for k = 3:N
    if mod(k,151) == 0
        fprintf('Vel on lap %.0f is : %.4f\n', k, u(k-1));
    end
    %% Error
    err(k) = u_star - u(k-1);
    %% Acc from v
    u_dot(k-1) = derivator(u_dot(k-2), u(k-1), u(k-2), Ts);

    %% Controllers
    switch controller
        case '0' % nothing on it and velociity is 0
            % Should stay fixed at 0
            pid(k) = 0;
        case '1' % start from faster velocity to go down to 0
            % Should decrease in time
            pid(k) = 0;
        case '2'
            pid(k) = 1;
        case 'PID'
            %% Simple PID  (funziona SEMPRE)
            err_i = integrator(err_i, err(k), err(k-1), Ts);
            i_err(k) = Ki*err_i;
            p_err = Kp * err(k);
            % % d_err = -(Kd * u_dot(k-1));
            pid(k) = i_err(k) + p_err;% + d_err;
        case 'A+D'
            %% PID + DELTA + ANTI WINDUP (funziona <=> speed0 = 0)
            i_err(k) = Ki*err(k);
            p_err = Kp * u_dot(k-1);
            term_sum(k) = i_err(k) - p_err - Kt*pid(k-1);
            int_term(k) = integrator(int_term(k-1), term_sum(k), term_sum(k-1), Ts);
            err_sat = max(min(int_term(k), max_pid), -max_pid);
            pid(k) = int_term(k) - err_sat;
        case 'D'
            %% PID & Delta FUNZIONA (funziona <=> speed0 = 0)
            i_err(k) = Ki*err(k);
            p_err = Kp * u_dot(k-1);
            term_sum(k) = i_err(k) - p_err - Kt*pid(k-1);
            int_term(k) = integrator(int_term(k-1), term_sum(k), term_sum(k-1), Ts);
            err_sat = max(min(int_term(k), max_pid), -max_pid);
            pid(k) = int_term(k) - err_sat;
        case 'A'
            %% PID + ANTI WINDUP
            err_i = integrator(err_i, err(k), err(k-1), Ts);
            err_i_sat = max(min(err_i, max_pid), -max_pid);
            err_i = err_i - err_i_sat;
            i_err(k) = Ki*err_i;
            p_err = Kp * err(k);
            pid(k) = i_err(k) + p_err;% + d_err;
    end
    %% Dynamic
    % Considering everything as a delta
    delta = u(k-1) - speed0;
    a(k) = (pid(k) - tau0 - dv_lin*delta) / mv;
    u(k) = integrator(u(k-1), a(k), a(k-1), Ts);

    %% New point
    pr(:,k) = pr(:,k-1) + [u(k)*Ts; 0; 0];
end

figure;
scatter3(pr(1,:), pr(2,:), pr(3,:), [], time);
colorbar; 
colormap(jet);
xlabel('On X');
ylabel('On Z');
title('Trajectory XYZ (Color = time)');
grid on;

figure;
plot(time, u(:));

function [kp, ki, Kt] = gainComputation(speed0, type)
    wn = 0.2;
    damp = 0.6;
    % p = 10;
    %% Model
    if type == 'B'
        % massa totale [kg]
        m = 11.5; 
        % added mass
        tau_a = -27.08;
        % Linear damping
        tau_r = -0.1213;
        % Quadratic damping
        tau_d = -23.9000; %-18.18;
    end
    if type == 'M'
        % massa totale [kg]
        m = 30;
        % added mass
        tau_a = -25;
        % Linear damping
        tau_r = -0.2;
        % Quadratic damping
        tau_d = -19.5;
    end
    mv = m - tau_a;
    % Dissipative forces (con v0 = 0.1 solo nel surge)
    dv = -tau_r - tau_d.*abs(speed0);
    dv_lin = -tau_r - 2*tau_d.*abs(speed0);

    %% Computation (Equazione giusta, problema con controllore)
    kp = 2*damp*wn*mv - dv_lin; 
    ki = wn^2 * mv;
    Ti = kp/ki;
    % Td = kd/kp;
    Kt = 1/Ti;
end