clc; clear; close all;
addpath('rotation');
%% Definition
IND_H = 1;                  I_IND_U = 1;
ALPHA = 2;                  I_IND_W = 2;
BETA = 3;
PHI = 4;      M_PHI = 5;
THETA = 5;    M_THETA = 6;
IND_P = 6;    M_IND_P = 7;  I_IND_P = 3;
IND_Q = 7;    M_IND_Q = 8;  I_IND_Q = 4;    

%% Filter Parameters
Ts = 0.001;         % Sampling time [s]
Tf = 20;            % Final time [s]
time = 0:Ts:Tf;     % Time vector
N = length(time);   % Number of iterations

%% Matrix Dimensions
n_dim = 7;          % Number of states
m_dim = 8;          % Number of measurements
l_dim = 4;          % Number of inputs % (no v for now)
d_dim = 3;          % world space total dimensions

%% Noise
[Q, G, R] = noise_setup(n_dim, m_dim, Ts);

%% EKF Prameters
prob = zeros(d_dim,N);         % AUV initial position
x_est = zeros(n_dim, N);       % Estimated state
x_true = zeros(n_dim, N);      % True state
z_meas = zeros(m_dim, N);      % Measurements
u = zeros(l_dim, N);           % Known inputs
tau_star = zeros(l_dim, N);    % Desired input
u_dot = zeros(l_dim,N);        % AUV acceleration
p_err = zeros(1,l_dim);        % Proportional error
i_err = zeros(1,l_dim);        % Integral Error
num_s = 4;                     % number of echosonar

%% Design Parameters
% Terrain
alpha = 0;
beta = 0;
pplane = [0, 0, 10]';
n0 = [0, 0, 1]'; % terrain frame
wRt = zeros(3,3);

% AUV Parameters 
psi = 0;                                   % Robot yaw
s = zeros(3,num_s);                        % Number of sensors
% x0 = [h, alpha, beta, phi, theta, p, q]
x0 = [9.5106, alpha, beta, 0, 0, 0, 0]';   % True initial state 
x0_est = zeros(n_dim, 1);                  % Estimated initial state
wRr = zeros(3,3);                          % robot in world rotation

Gamma = -pi/8;                             % y1 angle (rear)
Lambda = pi/8;                             % y2 angle (front)
Eta = -pi/8;                               % y3 angle (left)
Zeta = pi/8;                               % y4 angle (right)

% Control Parametrs
speed0 = [0, 0, 0, 0]'; % surge, heave, p and q.  % (sway)? %
% tau0 = tau0_values(speed0);

%% EKF Start
% covariance 
P0 = diag([1, 0.08, 2, 0.09, 1, 0.08, 2]);
P0 = P0 * (1.1);% + 2*rand);
P = P0;
% State
x_true(:,1) = x0;
x_est(:,1) = x0_est;

% Sensors definition
s(:, 1) = [-sin(Gamma + x_est(THETA,1)), 0, cos(Gamma + x_est(THETA,1))]';
s(:, 2) = [-sin(Lambda + x_est(THETA,1)), 0, cos(Lambda + x_est(THETA,1))]';
s(:, 3) = [0, -sin(Eta + x_est(PHI,1)), cos(Eta + x_est(PHI,1))]';
s(:, 4) = [0, -sin(Zeta + x_est(PHI,1)), cos(Zeta + x_est(PHI,1))]';

%% EKF Simulation
for k = 2:N
    fprintf('New Lap, time = %.0f \n', k);
    %% EKF: Input Control Computation
    if k == 2
        % desired input
        [tau_star(:,k-1), p_err, i_err] = input_control(x_est(:,k-1), Ts, p_err, i_err, speed0(1));
        % % Computation of the tau
        % tau = tau_generator(Ts, speed0, tau_star(:,k-1), tau0, speed0);
        % % Controller application
        % [u(:,k-1), u_dot(:,k-1)] = dynamic_model(tau, tau0, speed0, speed0, Ts);
        u(:,k-1) = tau_star(:,k-1); % tmp
    else
        [tau_star(:,k-1), p_err, i_err] = input_control(x_est(:,k-1), Ts, p_err, i_err, u(1,k-2)); % Define input
        % % Computation of the tau
        % tau = tau_generator(Ts, u(:,k-2), tau_star(:,k-1), tau0, speed0);
        % % Controller application
        % [u(:,k-1), u_dot(:,k-1)] = dynamic_model(tau, tau0, speed0, u(:,k-2), Ts);
        u(:,k-1) = tau_star(:,k-1); % tmp
    end

    %% EKF: Real Measurement
    v = mvnrnd(zeros(m_dim,1), R)'; % Measurement noise
 
    [z_meas(:,k), hmes, prob(:,k)] = measurament(alpha, beta, pplane, n0, Gamma, Lambda, Eta, Zeta, s, num_s, ...
                                                u(:,k-1), Ts, prob(:,k-1), z_meas(M_PHI,k-1), z_meas(M_THETA,k-1));
    z_meas(:,k) = z_meas(:,k) + v;

    %% EKF: True State update
    w = mvnrnd(zeros(n_dim,1), Q)'; % Process noise
    x_true(:,k) = [hmes, alpha, beta, alpha, beta, 0, 0]';%  +w; % final scope

    %% EKF: Prediction
    % per chat non ci vuole w nella x_pred !!!!
    % State prediction
    [x_pred, wRt, wRr] = f(x_est(:,k-1), u(:,k-1), Ts, psi); % State prediction
    F = jacobian_f(x_est(:,k-1), u(:,k-1), Ts, n_dim, psi, wRt, wRr); % Dynamics Jacobian
    P_pred = F * P * F' + Q;

    %% Sensors Computation
    % Expected sensor values for the expected output
    s(:, 1) = [-sin(Gamma + x_pred(THETA)), 0, cos(Gamma + x_pred(THETA))]';
    s(:, 2) = [-sin(Lambda + x_pred(THETA)), 0, cos(Lambda + x_pred(THETA))]';
    s(:, 3) = [0, -sin(Eta + x_pred(PHI)), cos(Eta + x_pred(PHI))]';
    s(:, 4) = [0, -sin(Zeta + x_pred(PHI)), cos(Zeta + x_pred(PHI))]';
    for j = 1:num_s
        if (norm(s(:,j)) ~= 1)
            fprintf('ERROR: norm of the predicted sensor j = %.0f is not 1', j);
        end
    end

    %% Norm to the terrain
    n = wRt*n0;
    if (norm(n) ~= 1)
        fprintf('\nVettore superficie\n');
        fprintf('n_yx: [%.4f; %.4f; %.4f]\n', n(1), n(2), n(3));
        fprintf('ALERT: norm n is not 1\n');
    end
   
    %% EKF: Update
    H = jacobian_h(x_pred, s, m_dim, n_dim, num_s, n, n0, Gamma, Lambda, Eta, Zeta);   % Observation Jacobian
    if any(isnan(H(:)))
        H
        error('Esecuzione interrotta: H contiene valori NaN.');
    end
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman Gain
    if any(isnan(K(:)))
        K
        error('Esecuzione interrotta: K contiene valori NaN.');
    end
    
    % Measurament prediction
    z_pred = h(x_pred, s, num_s, m_dim, wRt, n);      

    %% EKF: State Update
    x_est(:,k) = x_pred + K * (z_meas(:,k) - z_pred);
    P = (eye(n_dim) - K * H) * P_pred;
end


%% Final Plot
for i = 1:n_dim
    figure;
    plot(time, x_true(i,:), 'b', 'DisplayName', 'True');
    hold on;
    plot(time, x_est(i,:), 'r--', 'DisplayName', 'Estimated');
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
    legend; grid on;
    hold off;
end

%% Points plot
figure;
scatter(prob(1,:), prob(2,:), prob(3,:), [], time);
colorbar; 
colormap(jet);
xlabel('On X');
ylabel('On Z');
title('Trajectory XYZ (Color = time)');
grid on;
