clc; clear; close all;
%% Filter Parameters
Ts = 0.001;       % Sampling time [s]
Tf = 50;          % Final time [s]
time = 0:Ts:Tf;   % Time vector
N = length(time); % Number of iterations

%% Design Parameters
% Terrain
beta = pi/7;
qt = - 10; % For line construction

% Noise
sig1 = 0.15; % State noise h
sig2 = 0.15; % State noise beta
eta1 = 0.005; % Measurement noise y1
eta2 = 0.005; % Measurement noise y2

% AUV Parameters
Gamma = -pi/6; % Sonar angle y1
Lambda = pi/6; % Sonar angle y2
p_err = 0;
i_err = 0;

%% Matrix Dimensions
n = 2; % Number of states
m = 2; % Number of measurements
q = 2; % Process noise dimension
r = 2; % Measurement noise dimension

%% Initialization
prob = zeros(2,N);         % AUV initial position
x_est = zeros(n, N);       % Estimated state
x_true = zeros(n, N);      % True state
z_meas = zeros(m, N);      % Measurements
u = zeros(q, N);           % Known inputs

% Initial state
x0 = [9.0097, beta]';      % True initial state (pi/10 =9.5106, pi/7 =9.0097, pi/4 =7.0711)
x0_est = [0, 0]';          % Estimated initial state

% Initial covariance matrix
P0 = [1, 0;
      0, 0.8];
P0 = P0 * (1.1);% + 2*rand);

x_true(:,1) = x0;
x_est(:,1) = x0_est;
P = P0;

% Process noise covariance (n x n)
Q = zeros(n,n);
Q(1,1) = (sig1^2);
Q(2,2) = (sig2^2);

% G coefficient noise matrix
G = eye(n) * Ts;

% Measurement noise covariance (m x m)
R = zeros(m,m);
R(1,1) = (eta1^2);
R(2,2) = (eta2^2);

u(:,1) = [0, 0]';          % Starting control
% figure;

%% EKF Simulation
for k = 2:N
    %% Control Input
    [u(:,k-1), p_err, i_err] = input_control(x_est(:,k-1), Ts, p_err, i_err); % Define input

    %% Simulated Measurement
    v = mvnrnd(zeros(m,1), R)'; % Measurement noise
    [z_meas(:,k), hmes, prob(:,k)] = measurament(beta, qt, Gamma, Lambda, u(:,k-1), Ts, prob(:,k-1));
    z_meas(:,k) = z_meas(:,k) + v;

    %% True Dynamic Simulation
    w = mvnrnd(zeros(n,1), Q)'; % Process noise
    x_true(:,k) = [hmes, beta]' + w;

    %% --- EKF: Prediction ---
    x_pred = f(x_est(:,k-1), u(:,k-1), Ts); % State prediction
    F = jacobian_f(x_est(:,k-1), u(:,k-1), Ts); % Dynamics Jacobian
    P_pred = F * P * F' + Q;
    fprintf('P_pre = [%.2f , %.2f ;\n        %.2f , %.2f]\n', P_pred(1,1), P_pred(1,2), P_pred(2,1),P_pred(2,2)')

    %% --- EKF: Update ---
    H = jacobian_h(x_pred, Gamma, Lambda);   % Observation Jacobian
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman Gain
    fprintf('K = [%.2f , %.2f ;\n        %.2f , %.2f]\n', K(1,1), K(1,2), K(2,1), K(2,2)')
    z_pred = h(x_pred, Gamma, Lambda);        % Predicted measurement
    x_est(:,k) = x_pred + K * (z_meas(:,k) - z_pred);
    P = (eye(n) - K * H) * P_pred;
    fprintf('New Lap \n');
end

%% Final Plot (Example)
figure;
for i = 1:n
    subplot(n,1,i);
    plot(time, x_true(i,:), 'b', 'DisplayName', 'True');
    hold on;
    plot(time, x_est(i,:), 'r--', 'DisplayName', 'Estimated');
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
    legend; grid on;
end