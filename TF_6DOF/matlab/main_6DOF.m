clc; clear; close all;

%% Filter Parameters
Ts = 0.001;       % Sampling time [s]
Tf = 20;          % Final time [s]
time = 0:Ts:Tf;   % Time vector
N = length(time); % Number of iterations

%% Design Parameters
% Terrain
beta = pi/10;
qt = - 10; % For line construction
% Robot 
theta = 0;
% (pi/10 =9.5106, pi/7 =9.0097, pi/4 =7.0711)
x0 = [9.5106, beta, theta, 0]';      % True initial state 

% Noise
sig1 = 0.077; % State noise h
sig2 = 0.085; % State noise beta
sig3 = 0.083; % State noise theta
sig4 = 0.097; % State noise q
eta1 = 0.005; % Measurement noise y1
eta2 = 0.005; % Measurement noise y2
eta3 = 0.025; % Measurement noise theta

% AUV Parameters
Gamma = -pi/8; % Sonar angle y1
Lambda = pi/8; % Sonar angle y2
p_err = zeros(1,2);
i_err = zeros(1,2);

% AUV Model Parameters
speed0 = [0, 0, 0]'; % surge, heave and pitch
tau0 = tau0_values(speed0);

%% Matrix Dimensions
n = 4; % Number of states
m = 3; % Number of measurements
l = 3; % Number of inputs
q = 4; % Process noise dimension
r = 3; % Measurement noise dimension

%% Initialization
prob = zeros(2,N);         % AUV initial position
x_est = zeros(n, N);       % Estimated state
x_true = zeros(n, N);      % True state
z_meas = zeros(m, N);      % Measurements
u = zeros(l, N);           % Known inputs
u_star = zeros(l, N);      % Desired input

% Initial state
x0_est = [0, 0, 0, 0]';          % Estimated initial state

% Initial covariance matrix
P0 = diag([1, 0.08, 0.07, 0.09]);
P0 = P0 * (1.1);% + 2*rand);

x_true(:,1) = x0;
x_est(:,1) = x0_est;
P = P0;

% Process noise covariance (n x n)
Q = zeros(n,n);
Q(1,1) = (sig1^2);
Q(2,2) = (sig2^2);
Q(3,3) = (sig3^2);
Q(4,4) = (sig4^2);

% G coefficient noise matrix
G = eye(n) * Ts;

% Measurement noise covariance (m x m)
R = zeros(m,m);
R(1,1) = (eta1^2);
R(2,2) = (eta2^2);
R(3,3) = (eta3^2);

%% EKF Simulation
for k = 2:N
    %% Control Input
    [u_star(:,k-1), pid_q, p_err, i_err] = input_control(x_est(:,k-1), Ts, p_err, i_err); % Define input

    if k == 2
        tau = tau_generator(Ts, speed0, u_star(:,k-1), tau0, speed0, pid_q);
        [u(:,k-1), theta] = dynamic_model(tau, tau0, speed0, speed0, Ts, z_meas(3,k-1));
    else
        tau = tau_generator(Ts, u(:,k-2), u_star(:,k-1), tau0, speed0, pid_q);
        [u(:,k-1), theta] = dynamic_model(tau, tau0, speed0, u(:,k-2), Ts, z_meas(3,k-1));
    end


    %% Measurement
    v = mvnrnd(zeros(m,1), R)'; % Measurement noise
    [z_meas(:,k), hmes, prob(:,k)] = measurament(x_est(:,k-1), beta, qt, Gamma, Lambda, u(:,k-1), Ts, prob(:,k-1), theta);
    z_meas(:,k) = z_meas(:,k) + v;

    %% True Dynamic Simulation
    x_true(:,k) = [hmes, beta, z_meas(3,k), 0]';

    %% --- EKF: Prediction ---
    w = mvnrnd(zeros(n,1), Q)'; % Process noise
    x_pred = f(x_est(:,k-1), u(:,k-1), Ts) + w; % State prediction with noise state
    F = jacobian_f(x_est(:,k-1), u(:,k-1), Ts); % Dynamics Jacobian
    P_pred = F * P * F' + Q;
   
    %% --- EKF: Update ---
    H = jacobian_h(x_pred, Gamma, Lambda);   % Observation Jacobian
    if any(isnan(H(:)))
        H
        error('Esecuzione interrotta: H contiene valori NaN.'); % Stop execution and show error
    end
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman Gain
    if any(isnan(K(:)))
        K
        error('Esecuzione interrotta: K contiene valori NaN.'); % Stop execution and show error
    end
    z_pred = h(x_pred, Gamma, Lambda);        % Predicted measurement
    x_est(:,k) = x_pred + K * (z_meas(:,k) - z_pred);
    P = (eye(n) - K * H) * P_pred;
    fprintf('New Lap \n');
end

%% Final Plot (Example)
for i = 1:n
    figure;
    plot(time, x_true(i,:), 'b', 'DisplayName', 'True');
    hold on;
    plot(time, x_est(i,:), 'r--', 'DisplayName', 'Estimated');
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
    legend; grid on;
    hold off;
end