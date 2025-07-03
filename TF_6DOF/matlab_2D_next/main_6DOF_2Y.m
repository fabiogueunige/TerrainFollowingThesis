clc; clear; close all;

%% Filter Parameters
Ts = 0.001;       % Sampling time [s]
Tf = 50;          % Final time [s]
time = 0:Ts:Tf;   % Time vector
N = length(time); % Number of iterations

%% Matrix Dimensions
n = 4; % Number of states
m = 4; % Number of measurements
l = 3; % Number of inputs
q = 4; % Process noise dimension
r = 4; % Measurement noise dimension

%% Design Parameters
% Terrain
beta = pi/10;
qt = - 10; % For line construction
% Robot 
% (pi/10 =9.5106, pi/7 =9.0097, pi/4 =7.0711)
x0 = [9.5106, beta, 0, 0]';      % True initial state 

% Noise
sig1 = 0.077; % State noise h
sig2 = 0.085; % State noise beta
sig3 = 0.083; % State noise theta
sig4 = 0.097; % State noise q
eta1 = 0.077; % Measurement noise y1
eta2 = 0.085; % Measurement noise y2
eta3 = 0.065; % Measurement noise accelerometer
eta4 = 0.077; % Measurament noise gyroscope

% AUV Parameters
Gamma = -pi/6; % Sonar angle y1
Lambda = pi/6; % Sonar angle y2
p_err = zeros(1,l);
i_err = zeros(1,l);

% AUV Model Parameters
speed0 = [0, 0, 0]'; % surge, heave and pitch
tau0 = tau0_values(speed0);

%% Initialization
prob = zeros(2,N);         % AUV initial position
x_est = zeros(n, N);       % Estimated state
x_true = zeros(n, N);      % True state
z_meas = zeros(m, N);      % Measurements
u = zeros(l, N);           % Known inputs
tau_star = zeros(l, N);      % Desired input
u_dot = zeros(l,N);

% Initial state
x0_est = [0, 0, 0, 0]';          % Estimated initial state

% Initial covariance matrix
P0 = diag([1, 0.08, 2, 0.09]);
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
R(4,4) = (eta4^2);

%% EKF Simulation
for k = 2:N
    fprintf('New Lap, time = %.0f \n', k);
    if (k == 5000)
        beta = 0;
    end
    if (k == 10000)
        beta = pi/7;
    end
    if (k == 15000)
        beta = 0;
    end
    if (k == 40000)
        beta = -pi/7;
    end
    %% Control Input
    if k == 2
        [tau_star(:,k-1), p_err, i_err] = input_control(x_est(:,k-1), Ts, p_err, i_err, speed0(1)); % Define input
        tau = tau_generator(Ts, speed0, tau_star(:,k-1), tau0, speed0);
        [u(:,k-1), u_dot(:,k-1)] = dynamic_model(tau, tau0, speed0, speed0, Ts);
    else
        [tau_star(:,k-1), p_err, i_err] = input_control(x_est(:,k-1), Ts, p_err, i_err, u(1,k-2)); % Define input
        tau = tau_generator(Ts, u(:,k-2), tau_star(:,k-1), tau0, speed0);
        [u(:,k-1), u_dot(:,k-1)] = dynamic_model(tau, tau0, speed0, u(:,k-2), Ts);
    end
    %% Measurement
    v = mvnrnd(zeros(m,1), R)'; % Measurement noise
    [z_meas(:,k), hmes, prob(:,k)] = measurament(x_est(:,k-1), beta, qt, Gamma, Lambda,...
                                                u(:,k-1), Ts, prob(:,k-1), z_meas(3,k-1));

    % adding noise on pitch = (3) then measurament will probably fail!!
    % A lot dependant to measuraments
    v(3) = 0;
    z_meas(:,k) = z_meas(:,k) + v;

    %% True Dynamic Simulation
    w = mvnrnd(zeros(n,1), Q)'; % Process noise
    x_true(:,k) = [hmes, beta, beta, 0]';%  +w; % final scope

    %% --- EKF: Prediction ---
    % per chat non ci vuole w nella x_pred
    x_pred = f(x_est(:,k-1), u(:,k-1), Ts); %+ w; % State prediction with noise state
    F = jacobian_f(x_est(:,k-1), u(:,k-1), Ts); % Dynamics Jacobian
    P_pred = F * P * F' + Q;
   
    %% --- EKF: Update ---
    H = jacobian_h(x_pred, Gamma, Lambda);   % Observation Jacobian
    if any(isnan(H(:)))
        H
        error('Esecuzione interrotta: H contiene valori NaN.');
    end
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman Gain
    if any(isnan(K(:)))
        K
        error('Esecuzione interrotta: K contiene valori NaN.');
    end
    z_pred = h(x_pred, Gamma, Lambda);        % Predicted measurement
    x_est(:,k) = x_pred + K * (z_meas(:,k) - z_pred);
    P = (eye(n) - K * H) * P_pred;

end

%% Final Plot
for i = 1:n
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
scatter(prob(1,:), prob(2,:), [], time);
colorbar; 
colormap(jet);
xlabel('On X');
ylabel('On Z');
title('Trajectory XZ Plane (Color = time)');
grid on;
