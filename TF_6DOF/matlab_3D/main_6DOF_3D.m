clc; clear; close all;
addpath('rotation');
addpath('visualization');
addpath('math_function');

%% Definition
IND_H = 1;                  
ALPHA = 2;                  
BETA = 3;
PHI = 4;        M_PHI = 5;    
THETA = 5;      M_THETA = 6;  
I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;

%% Filter Parameters
Ts = 0.001;         % Sampling time [s]
Tf = 25;            % Final time [s]
time = 0:Ts:Tf;     % Time vector
N = length(time);   % Number of iterations

%% Matrix Dimensions
n_dim = 5;          % Number of states
m_dim = 6;          % Number of measurements
i_dim = 4;          % Number of inputs % (no v for now)
d_dim = 3;          % world space total dimensions
s_dim = 4;          % number of echosonar

%% Noise
[Q, G, R] = noise_setup(n_dim, m_dim, Ts);
% Q = G*Q*G'

%% EKF Prameters
prob = zeros(d_dim,N);         % AUV initial position
x_pred = zeros(n_dim, N);      % State predicted
x_est = zeros(n_dim, N);       % Estimated state
x_true = zeros(n_dim, N);      % True state
z_meas = zeros(m_dim, N);      % Measurements
u = zeros(i_dim, N);           % Known inputs
tau_star = zeros(i_dim, N);    % Desired input
u_dot = zeros(i_dim,N);        % AUV acceleration
z_pred = zeros(m_dim, N);      % Predicted output
n = zeros(d_dim,N);            % Surface vector
p_err = zeros(i_dim, N);       % Proportional error
i_err = zeros(i_dim, N);       % Integral error

%% Software Design
h_ref = 7;

%% Terrain Parameters
alpha = pi/10;
beta = pi/10;
pplane = [0, 0, 10]';
n0 = [0, 0, 1]'; % terrain frame
wRt = zeros(d_dim, d_dim, N);
wRt_pre = zeros(d_dim, d_dim, N);

%% AUV Parameters 
psi = 0;                                   % Robot yaw (may be controlled in fututre)
s = zeros(d_dim,s_dim);                    % Robot echosonar
% x0 = [h, alpha, beta, phi, theta, p, q]
x0 = [10, alpha, beta, 0, 0]';             % True initial state 
x0_est = zeros(n_dim, 1);                  % Estimated initial state
wRr = zeros(d_dim, d_dim, N);              % robot in world rotation
wRr_pre = zeros(d_dim, d_dim, N);          % robot in world predicted rotation

Gamma = -pi/8;                             % y1 angle (rear)
Lambda = pi/8;                             % y2 angle (front)
Eta = pi/8;                                % y3 angle (left)
Zeta = -pi/8;                              % y4 angle (right)

r_s(:, 1) = [sin(Gamma), 0, cos(Gamma)]';  % y1 versor (rear)
r_s(:, 2) = [sin(Lambda), 0, cos(Lambda)]';% y2 versor (front)
r_s(:, 3) = [0, -sin(Eta), cos(Eta)]';     % y3 versor (left)
r_s(:, 4) = [0, -sin(Zeta), cos(Zeta)]';   % y4 versor (right)

% Second check visibility parameters
z_r = zeros(d_dim, s_dim);
z_r(:,1) = [1, 0, 1];
z_r(:,2) = [1, 0, 1];
z_r(:,3) = [0, 1, 1];
z_r(:,4) = [0, 1, 1];

for j = 1:s_dim
    z_r(:,j) = vector_normalization(z_r(:,j));
end
% Control Parametrs
speed0 = [0, 0, 0, 0]'; % surge, heave, p and q.  % (sway)? %
tau0 = tau0_values(speed0, i_dim);

%% EKF Start
% covariance 
P0 = diag([1, 0.08, 2, 0.09, 1]);
P0 = P0 * (1.1)+ 3*rand;
P = P0;
% State
x_true(:,1) = x0;
x_est(:,1) = x0_est;
wRr(:,:,1) = eye(d_dim);
wRr_pre(:,:,1) = wRr(:,:,1);
wRt(:,:,1) = eye(d_dim) * rotx(pi);
wRt_pre(:,:,1) = wRt(:,:,1);

%% EKF Simulation
for k = 2:N
    fprintf('\n New Lap, time = %.0f \n', k);
    %% EKF: Input Control Computation
    if k >= 100
        % Desired input
        [tau_star(:,k-1), p_err(:,k), i_err(:,k)] = input_control(x_est(:,k-1), Ts, p_err(:,k-1), i_err(:,k-1), u(I_IND_U,k-2), ...
                                                                    wRr(:,:,k-1), wRt(:,:,k-1)); % Define input
        % Tau to generate the desired inptut
        tau = tau_generator(Ts, u(:,k-2), tau_star(:,k-1), tau0, speed0, i_dim);
        % Controller application
        [u(:,k-1), u_dot(:,k-1)] = dynamic_model(tau, tau0, speed0, u(:,k-2), Ts, i_dim);
    end

    %% EKF: Real Measurement
    % Measurement noise
    v = mvnrnd(zeros(m_dim,1), R)'; 
    [z_meas(:,k), hmes, prob(:,k)] = measurament(alpha, beta, pplane, n0, r_s, s_dim, u(:,k-1), Ts, prob(:,k-1), ...
                                                   z_meas(M_PHI,k-1), z_meas(M_THETA,k-1), k, psi, z_r);
    z_meas(:,k) = z_meas(:,k) + v;

    %% EKF: True State update
    % w = mvnrnd(zeros(n_dim,1), Q)'; % Process noise
    % Desired state estimation (real one)
    x_true(:,k) = [h_ref, alpha, beta, alpha, beta]'; % z_meas(M_PHI,k), z_meas(M_THETA,k) , u(I_IND_P,k-1), u(I_IND_Q,k-1) 

    %% EKF: Prediction
    % per chat non ci vuole w nella x_pred !!!!
    % State prediction
    [x_pred(:,k), wRt_pre(:,:,k), wRr_pre(:,:,k)] = f(x_est(:,k-1), u(:,k-1), Ts, psi);
    % Dynamics Jacobian
    F = jacobian_f(x_est(:,k-1), u(:,k-1), Ts, n_dim, psi, wRt_pre(:,:,k), wRr_pre(:,:,k)); 
    % Covariance prediction
    P_pred = F * P * F' + Q;

    %% Sensors Computation
    % Expected sensor values for the expected output
    for j = 1:s_dim
        % Update sensor value
        s(:,j) = wRr_pre(:,:,k) * r_s(:,j);
        if (norm(s(:,j)) ~= 1)
            fprintf('ALERT: Norm Predicted SENSOR %.0f has been normalized\n', j);
            s(:,j) = vector_normalization(s(:,j));
        end
    end

    %% Norm to the terrain
    n(:,k) = wRt_pre(:,:,k)*n0;
    if (norm(n(:,k)) ~= 1)
        fprintf('ALERT: Norm predicted has been normalized\n');
        n(:,k) = vector_normalization(n(:,k));
        fprintf('n: [%.4f; %.4f; %.4f]\n', n(1,k), n(2,k), n(3,k));
    end
   
    %% EKF: Gain and Prediction Update
    H = jacobian_h(x_pred(:,k), s, m_dim, n_dim, s_dim, n(:,k), n0, r_s, psi);   % Observation Jacobian
    if any(isnan(H(:)))
        error('Esecuzione interrotta: H contiene valori NaN. Istante %0.f', k);
    end
    fprintf('       Kalman Gain\n');
    K = P_pred * H' / (H * P_pred * H' + R); % Kalman Gain
    if any(isnan(K(:)))
        error('Esecuzione interrotta: K contiene valori NaN. Istante %0.f', k);
    end
    
    % Measurament prediction
    z_pred(:,k) = h(x_pred(:,k), s, s_dim, m_dim, n(:,k));      

    %% EKF: State Update
    x_est(:,k) = x_pred(:,k) + K * (z_meas(:,k) - z_pred(:,k));
    P = (eye(n_dim) - K * H) * P_pred;

    %% Rotation Update 
    wRt(:,:,k) = rotz(0)*roty(x_est(BETA))*rotx(x_est(ALPHA))*rotx(pi);

    % robot
    wRr(:,:,k) = rotz(psi)*roty(x_est(THETA))*rotx(x_est(PHI));
end

%% Final Plot
ttl = {'altitude', 'alpha', 'beta', 'phi', 'pitch'};
for i = 1:n_dim
    figure;
    plot(time, x_true(i,:), 'b', 'DisplayName', 'True');
    hold on;
    plot(time, x_est(i,:), 'g', 'DisplayName', 'Estimated');
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
    legend; grid on;
    title(ttl{i})
    hold off;
end

ttl = {'u input', 'w input', 'p input', 'q input'};
for i = 1:i_dim
    figure;
    plot(time, u(i,:), 'b', 'DisplayName', ttl{i});
    hold on
    xlabel('Time [s]'); ylabel('Space [m]');
    grid on;
    title(ttl{i})
    hold off;
end

%% Points plot
figure;
scatter3(prob(1,:), prob(2,:), prob(3,:), [], time);
colorbar; 
colormap(jet);
xlabel('On X');
ylabel('On Z');
title('Trajectory XYZ (Color = time)');
grid on;
