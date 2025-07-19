clc; clear; close all;
addpath('rotation');
addpath('visualization');
addpath('math_function');

%% Definition
IND_H = 1;      ALPHA = 2;      BETA = 3;  
global PHI; global THETA; global PSI;          
PHI = 1;        THETA = 2;      PSI = 3;  
global SURGE; global SWAY; global HEAVE;
SURGE = 1;  SWAY = 2;   HEAVE = 3;
global ROLL; global PITCH; global YAW;
ROLL = 4;   PITCH = 5;  YAW = 6;

%% Filter Parameters
Ts = 0.001;         % Sampling time [s]
Tf = 25;            % Final time [s]
time = 0:Ts:Tf;     % Time vector
N = length(time);   % Number of iterations

global DEBUG;
DEBUG = false;

%% Matrix Dimensions
n_dim = 3;          % Number of states
m_dim = 4;          % Number of measurements
i_dim = 5;          % Number of inputs 
d_dim = 3;          % world space total dimensions
s_dim = 4;          % number of echosonar
a_dim = 3;          % number of angles

%% Noise
[Q, R_tp, R_a] = noise_setup(n_dim, m_dim, a_dim);

%% Terrain Parameters
alpha = pi/10;
beta = pi/7;
pplane = [0, 0, 20]';
n0 = [0, 0, 1]'; % terrain frame
wRt = zeros(d_dim, d_dim, N);
wRt_pre = zeros(d_dim, d_dim, N);
n = zeros(d_dim,N);                 % Surface vector

%% AUV Parameters 
psi = 0; % yaw to get motion
prob = zeros(d_dim,N);              % AUV initial position

% robot velocities & acceleration
u = zeros(i_dim, N);                % Known inputs
u_dot = zeros(i_dim,N);             % AUV acceleration

% robot angles
a_true = zeros(a_dim, N);           % True angles comparation
rob_rot = zeros(a_dim, N);          % Robot angles roll, pitch, yaw
clean_rot = zeros(a_dim,N);         % NO Noise Rotation

% robot rotations
wRr = zeros(d_dim, d_dim, N);       % Robot in world rotation
wRr_real = zeros(d_dim, d_dim);     % Real robot rotation

% echosonar part
s = zeros(d_dim,s_dim);                     % Robot echosonar
Gamma = -pi/8;                              % y1 angle (rear)
Lambda = pi/8;                              % y2 angle (front)
Eta = pi/8;                                 % y3 angle (left)
Zeta = -pi/8;                               % y4 angle (right)
r_s(:, 1) = [sin(Gamma), 0, cos(Gamma)]';   % y1 versor (rear)
r_s(:, 2) = [sin(Lambda), 0, cos(Lambda)]'; % y2 versor (front)
r_s(:, 3) = [0, -sin(Eta), cos(Eta)]';      % y3 versor (left)
r_s(:, 4) = [0, -sin(Zeta), cos(Zeta)]';    % y4 versor (right)

%% EKF Parameters
x_pred = zeros(n_dim, N);      % State predicted
x_est = zeros(n_dim, N);       % Estimated state
x_true = zeros(n_dim, N);      % True state
z_meas = zeros(m_dim, N);      % Measurements
z_pred = zeros(m_dim, N);      % Predicted output
R = zeros(m_dim, m_dim, N);    % Observation matrix
x0 = [10, alpha, beta]';       % True initial state 
x0_est = zeros(n_dim, 1);      % Estimated initial state
ni = zeros(m_dim, N);          % Innovation
S = zeros(m_dim, m_dim, N);    % Covariance Innovation

%% Controller Parameters
pid = zeros(i_dim, N);              % PID for Dynamics
integral_err = zeros(i_dim, N);     % integral error
p_err = zeros(i_dim, N);            % proportional error
i_err = zeros(i_dim, N);            % integral error
t_sum = zeros(i_dim,N);

% initial controller
speed0 = [0; 0; 0; 0; 0];
tau0 = tau0_values(speed0, i_dim);

%% Software Design
global h_ref;
h_ref = 7;

%% EKF Start
% covariance 
P0 = diag([1, 0.08, 2]);
P0 = P0 * (1.1) + 5*rand;
P = P0;

% State
x_true(:,1) = x0;
x_est(:,1) = x0_est;

% Rotations
wRr(:,:,1) = eye(d_dim);
wRt(:,:,1) = eye(d_dim) * rotx(pi);
wRt_pre(:,:,1) = wRt(:,:,1);

%% EKF Simulation
for k = 2:N
    if k == 2 || mod(k, 100) == 0
        disp(['Processing iteration \n', num2str(k)]);
    end
    %% R setting
    R(:,:,k) = R_tp;
    %% EKF: Input Control Computation
    if k >= 10
        % PID Values with Saturation
        [pid(:,k), integral_err(:,k), p_err(:,k), i_err(:,k), u_dot(:,k-1), t_sum(:,k)] = input_control(x_est(:,k-1), rob_rot(:,k-1), pid(:,k-1), integral_err(:,k-1), ...
                         u(:,k-2), u(:,k-3), u_dot(:,k-2), t_sum(:,k-1), speed0, wRr(:,:,k-1), wRt(:,:,k-1), Ts, i_dim, p_err(:,k-1), i_err(:,k-1));
        % Dynamic model
        [u(:,k-1)] = dynamic_model(pid(:,k), tau0, speed0, u(:,k-2), Ts, i_dim, u_dot(:,k-2));
    end

    %% EKF: Real Measurement
    % Measurement noise
    v = mvnrnd(zeros(m_dim,1), R_tp)'; 
    v_a = mvnrnd(zeros(a_dim,1), R_a)';
    [clean_rot(:,k), wRr_real] = AHRS_measurement(clean_rot(:,k-1), u(:,k-1), Ts);
    [z_meas(:,k), hmes, prob(:,k), R(:,:,k)] = measurament(alpha, beta, pplane, n0, r_s, s_dim, u(:,k-1), Ts,  ...
                                                   prob(:,k-1), wRr_real, k, R(:,:,k));
    % Adding noise to real measurament
    %%%%%%%%%%%%%%% NO NOISE %%%%%%%%%%%%%%%%%%%%%%%
    % rob_rot(:,k) = clean_rot(:,k);
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%
    rob_rot(:,k) = clean_rot(:,k) + v_a;
    z_meas(:,k) = z_meas(:,k) + v;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Robot rotation matrix computation
    wRr(:,:,k) = rotz(rob_rot(PSI,k))*roty(rob_rot(THETA,k))*rotx(rob_rot(PHI,k)); 

    %% EKF: True State update
    % TRUE state estimation
    x_true(:,k) = [h_ref, alpha, beta]';  
    a_true(:,k) = [alpha, beta, 0]'; % True angles

    %% EKF: Prediction
    % State prediction
    w = mvnrnd(zeros(n_dim,1), Q)'; % Process noise
    [x_pred(:,k), wRt_pre(:,:,k)] = f(x_est(:,k-1), u(:,k-1), Ts, wRr(:,:,k));
    %%%%%%%%%%%%%%% NO NOISE %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%
    x_pred(:,k) = x_pred(:,k) + w;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Dynamics Jacobian
    F = jacobian_f(x_est(:,k-1), u(:,k-1), Ts, n_dim, wRr(:,:,k)); 
    % Covariance prediction
    P_pred = F * P * F' + Q;

    %% Sensors Computation
    % Expected sensor values for the expected output
    for j = 1:s_dim
        % Update sensor value
        s(:,j) = wRr(:,:,k) * r_s(:,j);
        if (norm(s(:,j)) ~= 1)
            printDebug('ALERT: Norm Predicted SENSOR %.0f has been normalized\n', j)
            s(:,j) = vector_normalization(s(:,j));
        end
    end

    %% Norm to the terrain
    n(:,k) = wRt_pre(:,:,k)*n0;
    if (norm(n(:,k)) ~= 1)
        n(:,k) = vector_normalization(n(:,k));
        printDebug('ALERT: n normalized: [%.4f; %.4f; %.4f]\n', n(1,k), n(2,k), n(3,k));
    end
   
    %% EKF: Gain and Prediction Update
    % Observation Jacobian
    H = jacobian_h(x_pred(:,k), s, m_dim, n_dim, s_dim, n(:,k), n0);
    if any(isnan(H(:)))
        error('Esecuzione interrotta: H contiene valori NaN. Istante %0.f', k);
    end
    % Covariance Innovation
    S(:,:,k) = (H * P_pred * H' + R(:,:,k));
    % Kalman Gain
    K = P_pred * H' / S(:,:,k); % Kalman Gain
    if any(isnan(K(:)))
        error('Esecuzione interrotta: K contiene valori NaN. Istante %0.f', k);
    end
    
    % Measurament prediction
    z_pred(:,k) = h(x_pred(:,k), s, s_dim, m_dim, n(:,k));      

    %% EKF: State Update
    % State 
    % Innovation
    ni(:,k) = (z_meas(:,k) - z_pred(:,k));
    % State Estimated
    x_est(:,k) = x_pred(:,k) + K * ni(:,k);
    % Covariance
    P = (eye(n_dim) - K * H) * P_pred;

    %% Rotation Update 
    % terrain
    wRt(:,:,k) = rotz(0)*roty(x_est(BETA))*rotx(x_est(ALPHA))*rotx(pi);
end

%% Plotting
% States
ttl = {'altitude', 'alpha', 'beta'};
for i = 1:n_dim
    figure;
    if (i == 1)
        plot(time, x_true(i,:), 'b', 'DisplayName', 'True');
        hold on;
        plot(time, x_est(i,:), 'g', 'DisplayName', 'Estimated');
    else
        plot(time, rad2deg(x_true(i,:)), 'b', 'DisplayName', 'True');
        hold on;
        plot(time, rad2deg(x_est(i,:)), 'g', 'DisplayName', 'Estimated');
    end
    
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
    legend; grid on;
    title(ttl{i})
    hold off;
end

% Robot angles
ttl = {'roll', 'pitch', 'yaw'};
for i = 1:a_dim
    figure;
    plot(time, rad2deg(a_true(i,:)), 'b', 'DisplayName', 'True');
    hold on;
    plot(time, rad2deg(rob_rot(i,:)), 'g', 'DisplayName', 'Estimated');
    xlabel('Time [s]'); ylabel(sprintf('x_%d', i));
    legend; grid on;
    title(ttl{i})
    hold off;
end

% Inputs
ttl = {'u input', 'v input', 'w input', 'p input', 'q input'};
for i = 1:i_dim
    figure;
    if i <= HEAVE
        plot(time, u(i,:), 'b', 'DisplayName', 'u');
    else
        plot(time, rad2deg(u(i,:)), 'b', 'DisplayName', ttl{i});
    end
    hold on
    xlabel('Time [s]'); ylabel('Space [m]');
    grid on;
    title(ttl{i})
    hold off;
end

% Points
figure;
scatter3(prob(1,:), prob(2,:), prob(3,:), [], time);
colorbar; 
colormap(jet);
xlabel('On X');
ylabel('On Z');
title('Trajectory XYZ (Color = time)');
grid on;
