function [Q, P0] = stateLoc_init(dim_f)
    % Initial state covariance
    P0 = eye(dim_f) * 1.0;  % Reduced initial uncertainty (was 3)
    P0(12:15, 12:15) = eye(4) * 0.1; % Gyro biases start from 0

    q_pos_xy = 0.05;   % Horizontal position (reduced from 0.1)
    q_pos_z  = 0.02;   % Vertical position (more observable via PS)
    
    % Attitude: keep low, AHRS is accurate
    q_att   = 0.005;   % Reduced from 0.01
    
    % Velocity: REDUCED significantly (was too pessimistic)
    % DVL provides accurate velocity, don't need high process noise
    q_vel_xy = 0.5;    % Reduced from 2.0 - horizontal velocity
    q_vel_z  = 0.3;    % Vertical velocity (heave)
    
    % Angular rates: moderate uncertainty
    q_rates = 0.3;     % Reduced from 1.0
    
    % Gyro bias: very slow varying
    q_gyro = 0.0001;    % Reduced from 0.01

    % Process noise covariance Q
    Q = diag([...
        q_pos_xy, q_pos_xy, q_pos_z, ...   % Position (x, y, z)
        q_att, q_att, q_att, ...            % Attitude (phi, theta, psi)
        q_vel_xy, q_vel_xy, q_vel_z, ...    % Linear velocity (u, v, w)
        q_rates, q_rates, q_rates, ...      % Angular rates (p, q, r)
        q_gyro, q_gyro, q_gyro ...          % Gyro biases
    ]);
end