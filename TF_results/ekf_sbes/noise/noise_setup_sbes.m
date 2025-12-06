function [Q, R_sbes, R_ahrs] = noise_setup_sbes(state_dim, meas_dim, angle_dim)
    % NOISE_SETUP Configure noise covariance matrices for EKF
    %
    % Inputs:
    %   state_dim  - Dimension of state vector (3)
    %   meas_dim   - Dimension of measurement vector (4 SBES sensors)
    %   angle_dim  - Dimension of angle measurements (3: roll, pitch, yaw)
    %
    % Outputs:
    %   Q          - Process noise covariance matrix (state_dim x state_dim)
    %   R_sbes     - SBES measurement noise covariance (meas_dim x meas_dim)
    %   R_ahrs     - AHRS angle measurement noise covariance (angle_dim x angle_dim)
    %
    % Noise parameters tuned empirically for BlueROV2 in underwater terrain following
    
    % Process noise standard deviations
    sigma_altitude = 0.099;          % Altitude noise [m]
    sigma_alpha    = deg2rad(0.55);  % Roll angle noise [rad]
    sigma_beta     = deg2rad(0.5);   % Pitch angle noise [rad]
    
    % SBES measurement noise standard deviations
    eta_sensor1 = 0.177;  % Rear sensor noise [m]
    eta_sensor2 = 0.185;  % Front sensor noise [m]
    eta_sensor3 = 0.177;  % Left sensor noise [m]
    eta_sensor4 = 0.185;  % Right sensor noise [m]
    
    % AHRS angle measurement noise standard deviations
    sigma_roll_meas  = deg2rad(0.5);   % Roll measurement noise [rad]
    sigma_pitch_meas = deg2rad(0.5);   % Pitch measurement noise [rad]
    sigma_yaw_meas   = deg2rad(0.08);  % Yaw measurement noise [rad]
    
    % Build covariance matrices (diagonal)
    Q = diag([sigma_altitude^2, sigma_alpha^2, sigma_beta^2]);
    
    R_sbes = diag([eta_sensor1^2, eta_sensor2^2, eta_sensor3^2, eta_sensor4^2]);
    
    R_ahrs = diag([sigma_roll_meas^2, sigma_pitch_meas^2, sigma_yaw_meas^2]);
end

