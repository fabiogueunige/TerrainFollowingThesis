function [Q] = ekf_setup_sbes()
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
    
    % Build covariance matrices (diagonal)
    Q = diag([sigma_altitude^2, sigma_alpha^2, sigma_beta^2]);
end

