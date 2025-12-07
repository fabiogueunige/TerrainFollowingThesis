function [Q_s, P0] = stateSbes_init()
    % Initial state covariance - MUST be diagonal, no random!
    % Large initial uncertainty since we start with unknown state
    P0 = diag([4.0, 0.5, 0.5]);  % [altitude^2, alpha^2, beta^2]
    % Note: Removed random component that was corrupting the matrix

    % Process noise (model uncertainty)
    % Tuning: Higher Q = more trust in measurements, lower Q = more trust in model
    sigma_altitude = 0.15;         % Altitude noise [m]
    sigma_alpha    = deg2rad(2);   % Roll angle noise [rad]  
    sigma_beta     = deg2rad(2);   % Pitch angle noise [rad]

    % Build covariance matrices (diagonal)
    Q_s = diag([sigma_altitude^2, sigma_alpha^2, sigma_beta^2]);
end