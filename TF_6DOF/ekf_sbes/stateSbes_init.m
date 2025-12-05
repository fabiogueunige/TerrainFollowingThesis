function [Q_s, P0] = stateLoc_init()
    % Initial state covariance
    P0 = diag([1, 0.08, 2]);
    P0 = P0 * (1.1) + 2*rand;
    

    sigma_altitude = 0.099;          % Altitude noise [m]
    sigma_alpha    = deg2rad(0.55);  % Roll angle noise [rad]
    sigma_beta     = deg2rad(0.5);   % Pitch angle noise [rad]

    % Build covariance matrices (diagonal)
    Q_s = diag([sigma_altitude^2, sigma_alpha^2, sigma_beta^2]);
end