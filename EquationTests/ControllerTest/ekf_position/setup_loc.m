function [Q_km1, R_loc, P0_loc] = setup_loc()
    % Sensor noise parameters (very low for accurate tracking)
    sigma_AHRS = 0.5;    % rad (reduced 10x)
    sigma_DVL  = 0.2;    % m/s (reduced 10x)
    sigma_depth = 0.5;   % m (reduced 10x)
    R_loc = diag([sigma_DVL^2, sigma_DVL^2, sigma_DVL^2, ...
                sigma_AHRS^2, sigma_AHRS^2, sigma_AHRS^2, ...
                sigma_depth^2]);

    % Process noise covariance Q_{k-1} - very low for close tracking
    Q_km1 = diag([1^2, 1^2, 1^2, 1^2, 1^2, 1^2]);
    
    % Initial state covariance
    P0_loc = diag([1, 1, 1, 0.5, 0.5, 0.5]);
end

