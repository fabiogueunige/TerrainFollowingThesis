function [R_loc] = setupLoc_sensors()
    % Sensor noise parameters (realistic values for simulation)
    % Standard deviations in natural units
    sigma_AHRS = 0.01;   % ~0.57 degrees - typical IMU accuracy
    sigma_AHRSG = 0.005;  % ~0.3 deg/s - typical gyroscope noise
    sigma_DVL  = 0.02;   % 2 cm/s - typical DVL accuracy
    sigma_depth = 0.01;  % 1 cm - typical depth sensor accuracy
    
    R_loc = diag([sigma_DVL^2, sigma_DVL^2, sigma_DVL^2, ...
                sigma_AHRS^2, sigma_AHRS^2, sigma_AHRS^2, ...
                sigma_AHRSG^2, sigma_AHRSG^2, sigma_AHRSG^2, ...
                sigma_depth^2]);
end

