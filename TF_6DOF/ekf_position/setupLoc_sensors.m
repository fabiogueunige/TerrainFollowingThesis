function [R_loc] = setupLoc_sensors()
    % Sensor noise parameters (very low for accurate tracking)
    sigma_AHRS = 0.5;   
    sigma_AHRSG = 0.9;  
    sigma_DVL  = 0.2;   
    sigma_depth = 0.03;  
    R_loc = diag([sigma_DVL^2, sigma_DVL^2, sigma_DVL^2, ...
                sigma_AHRS^2, sigma_AHRS^2, sigma_AHRS^2, ...
                sigma_AHRSG^2, sigma_AHRSG^2, sigma_AHRSG^2, ...
                sigma_depth^2]);
end

