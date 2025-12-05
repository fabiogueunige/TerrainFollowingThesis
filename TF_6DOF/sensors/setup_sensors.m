function [R_AHRS, R_DVL, R_PS, R_SBES] = setup_sensors()
    % Sensor noise parameters (very low for accurate tracking)
    eta_AHRS = 0.5;   
    eta_AHRS2 = 0.3;
    eta_DVL  = 0.2;    
    eta_depth = 0.5;  

    % SBES measurement noise standard deviations
    eta_sensor1 = 0.177;  % Rear sensor noise [m]
    eta_sensor2 = 0.185;  % Front sensor noise [m]
    eta_sensor3 = 0.177;  % Left sensor noise [m]
    eta_sensor4 = 0.185;  % Right sensor noise [m]

    R_AHRS = diag([eta_AHRS^2, eta_AHRS^2, eta_AHRS^2, ...
                    eta_AHRS2^2, eta_AHRS2^2, eta_AHRS2^2]);

    R_DVL = diag([eta_DVL^2, eta_DVL^2, eta_DVL^2]);

    R_PS = eta_depth^2;

    R_SBES = diag([eta_sensor1^2, eta_sensor2^2, eta_sensor3^2, eta_sensor4^2]);
end

