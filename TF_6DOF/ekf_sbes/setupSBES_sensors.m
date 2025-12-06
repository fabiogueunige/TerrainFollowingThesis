function [R_SBES] = setupSBES_sensors()

    % SBES measurement noise standard deviations
    eta_sensor1 = 0.00177;  % Rear sensor noise [m]
    eta_sensor2 = 0.00185;  % Front sensor noise [m]
    eta_sensor3 = 0.00177;  % Left sensor noise [m]
    eta_sensor4 = 0.00185;  % Right sensor noise [m]

    R_SBES = diag([eta_sensor1^2, eta_sensor2^2, eta_sensor3^2, eta_sensor4^2]);
end

