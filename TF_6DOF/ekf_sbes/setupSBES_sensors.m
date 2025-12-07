function [R_SBES] = setupSBES_sensors()
    % SBES measurement noise standard deviations
    % Tuning factor to balance NIS consistency vs estimation accuracy
    % - Factor 1.0: NIS~372, RMSE~0.05m (original, overconfident)
    % - Factor 3.0: NIS~40, RMSE~0.15m (balanced)
    % - Factor 5.5: NIS~5, RMSE~0.77m (consistent but less accurate)
    tuning_factor = 3.0;  % Balanced trade-off
    
    % Base sensor noise (from sensor specs or calibration)
    eta_sensor1_base = 0.05;  % Rear sensor noise [m]
    eta_sensor2_base = 0.05;  % Front sensor noise [m]
    eta_sensor3_base = 0.05;  % Left sensor noise [m]
    eta_sensor4_base = 0.05;  % Right sensor noise [m]

    % Apply tuning factor
    eta_sensor1 = eta_sensor1_base * tuning_factor;
    eta_sensor2 = eta_sensor2_base * tuning_factor;
    eta_sensor3 = eta_sensor3_base * tuning_factor;
    eta_sensor4 = eta_sensor4_base * tuning_factor;

    R_SBES = diag([eta_sensor1^2, eta_sensor2^2, eta_sensor3^2, eta_sensor4^2]);
end

