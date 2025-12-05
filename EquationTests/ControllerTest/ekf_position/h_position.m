function [y] = h_position(x, dim_meas)
    %% DVL
    y = zeros(sum(dim_meas),1);
    y(1:3) = x(7:9);  % DVL measures linear velocities in body frame

    %% AHRS
    y(4:6) = x(4:6); % AHRS measures Euler angles (roll, pitch, yaw)

    %% AHRS angular velocities with Gyro
    y(7:9) = x(10:12) + x(13:15); % AHRS measures angular velocities in body frame

    %% sigma_depth
    y(10) = x(3); % Pressure sensor measures depth (heave position)
end

