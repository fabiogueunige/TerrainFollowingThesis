function [H] = jacobianH_position(dim_s, dim_ekf)
    % H =   eta, nu, bias
    %    DVL
    %    AHRS
    %    AHRS (gyro)
    %    PS

    % Initialize H matrix to zeros
    H = zeros(sum(dim_s), dim_ekf);

    H(1:3, 7:9) = eye(3);   % DVL
    H(4:6, 4:6) = eye(3);   % AHRS
    H(7:9, 10:12) = eye(3); % AHRS gyro part
    H(7:9, 13:15) = eye(3); % AHRS bias part
    H(10, 3) = 1;           % PS
end