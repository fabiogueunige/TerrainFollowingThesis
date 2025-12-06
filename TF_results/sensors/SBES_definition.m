function [sensors] = SBES_definition(wRr)
    %% Sensor Definition
    Gamma = -pi/8;                              % y1 angle (rear = SUD)
    Lambda = pi/8;                              % y2 angle (front = NORD)
    Eta = pi/8;                                 % y3 angle (left = EST)
    Zeta = -pi/8;                               % y4 angle (right = OVEST)
    r_s(:, 1) = [sin(Gamma), 0, cos(Gamma)]';   % y1 versor (rear)
    r_s(:, 2) = [sin(Lambda), 0, cos(Lambda)]'; % y2 versor (front)
    r_s(:, 3) = [0, -sin(Eta), cos(Eta)]';      % y3 versor (left)
    r_s(:, 4) = [0, -sin(Zeta), cos(Zeta)]';    % y4 versor (right)

    num_s = 4;

    % Sensor echosounder considering robot clean rotation (to be reliable)
    sensors = zeros(3,num_s);
    % check of consistency for the sensors
    for j = 1:num_s
        sensors(:,j) = wRr*r_s(:,j);
        if (norm(sensors(:,j)) ~= 1)
            printDebug('!! ALERT !!: norm Measured sensor %.0f has been normalized\n', j);
            sensors(:,j) = vector_normalization(sensors(:,j));
        end
    end
end