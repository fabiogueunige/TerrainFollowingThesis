function [x, x_dot] = kinematic_model(v, x_old, x_dot_old, dim_i, Ts)
    x_dot = zeros(6,1); %dim_i 
    x = zeros(6,1); %dim_i 

    x_dot(ROLL) = v(ROLL); % v(ROLL); + v(PITCH)*sin(x_old(ROLL))*tan(x_old(PITCH)) + v(YAW)*cos(x_old(ROLL))*tan(x_old(PITCH));
    x_dot(PITCH) = v(PITCH); %v(PITCH)*cos(x_old(ROLL)) - v(YAW)*sin(ROLL);
    x_dot(YAW) = v(YAW); % v(PITCH)*(sin(x_old(ROLL))/cos(x_old(PITCH))) + v(YAW)*(cos(x_old(ROLL))/cos(x_old(PITCH)));

    % Quelle di posizione ricavo tranquillamente con matrice di rotazione

    x = integrator(x_old, x_dot, x_dot_old, Ts);
    
end