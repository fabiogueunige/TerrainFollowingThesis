function [x, x_dot] = kinematic_model(v_tmp, x_old, x_dot_old, dim_i, Ts)
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;

    %%%%%%% NO FULL ACTUATION %%%%%%%%%%%%%%
    x_dot = zeros(6,1); %dim_i 
    x = zeros(6,1); %dim_i 
    v = zeros(6,1);
    v(1:dim_i) = v_tmp;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% LINEARIZED
    x_dot(ROLL) = v(ROLL);
    x_dot(PITCH) = v(PITCH);
    x_dot(YAW) = v(YAW);

    %% NEW NOT LINEARIZED
    % angular cartesian velocities
    % x_dot(ROLL) = v(ROLL) + v(PITCH)*sin(x_old(ROLL))*tan(x_old(PITCH)) + v(YAW)*cos(x_old(ROLL))*tan(x_old(PITCH));
    % x_dot(PITCH) = v(PITCH)*cos(x_old(ROLL)) - v(YAW)*sin(ROLL);
    % x_dot(YAW) = 0; % v(PITCH)*(sin(x_old(ROLL))/cos(x_old(PITCH))) + v(YAW)*(cos(x_old(ROLL))/cos(x_old(PITCH)));

    %% Position Values
    x(ROLL:YAW) = integrator(x_old(ROLL:YAW), x_dot(ROLL:YAW), x_dot_old(ROLL:YAW), Ts);
    % rotation
    wRr_kinematics = rotz(x(YAW)) * roty(x(PITCH)) * rotx(x(ROLL));
    % linear cartesian velocities
    x_dot(SURGE:HEAVE) = wRr_kinematics * (v(SURGE:HEAVE));

    x(SURGE:HEAVE) = integrator(x_old(SURGE:HEAVE), x_dot(SURGE:HEAVE), x_dot_old(SURGE:HEAVE), Ts);  
end

% linear cartesian velocities (from trasformation)
% x_dot(SURGE) = v(SURGE)*cos(x_old(YAW))*cos(x_old(PITCH)) + v(SWAY)*(-sin(x_old(YAW))*cos(x_old(ROLL)) + cos(x_old(YAW))*sin(x_old(ROLL))*sin(x_old(PITCH))) ...
%      + v(HEAVE)*(sin(x_old(ROLL))*cos(x_old(YAW)) + cos(x_old(ROLL))*cos(x_old(YAW))*sin(x_old(PITCH)));
% x_dot(SWAY) = v(SURGE)*sin(x_old(YAW))*cos(x_old(PITCH)) + v(SWAY)*(cos(x_old(YAW))*cos(x_old(ROLL)) + sin(x_old(YAW))*sin(x_old(ROLL))*sin(x_old(PITCH))) ...
%      + v(HEAVE)*(sin(x_old(YAW))*sin(x_old(THETA))*cos(x_old(ROLL)) - cos(x_old(YAW))*sin(x_old(YAW)));
% x_dot(HEAVE) = -v(SURGE)*sin(x_old(PITCH)) + v(SWAY)*(cos(x_old(PITCH))*sin(x_old(ROLL))) + v(HEAVE)*cos(x_old(ROLL))*cos(x_old(PITCH));