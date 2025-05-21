clc
clear all
% terrain construction
%% Measuraments Variables
roll = 0;
yaw = 0;
beta = -pi/10;
theta = pi/7;
delta = theta - beta; % !!! theta - beta
hv = 5;
Ts = 0.001;
i_point = [0,0]';

% rob frame
vu = 100;
vh = 300;

% Velocities in the robot frame
v = [vu, 0, vh]';

% Transformation robot referred to inertial
Rz = [cos(yaw), -sin(yaw), 0;
      sin(yaw), cos(yaw), 0;
      0, 0, 1];
Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
      -sin(theta), 0, cos(theta)];
Rx = [1 , 0, 0
      0, cos(roll), -sin(roll);
      0, sin(roll), cos(roll)];
o_R_r = Rz*Ry*Rx; 

delta_pos = o_R_r * v * Ts;

% Transformation seafloor referred to inertial (0 alto e s basso)
Rtemp = [1 0 0;
     0 -1 0;
     0 0 -1];
Ry = [cos(beta), 0, sin(beta);
      0, 1, 0;
      -sin(beta), 0, cos(beta)];
o_R_s = Ry * Rtemp;
% terrain frame
deltah_s = (o_R_s') * o_R_r * v * Ts;

% robot frame
deltah_r_x = (v(1)*cos(delta) + v(3)*sin(delta)) * Ts
deltah_s_x = deltah_s(1)

deltah_r_z = (-v(1)*sin(delta) + v(3)*cos(delta)) * Ts
deltah_s_z = deltah_s(3) 

point_in = i_point + [delta_pos(1), -delta_pos(3)]'



% inertia frame 
% % !! Adesso sbagliato perchè non torna la vera distanza totale
% deltah_i = o_R_r * Ry* v * Ts;
% deltah_i = norm(deltah_i) % simile. ma non perfetto perchè non considera 
% % segni, ma totale
