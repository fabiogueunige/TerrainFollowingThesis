clc
clear all
% terrain construction
%% Measuraments Variables
roll = 0;
yaw = 0;
beta = pi/6;
theta = pi/10;
hv = 5;
Ts = 0.001;

% Velocities in the robot frame
v = [0, 0, -150]';

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

% Transformation seafloor referred to inertial (0 alto e s basso)
Rtemp = [1 0 0;
     0 -1 0;
     0 0 -1];
Ry = [cos(beta), 0, sin(beta);
      0, 1, 0;
      -sin(beta), 0, cos(beta)];
o_R_s = Ry * Rtemp; % per me se no andrebbe cambiato il beta

% robot frame
deltah_r = v(1)*sin(beta - theta)*Ts + v(3)*cos(beta - theta)*Ts

% terrain frame
deltah_s = (o_R_s') * o_R_r * v * Ts;
deltah_s(3)

% inertia frame 
% !! Adesso sbagliato perchè non torna la vera distanza totale
deltah_i = o_R_r * Ry* v * Ts;
deltah_i = norm(deltah_i) % simile. ma non perfetto perchè non considera 
% segni, ma totale
