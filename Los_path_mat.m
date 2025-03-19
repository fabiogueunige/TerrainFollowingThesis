close all
clear all
clc

% Equilibrium point and system specification
m = 30;
I = 4.14;
p = 10; % system pole
wn = 0.2;
damp = 0.6;

% x initial
speed0 = [0.1, 0, 0, 0]';
depth_des = 0.5;
eta_init = [0.1, 0, 0, 0.7]';
current = [0, 0, 0, 0]';
des_vel = [0.1, 0, 0, 0]';
deltah = 4;

% Added mass
tau_a = [-25; -2.325; -8.690; -19.4311]; % kl_dot

% Restoring forces
tau_r = [-0.2; -55.117; -4.14; 33.6804]; % kl

% Hydrodynamic forces
tau_d = [-19.5; -147.9; -6.23; -89.4105]; % kl_modl (era -89...

% Virtual mass
mv = [m; m; I; m] - tau_a;

% Dissatipative forces
dv = -tau_r - 2*tau_d .* abs(speed0);

%% Gain Computation
% Surge
Kd_surge = 0;
Kp_surge = 2*damp*wn*mv(1) - dv(1); % 17.3
Ki_surge = wn^2 * mv(1); % 55.4

% YAW
Kd_yaw = (p + 2*damp*wn)*mv(3) - dv(3); % 49.34
Kp_yaw = mv(3)*((wn^2) + 2*damp*p*wn); % 
Ki_yaw = p*(wn^2)*mv(3);

% DEPTH (pid)
Kd_dep = (p + 2*damp*wn)*mv(4) + dv(4);
Kp_dep = mv(4)*((wn^2) + 2*damp*p*wn);
Ki_dep = p*(wn^2)*mv(4);