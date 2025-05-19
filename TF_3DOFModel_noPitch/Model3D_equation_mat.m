% Equilibrium point and system specification
m = 30;
I = 4.14;
p = 10; % system pole
wn = 0.2;
damp = 0.6;

% Controllare che rispetti il punto di equilibrio e calcolare quello nuovo
speed0 = [0.1, 0, 0, 0]';
sat_surge = 15; sat_dep = 15; sat_yaw = 15;
sat_tr = 2000;

% [surge, sway, heave, roll, pitch, yaw]
% Added mass
tau_a = [-25; -2.325; -19.4311; -8.690]; % kl_dot

% Restoring forces
tau_r = [-0.2; -55.117; 33.6804; -4.14]; % kl

% Hydrodynamic forces
tau_d = [-19.5; -147.9; -89.4105; -6.23]; % kl_modl

% Virtual mass
mv = [m; m; m; I] - tau_a;

% Dissatipative forces
dv = -tau_r - 2*tau_d .* abs(speed0);