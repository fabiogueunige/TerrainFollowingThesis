close all
clear all
clc

%% Measuraments Variables
pr0 = [0,0]';
beta = pi/10;
qt = - 10;
Gamma = -pi/6;
Lambda = pi/6;

%% Extended Kalman filter
% review of every parameter
Ts = 0.001;
sig1 = 0.1;
sig2 = 0.2;
sig3 = 0.15;
eta1 = 0.05;
eta2 = 0.05;

% velocities: v = [v_surge, v_sway, v_heave]'
v_surge = 0.5; % constant (for now value)
% v_h = should be given by a pid
v_heave = 0.00; % vertical speed robot

% !!! to change the values
init_cond = [2; 0; 0];
cov_ic = [1, 0, 0;
          0, 0.8, 0;
          0, 0, 0.7];
cov_ic = cov_ic * (1 + 2*rand);

% A, B, F, H on simulink
%% noise 
% Noise on pitch = 0 because not controllable
% R measurament noise
R = [(eta1^2), 0;
     0, (eta2^2)];

% Q state noise matrix
Q = zeros(3,3);
Q(1,1) = (sig1^2);
% Q(2,2) = (sig2^2);
Q(3,3) = (sig3^2);

% G coefficient noise matrix
G = eye(3) * Ts;
