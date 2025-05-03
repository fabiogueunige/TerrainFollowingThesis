close all
clear all
clc

%% Extended Kalman filter
% review of every parameter
Ts = 0.001;
sig1 = 0.1;
sig2 = 0.2;
sig3 = 0.15;
eta1 = 0.15;
eta2 = 0.2;

% velocities
v_surge = 2; % constant (for now value)
% v_h = should be given by a pid
v_ver = 0.01; % (TO Replace)

% !!! to change the values
init_cond = [0; 0; 0];
cov_ic = [1, 0, 0;
          0, 0.8, 0;
          0, 0, 0.7];
cov_ic = cov_ic * (0.5 + 2*rand);

% A, B, F, H on simulink
%% noise (ok)
% R measurament noise
R = [eta1, 0;
     0, eta2];

% Q state noise matrix
Q = zeros(3,3);
Q(1,1) = (sig1^2);
Q(2,2) = (sig2^2);
Q(3,3) = (sig3^2);

% G coefficient noise matrix
G = eye(3) * Ts;
