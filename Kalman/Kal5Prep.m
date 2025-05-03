close all
clear all
clc

%% With  Extended Kalman filter
Ts = 0.001;
sig1 = 0.1;
sig2 = 0.2;
eta1 = 0.15;
pb = [3; 3];

init_cond = [0; 0; 1; 1];
cov_ic = [2, 0, 0, 0;
          0, 2, 0, 0;
          0, 0, 3, 0;
          0, 0, 0, 3];
cov_ic = cov_ic * (0.5 + 2*rand);
v0 = [3; 0];
vc = [1.0; 1.0];

% A matrix
A = [1, 0, Ts, 0;
     0, 1, 0, Ts;
     0, 0, 1, 0;
     0, 0, 0, 1];

% B matrix
B = [Ts, 0;
     0, Ts;
     0, 0;
     0, 0];

% F matrix
% Computation on simulink
% H matrix
% Computation on simulink

% R measurament noise
R = (eta1)^2;

% Q state noise matrix
Q = eye(4);
Q(1:2,1:2) = Q(1:2,1:2) * (sig1^2);
Q(3:4,3:4) = Q(3:4,3:4) * (sig2^2);

% G coefficient noise matrix
G = eye(4) * Ts;
