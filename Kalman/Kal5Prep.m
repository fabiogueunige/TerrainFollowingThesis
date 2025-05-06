close all
clear all
clc

%% With  Extended Kalman filter
Ts = 0.001;
sig1 = 0.005;
sig2 = 0.001;
eta1 = 0.002;
pb = [0.5; 0.5];

init_cond = [0; 0; 0.02; 0.01];
P0 = diag([1, 1.1, 0.8, 0.9]);
cov_ic = P0 * (1 + 2*rand);
v0 = [1; 0];

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
