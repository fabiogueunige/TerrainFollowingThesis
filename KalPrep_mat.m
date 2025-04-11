close all
clear all
clc

K1 = 0.48;
K2 = 0.16;
bayas = 0.5;

%% With Kalman filter
Ts = 0.001;
sig1 = 0.1;
sig2 = 0.2;
eta1 = deg2rad(1);

init_cond = [pi/2; 5];
cov_ic = [10, 0;
        0, 10];

A = [1, Ts;
    0, 1];
B = [Ts; 0];
C = [1, 0];
R = (eta1)^2;
Q = [(sig1^2), 0;
    0, (sig2^2)];
G = [Ts 0;
    0 Ts];

%% Riccati Kalman Filter
Kf = P * C' / (C * P * C' + R);


% x(:,k+1) = A * x(:,k) + B * u(k) + w;
%     y(k) = C * x(:,k) + v;
% 
%     %Predizione
%     x_pred = A * x_hat(:,k) + B * u(k);
% 
%     %Correzione (abilita/disabilita per solo predizione)
%     x_hat(:,k+1) = x_pred + K * (y(k) - C * x_pred);
