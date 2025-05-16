close all
clear all
clc

%% Terrain Variables
beta = pi/10;
qt = - 10;
v_s = 0.1;

%% AUV Parameters
pr0 = [0,0]'; % AUV position
Gamma = -pi/6; % sonar angle y1
Lambda = pi/6; % sonar angle y2
h_ref = 7;
% velocities: v = [v_surge, v_sway, v_heave]'
des_vel = [0.1, 0, 0, 0]';

%% Extended Kalman filter parameters
Ts = 0.001; % sampling time
sig1 = 0.1; % state noise h
sig2 = 0.2; % state noise pitch
sig3 = 0.15; % state noise beta
eta1 = 0.05; % measurament noise y1
eta2 = 0.05; % measurament noise y2

% Supposed Initial condition for EKF 
init_cond = [2; 0; 0];
cov_ic = [1, 0, 0;
          0, 0.8, 0;
          0, 0, 0.7];
cov_ic = cov_ic * (1 + 2*rand);

%% noise matrices
% Noise on pitch = 0 because not controllable for now
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

%% AUV Model equations
run Model3D_equation_mat.m

%% Gain PID and PI
run Control_gains_mat.m