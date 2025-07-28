function [Q, R, R_ang] = noise_setup (q_dim, r_dim, a_dim)
    % State Noise
    sigh = 0.099;           % State noise h
    siga = deg2rad(0.55);      % State noise alpha
    sigb = deg2rad(0.5);    % State noise beta

    % Measurament noise (added a 0 to all)
    eta1 = 0.177;           % Measurement noise y1
    eta2 = 0.185;           % Measurement noise y2
    eta3 = 0.177;           % Measurement noise y3
    eta4 = 0.185;           % Measurement noise y4

    % Initialization
    Q = zeros(q_dim,q_dim); % Process noise covariance
    Q(1,1) = (sigh^2);
    Q(2,2) = (siga^2);
    Q(3,3) = (sigb^2);
    
    % Measurement noise covariance (r_dim x r_dim)
    R = zeros(r_dim,r_dim);
    R(1,1) = (eta1^2);
    R(2,2) = (eta2^2);
    R(3,3) = (eta3^2);
    R(4,4) = (eta4^2);

    % Measurament angle noise covariance (a_dim x a_dim)
    R_ang = zeros(a_dim,a_dim);
    R_ang(1,1) = (deg2rad(0.5)^2);
    R_ang(2,2) = (deg2rad(0.5)^2);
    R_ang(3,3) = (deg2rad(0.08)^2);
end
