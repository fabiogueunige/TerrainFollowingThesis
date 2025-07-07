function [Q, G, R] = noise_setup (q_dim, r_dim, sampling_time)
    % State Noise
    sigh = 0.999;           % State noise h
    siga = deg2rad(0.8);    % State noise alpha
    sigb = deg2rad(0.2);      % State noise beta
    sigph = deg2rad(0.5);   % State noise phi
    sigt = deg2rad(0.2);    % State noise theta
    % sigp = 0.097;         % State noise p
    % sigq = 0.095;         % State noise q
    % Measurament noise (added a 0 to all)
    eta1 = 0.0077;           % Measurement noise y1
    eta2 = 0.0085;           % Measurement noise y2
    eta3 = 0.0077;           % Measurement noise y3
    eta4 = 0.0085;           % Measurement noise y4
    eta5 = deg2rad(0.3);    % Measurement noise accelerometer p
    eta6 = deg2rad(0.1);      % Measurement noise accelerometer q
    % eta7 = 0.077;         % Measurament noise gyroscope p
    % eta8 = 0.077;         % Measurament noise gyroscope q

    % Initialization
    Q = zeros(q_dim,q_dim); % Process noise covariance
    Q(1,1) = (sigh^2);
    Q(2,2) = (siga^2);
    Q(3,3) = (sigb^2);
    Q(4,4) = (sigph^2);
    Q(5,5) = (sigt^2);
    % Q(6,6) = (sigp^2);
    % Q(7,7) = (sigq^2);
    
    % G coefficient noise matrix
    G = eye(q_dim) * sampling_time;
    
    % Measurement noise covariance (r_dim x r_dim)
    R = zeros(r_dim,r_dim);
    R(1,1) = (eta1^2);
    R(2,2) = (eta2^2);
    R(3,3) = (eta3^2);
    R(4,4) = (eta4^2);
    R(5,5) = (eta5^2);
    R(6,6) = (eta6^2);
    % R(7,7) = (eta7^2);
    % R(8,8) = (eta8^2);
end
