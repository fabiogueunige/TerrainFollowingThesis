function [Q, P0] = stateLoc_init(dim_f)
    % Initial state covariance
    P0 = eye(dim_f) * 3;  % Small initial uncertainty
    P0(12:15, 12:15) = eye(4) * 0.1; % Partono da 0
    
    q_pos   = 0.1;   % Incertezza piccola sull'integrazione posizione (x,y,z)
    q_att   = 0.01;  % Incertezza piccola sull'integrazione angoli (phi, theta, psi)
    q_vel   = 2.0;   % ALTA: Il modello di velocità lineare è inaffidabile (correnti)
    q_rates = 1.0;   % ALTA: Il modello di velocità angolare è inaffidabile
    q_gyro = 0.01;  % Bassa: i giroscopi sono abbastanza lenti a variare

    % Process noise covariance Q - very low for close tracking
   Q = diag([...
    q_pos, q_pos, q_pos, ...      
    q_att, q_att, q_att, ...       
    q_vel, q_vel, q_vel, ...       
    q_rates, q_rates, q_rates, ... 
    q_gyro, q_gyro, q_gyro ...  
]);
end