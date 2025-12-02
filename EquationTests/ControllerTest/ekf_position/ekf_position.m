function [x_e, wRr_e, P_e] = ekf_position(eta_old, u_clean, wRr_old, P_e_old, Q_loc, R_loc, Ts, choice)
    SURGE = 1;
    SWAY = 2;
    HEAVE = 3;
    ROLL = 4;
    PITCH = 5;
    YAW = 6;
    
    %% INIT
    dim_ekf = 6;
    dim_s = [3, 3, 1]; % DVL, AHRS, PS
    x_p = zeros(dim_ekf,1);
    x_e = zeros(dim_ekf,1);
    P_e = zeros(dim_ekf,dim_ekf);

    s_phi = sin(eta_old(ROLL));
    c_phi = cos(eta_old(ROLL));
    c_theta = cos(eta_old(PITCH));
    t_theta = tan(eta_old(PITCH)); 
    T = [1,   s_phi * t_theta,   c_phi * t_theta;
        0,        c_phi,              -s_phi;
        0,   s_phi / c_theta,   c_phi / c_theta];

    %% Clean Localization prediction
    x_p(1:3) = eta_old(1:3) + wRr_old * u_clean(1:3) * Ts; 
    
    % Safety check for gimbal lock
    if abs(c_theta) < 1e-6
        warning('Gimbal lock detected: theta near ±90°');
        T = eye(3);  % Use identity to avoid division by zero
    end
    x_p(4:6) = eta_old(4:6) + T * u_clean(4:6) * Ts;

    if (choice == -1) % linear dynamics
        x_e(1:3) = eta_old(1:3) + wRr_old * u_clean(1:3) * Ts; 
        % For linear case, integrate Euler angles properly
        x_e(4:6) = eta_old(4:6) + T*u_clean(4:6) * Ts;
    elseif choice == 1 % non linear with noise
        F_loc = eye(dim_ekf);
        H_loc = [1, 0, 0, 0, 0, 0;
            0, 1, 0, 0, 0, 0;
            0, 0, 1, 0, 0, 0;
            0, 0, 0, 1, 0, 0;
            0, 0, 0, 0, 1, 0;
            0, 0, 0, 0, 0, 1;
            0, 0, 1, 0, 0, 0]; 

        y = zeros(dim_ekf+1,1);

        z.DVL = zeros(dim_s(1),1);
        z.AHRS = zeros(dim_s(2),1);
        z.PS = zeros(dim_s(3),1);

        %% F computattion
        % Build rotation R(eta2) = Rz(yaw)*Ry(pitch)*Rx(roll)
        psi = eta_old(YAW);
        theta = eta_old(PITCH);
        phi = eta_old(ROLL);

        Rz = rotz(psi);
        Ry = roty(theta);
        Rx = rotx(phi);

        % Derivatives of rotation wrt Euler angles (columns of upper-right Jacobian)
        dR_dphi = Rz * Ry * d_rotx(phi);
        dR_dtheta = Rz * d_roty(theta) * Rx;
        dR_dpsi = d_rotz(psi) * Ry * Rx;

        % Upper-right block: [dR/dphi*nu1, dR/dtheta*nu1, dR/dpsi*nu1] * Ts
        J12 = [dR_dphi * u_clean(1:3), dR_dtheta * u_clean(1:3), dR_dpsi * u_clean(1:3)] * Ts;

        % Lower-right incremental term: (dT/dphi*p + dT/dtheta*q) * Ts
        JT = dT_dphi(phi, theta) * u_clean(4) + dT_dtheta(phi, theta) * u_clean(5);

        % Assemble F_loc per block structure
        F_loc(1:3,4:6) = J12;
        F_loc(4:6,4:6) = eye(3) + JT * Ts;

        %% P computation
        P_p = F_loc * P_e_old * F_loc' + Q_loc;

        %% Measuraments acquisition
        z = zeros(sum(dim_s),1);
        mes_vel = read_DVL(u_clean(1:3), R_loc(1:3,1:3));
        
        z(4:6) = read_AHRS(x_p(4:6), R_loc(4:6,4:6));
        rotme = rotz(z(6)) * roty(z(5)) * rotx(z(4));
        z(1:3) = rotme' * mes_vel; % Body to NED
        z(7) = read_PS(x_p(3), R_loc(7,7));

        %% Localization prediction
        w_loc = mvnrnd(zeros(dim_ekf,1), Q_loc)';
        x_p = x_p + w_loc;

        %% Covariance Innovation & Gain
        S_loc = H_loc * P_p * H_loc' + R_loc;
        K_loc = P_p * H_loc' / S_loc;

        %% State Update
        y = [x_p; x_p(3)];

        %% State Innovation
        ni_loc = z - y;
        x_e = x_p + K_loc * ni_loc;
        P_e = (eye(dim_ekf) - K_loc * H_loc) * P_p;
    else
        % choice == 0 or other: nonlinear dynamics without sensor noise
        x_e(1:3) = eta_old(1:3) + wRr_old * u_clean(1:3) * Ts;
        x_e(4:6) = eta_old(4:6) + T * u_clean(4:6) * Ts;
        P_e = P_e_old;
    end
    wRr_e = rotz(x_e(6)) * roty(x_e(5)) * rotx(x_e(4));
end

