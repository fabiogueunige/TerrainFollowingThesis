function [x_e, P_e, wRr_e] = ekf_position(x_old, tau, wRr, u_clean, eta_clean, P_e_old, Q_loc, Ts)
    
    %% INIT
    % dimensions
    dim_ekf = 15;
    dim_s = [6, 3, 1]; % DVL, AHRS, PS

    %% R definition
    R_loc = setupLoc_sensors();

    %% State f
    x_p = f_position(x_old, tau, wRr, Q_loc, dim_ekf, Ts);
    F_loc = jacobianF_position(x_old, wRr, dim_ekf, Ts);

    %% P computation
    P_p = F_loc * P_e_old * F_loc' + Q_loc;

    %% Observation
    y_p = h_position(x_p, dim_s);
    H_loc = jacobianH_position(dim_s, dim_ekf);

    %% Measuraments acquisition
    z = zeros(sum(dim_s),1);
    z(1:3) = read_DVL(u_clean(1:3), R_loc(1:3,1:3));
    tp = [eta_clean(4:6); u_clean(4:6)];
    z(4:9) = read_AHRS(tp, R_loc(4:9,4:9));
    z(10) = read_PS(eta_clean(3), R_loc(10,10));

    %% Covariance Innovation & Gain
    S_loc = H_loc * P_p * H_loc' + R_loc;
    K_loc = P_p * H_loc' / S_loc;

    %% State Innovation
    ni_loc = z - y_p;
    % Wrap to avoid angle going out of range
    for j = 4:6
        ni_loc(j) = wrapToPi(ni_loc(j));
    end
    x_e = x_p + K_loc * ni_loc;
    for j = 4:6
        x_e(j) = wrapToPi(x_e(j));
    end
    P_e = (eye(dim_ekf) - K_loc * H_loc) * P_p;
    
    wRr_e = rotz(x_e(6)) * roty(x_e(5)) * rotx(x_e(4));
end

