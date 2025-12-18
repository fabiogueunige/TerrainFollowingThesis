function [x_e, P_e, wRr_e, x_p, P_p] = ekf_position(x_old, tau, wRr, u_clean, eta_clean, P_old, Q_loc, Ts, P_p_old, x_p_old, update_DVL, update_AHRS, update_PS)
    
    %% INIT
    % dimensions
    dim_ekf = 15;
    dim_s = [6, 3, 1]; % DVL, AHRS, PS

    %% R definition
    R_loc = setupLoc_sensors();

    %% Predizione (sempre eseguita ogni ciclo)
    x_p = f_position(x_p_old, tau, wRr, Q_loc, dim_ekf, Ts);
    F_loc = jacobianF_position(x_p_old, wRr, dim_ekf, Ts);
    P_p = F_loc * P_p_old * F_loc' + Q_loc;

    %% Osservazione predetta
    y_p = h_position(x_p, dim_s);
    H_loc = jacobianH_position(dim_s, dim_ekf);

    %% Update condizionale - costruzione dinamica delle misure
    z_all = [];
    y_p_all = [];
    H_all = [];
    R_all = [];

    % Update DVL (10 Hz)
    if update_DVL
        z_DVL = read_DVL(u_clean(1:3), R_loc(1:3,1:3));
        z_all = [z_all; z_DVL];
        y_p_all = [y_p_all; y_p(1:3)];
        H_all = [H_all; H_loc(1:3,:)];
        if isempty(R_all)
            R_all = R_loc(1:3,1:3);
        else
            R_all = blkdiag(R_all, R_loc(1:3,1:3));
        end
    end

    % Update AHRS (100 Hz)
    if update_AHRS
        tp = [eta_clean(4:6); u_clean(4:6)];
        z_AHRS = read_AHRS(tp, R_loc(4:9,4:9));
        z_all = [z_all; z_AHRS];
        y_p_all = [y_p_all; y_p(4:9)];
        H_all = [H_all; H_loc(4:9,:)];
        if isempty(R_all)
            R_all = R_loc(4:9,4:9);
        else
            R_all = blkdiag(R_all, R_loc(4:9,4:9));
        end
    end

    % Update PS (20 Hz)
    if update_PS
        z_PS = read_PS(eta_clean(3), R_loc(10,10));
        z_all = [z_all; z_PS];
        y_p_all = [y_p_all; y_p(10)];
        H_all = [H_all; H_loc(10,:)];
        if isempty(R_all)
            R_all = R_loc(10,10);
        else
            R_all = blkdiag(R_all, R_loc(10,10));
        end
    end

    %% Esegui update solo se ci sono misure disponibili
    if ~isempty(z_all)
        % Innovazione
        ni = z_all - y_p_all;
        
        % Wrap degli angoli se AHRS è presente
        if update_AHRS
            if update_DVL
                idx_start = 4; % DVL ha 3 elementi, AHRS inizia dal 4
            else
                idx_start = 1; % AHRS è il primo
            end
            for j = idx_start:idx_start+2
                ni(j) = wrapToPi(ni(j));
            end
        end
        
        % Kalman Gain
        S = H_all * P_p * H_all' + R_all;
        K = P_p * H_all' / S;
        
        % State e Covariance Update
        x_e = x_p + K * ni;
        P_e = (eye(dim_ekf) - K * H_all) * P_p;
        
        % Wrap degli angoli nello stato stimato
        for j = 4:6
            x_e(j) = wrapToPi(x_e(j));
        end

        P_p = P_e; % Aggiorna la covarianza predetta per il prossimo ciclo
        x_p = x_e; % Aggiorna lo stato predetto per il prossimo ciclo
    else
        % Nessun update, mantieni lo stato stimato precedente
        x_e = x_old;
        P_e = P_old;
    end
    
    % Calcola matrice di rotazione solo se AHRS ha fatto update
    if update_AHRS
        wRr_e = rotz(x_e(6)) * roty(x_e(5)) * rotx(x_e(4));
    else
        wRr_e = wRr;  % Mantieni la matrice di rotazione precedente
    end
end

