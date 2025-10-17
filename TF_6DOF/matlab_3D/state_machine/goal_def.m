function gg = goal_def(c_state, ang, x_ekf, step)
    %% References Definition
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global PHI; global THETA; global PSI;   

    %% Goal Values
    u_star = 0.3;  
    u_star_slow = 0.05;
    v_star = -0.2;  % -0.3
    v_star_slow = -0.1; %-0.05;
    global h_ref;
    phi_ref = x_ekf(ALPHA);  % Desired roll angle
    theta_ref = x_ekf(BETA);  % Desired pitch angle
    psi_ref = 0;  

    %% Predefinition of the goal
    ang_to_cut = pi/10;  % Angle to cut for roll and pitch

    g_altitude = h_ref(step);  % Desired altitude for everyone

    switch c_state
        case 'Reset'
            g_surge = 0;        % Safeness better no move on
            g_sway = 0;         % Safeness better no move on
            g_roll = 0;         % Angle to be 0
            g_pitch = 0;        % Angle to be 0
            g_yaw = 0;      % Desired yaw angle (not used for now)
        case 'TargetAltitude'
            g_surge = 0;            % Safeness better no move on
            g_sway = 0;             % Safeness better no move on
            g_roll = ang(PHI);      % Keep angle as before
            g_pitch = ang(THETA);   % Keep angle as before 
            g_yaw = ang(PSI);       % Desired yaw angle
        case 'ContactSearch'
            g_surge = (u_star + u_star_slow) / 2;
            g_sway = (v_star + v_star_slow) / 2;
            g_roll = phi_ref;    
            g_pitch = theta_ref;
            g_yaw = ang(PSI);  
        case 'MovePitch' 
            g_surge = u_star_slow;
            g_sway = v_star_slow;
            g_roll = (ang(PHI) + phi_ref) / 2;      % Keep angle as before
            if ang(THETA) >= pi/4
                g_pitch = theta_ref - ang_to_cut;
            else
                g_pitch = theta_ref + ang_to_cut;
            end
            g_yaw = ang(PSI);  
        case 'MoveRoll' %% TO IMPROVE
            g_surge = u_star_slow;
            g_sway = v_star;
            if ang(PHI) >= pi/4
                g_roll = phi_ref - ang_to_cut;
            else
                g_roll = phi_ref + ang_to_cut;
            end
            g_pitch = (ang(THETA) + theta_ref) / 2;    % Keep angle as before
            g_yaw = ang(PSI);  
        case 'Following'
            g_surge = u_star;
            g_sway = v_star;
            g_roll = phi_ref;    
            g_pitch = theta_ref; 
            g_yaw = psi_ref;  

        % case 'Acceleration' %% TO IMPROVE
        % case 'Deceleration' %% TO IMPROVE

        case 'Emergency'
            g_surge = 0;
            g_sway = 0;
            g_roll = ang(PHI);    
            g_pitch = ang(THETA); 
            g_yaw = ang(PSI);  % Desired yaw angle
        case 'EndSimulation'
            g_surge = 0;
            g_sway = 0;
            g_roll = 0;
            g_pitch = 0;
            g_yaw = 0;  % Desired yaw angle (not used for now)
        otherwise
            g_surge = 0;
            g_sway = 0;
            g_roll = 0;
            g_pitch = 0;
            g_yaw = 0;
    end

    %% Assegnazione
    gg.surge = g_surge;         % Desired surge speed
    gg.sway = g_sway;           % Desired sway speed
    gg.altitude = g_altitude;   % Desired altitude
    gg.roll = g_roll;           % Desired roll angle
    gg.pitch = g_pitch;         % Desired pitch angle
    gg.yaw = g_yaw;             % Desired yaw angle 
end