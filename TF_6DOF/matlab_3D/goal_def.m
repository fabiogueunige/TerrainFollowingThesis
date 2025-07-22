function gg = goal_def(c_state, x_ekf, step)
    %% References Definition
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    u_star = 0.3;  % Desired surge speed
    v_star = 0.0;  % Desired sway speed
    global h_ref;
    phi_ref = x_ekf(ALPHA);  % Desired roll angle
    theta_ref = x_ekf(BETA);  % Desired pitch angle
    % yaw_ref = 0;  % Desired yaw angle (not used for now)

    %% Predefinition of the goal
    ang_to_cut = -pi/10;  % Angle to cut for roll and pitch

    g_altitude = h_ref(step);  % Desired altitude

    switch c_state
        case 'TargetAltitude'
            g_surge = u_star;
            g_sway = 0;
            g_roll = 0;    % Desired roll angle
            g_pitch = 0;   % Desired pitch angle
            % g_yaw = 0;  % Desired yaw angle (not used for now)
        case 'ContactSearch'
            g_surge = 0;
            g_sway = 0;
            g_roll = phi_ref;    % Desired roll angle
            g_pitch = theta_ref; % Desired pitch angle
            % g_yaw = 0;  % Desired yaw angle (not used for now)
        case 'MovePitch' %% TO IMPROVE
            g_surge = u_star;
            g_sway = v_star;
            g_roll = phi_ref;    % Desired roll angle
            g_pitch = theta_ref + ang_to_cut; % Desired pitch angle
            % g_yaw = 0;  % Desired yaw angle (not used for now)
        case 'MoveRoll' %% TO IMPROVE
            g_surge = u_star;
            g_sway = v_star;
            g_roll = phi_ref + ang_to_cut;    % Desired roll angle
            g_pitch = theta_ref; % Desired pitch angle
            % g_yaw = 0;  % Desired yaw angle (not used for now)
        case 'Following'
            g_surge = u_star;
            g_sway = v_star;
            g_roll = phi_ref;    % Desired roll angle
            g_pitch = theta_ref; % Desired pitch angle
            % g_yaw = 0;  % Desired yaw angle (not used for now)
        % case 'Acceleration' %% TO IMPROVE
        % case 'Deceleration' %% TO IMPROVE
        case 'Emergency'
            g_surge = 0;
            g_sway = 0;
            g_roll = phi_ref;    % Desired roll angle
            g_pitch = theta_ref; % Desired pitch angle
            % g_yaw = 0;  % Desired yaw angle (not used for now)
        case 'EndSimulation'
            g_surge = 0;
            g_sway = 0;
            g_roll = 0;
            g_pitch = 0;
            % g_yaw = 0;  % Desired yaw angle (not used for now)
        otherwise
            g_surge = 0;
            g_sway = 0;
            g_roll = 0;
            g_pitch = 0;
            % g_yaw = 0;
    end

    %% Assegnazione
    gg.surge = g_surge;  % Desired surge speed
    gg.sway = g_sway;    % Desired sway speed
    gg.altitude = g_altitude;  % Desired altitude
    gg.roll = g_roll;    % Desired roll angle
    gg.pitch = g_pitch;  % Desired pitch angle
    % gg.yaw = g_yaw;    % Desired yaw angle (not used for now)
end