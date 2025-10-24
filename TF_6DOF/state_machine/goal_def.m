function gg = goal_def(c_state, ang, x_ekf, step)
    %% References Definition
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global PHI; global THETA; global PSI;   

    %% Goal Values (u, v, w are in terrain frame)
    u_star = 0.3;  
    u_star_slow = 0.05;
    u_star_recovery = 0.02; % Even slower for recovery maneuvers
    v_star = -0.2;  
    v_star_slow = -0.1;
    v_star_recovery = -0.05;
    global h_ref;
    phi_ref = x_ekf(ALPHA);  % Desired roll angle from terrain estimate
    theta_ref = x_ekf(BETA);  % Desired pitch angle from terrain estimate
    psi_ref = 0;  

    %% Recovery parameters
    ang_to_cut_initial = pi/12;  % Smaller initial adjustment (15 deg)
    ang_to_cut_progressive = pi/8; % Progressive increase (22.5 deg)
    altitude_offset = 1.0; % Altitude adjustment for recovery (meters)

    g_altitude = h_ref(step);  % Desired altitude for most states

    switch c_state
        case 'Reset'
            g_surge = 0;
            g_sway = 0;
            g_roll = 0;
            g_pitch = 0;
            g_yaw = 0;
            
        case 'TargetAltitude'
            g_surge = 0;
            g_sway = 0;
            g_roll = ang(PHI);
            g_pitch = ang(THETA);
            g_yaw = ang(PSI);
            
        case 'ContactSearch'
            g_surge = (u_star + u_star_slow) / 2;
            g_sway = (v_star + v_star_slow) / 2;
            g_roll = phi_ref;    
            g_pitch = theta_ref;
            g_yaw = ang(PSI);
            
        case 'MovePitch'
            % Slow movement with progressive pitch adjustment
            g_surge = u_star_recovery;
            g_sway = v_star_recovery;
            g_roll = phi_ref; % Maintain roll aligned with terrain
            
            % Gradual pitch adjustment based on current angle
            pitch_error = abs(ang(THETA) - theta_ref);
            if pitch_error > pi/6
                % Large error: use progressive adjustment
                ang_adjustment = ang_to_cut_progressive;
            else
                % Small error: use gentle adjustment
                ang_adjustment = ang_to_cut_initial;
            end
            
            % Determine direction based on which sensors are lost
            % If sensors 1-2 are lost, they're in front, so increase pitch to dive slightly
            if ang(THETA) >= 0
                g_pitch = theta_ref + ang_adjustment;
            else
                g_pitch = theta_ref - ang_adjustment;
            end
            g_yaw = ang(PSI);
            
        case 'MoveRoll'
            % Slow movement with progressive roll adjustment
            g_surge = u_star_recovery;
            g_sway = v_star_slow; % Maintain some lateral movement
            g_pitch = theta_ref; % Maintain pitch aligned with terrain
            
            % Gradual roll adjustment based on current angle
            roll_error = abs(ang(PHI) - phi_ref);
            if roll_error > pi/6
                % Large error: use progressive adjustment
                ang_adjustment = ang_to_cut_progressive;
            else
                % Small error: use gentle adjustment
                ang_adjustment = ang_to_cut_initial;
            end
            
            % Determine direction based on which sensors are lost
            % If sensors 3-4 are lost, adjust roll to bring them back to terrain
            if ang(PHI) >= 0
                g_roll = phi_ref + ang_adjustment;
            else
                g_roll = phi_ref - ang_adjustment;
            end
            g_yaw = ang(PSI);
            
        case 'RecoveryAltitude'
            % Adjust altitude to try to recover sensor contact
            % Slow movement, maintain orientation, adjust altitude
            g_surge = u_star_recovery;
            g_sway = v_star_recovery;
            g_roll = phi_ref;
            g_pitch = theta_ref;
            g_yaw = ang(PSI);
            % Increase target altitude to move closer to terrain
            g_altitude = h_ref(step) - altitude_offset;
            
        case 'Following'
            g_surge = u_star;
            g_sway = v_star;
            g_roll = phi_ref;    
            g_pitch = theta_ref; 
            g_yaw = psi_ref;

        case 'Emergency'
            g_surge = 0;
            g_sway = 0;
            g_roll = ang(PHI);    
            g_pitch = ang(THETA); 
            g_yaw = ang(PSI);
            
        case 'EndSimulation'
            g_surge = 0;
            g_sway = 0;
            g_roll = 0;
            g_pitch = 0;
            g_yaw = 0;
            
        otherwise
            g_surge = 0;
            g_sway = 0;
            g_roll = 0;
            g_pitch = 0;
            g_yaw = 0;
    end

    %% Assignment
    gg.surge = g_surge;
    gg.sway = g_sway;
    gg.altitude = g_altitude;
    gg.roll = g_roll;
    gg.pitch = g_pitch;
    gg.yaw = g_yaw;
end