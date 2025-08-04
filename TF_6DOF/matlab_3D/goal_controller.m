function command = goal_controller(command, x_ekf, ang, gg, N, c_state, step, dim_a)
    %% definition 
    IND_H = 1;      ALPHA = 2;      BETA = 3; 
    
    %% Epsilon definition
    alt_eps = 5;
    alt_risk = 0.7;
    
    %% Altitude state
    if c_state == "TargetAltitude"
        if abs(gg.altitude - x_ekf(IND_H)) < alt_eps
            command.setpoint = true;
            command.emergency = false;
        end
    end

    %% Reset
    if c_state == "Reset"
        command.reset = true;
        for j = 1:dim_a
            if abs(ang(j) - 0) > deg2rad(10) 
                command.reset = false;
                break;
            end
        end
    end

    %% Emergency State
    if c_state == "Emergency"
        command.setpoint = false;
        command.emergency = false;
    end

    %% Safety Check Altitude ALWAYS
    if x_ekf(IND_H) < alt_risk
        command.setpoint = false;
        command.emergency = true;
    end
 
    %% End of the program
    if step == N
        command.end = true;
    end