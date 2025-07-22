function command = goal_controller(command, x_ekf, gg, N, c_state, step)
    %% definition 
    IND_H = 1;      ALPHA = 2;      BETA = 3; 
    
    %% Epsilon definition
    alt_eps = 5;
    alt_risk = 1;
    
    %% Altitude state
    if c_state == "TargetAltitude"
        if abs(gg.altitude - x_ekf(IND_H)) < alt_eps
            command.setpoint = true;
            command.following = true;
        end
    end

    %% Contact Search
    if c_state == "ContactSearch" || c_state == "MoveRoll" || c_state == "MovePitch"
        if command.sensor_fail <= 2
            command.following = true;
            command.emergency = false;
        else
            if command.sensor_fail > 3
                command.emergency = true;
            end
            command.following = false;
        end
    end

    %% Following state -> Contact Check
    if c_state == "Following"
        if command.sensor_fail > 1
            if ((~cmd.contact1 && ~cmd.contact2) || (~cmd.contact3 && ~cmd.contact4))
                command.following = false;
            else
                command.following = true;
            end
        else
            command.following = true;
        end
        if command.sensor_fail >= 3
            command.emergency = true;
        end
    end

    %% Emergency
    if c_state == "Emergency"
        command.following = false;
        command.emergency = false;
    end

    %% Safety Check Altitude ALWAYS
    if x_ekf(IND_H) < alt_risk
        command.setpoint = false;
        command.following = false;
        command.emergency = true;
    end
 
    %% End of the program
    if step == N
        command.end = true;
    end