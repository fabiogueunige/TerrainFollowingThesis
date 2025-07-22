function command = goal_controller(command, x_ekf, gg, N, c_state, step)
    %% definition 
    IND_H = 1;      ALPHA = 2;      BETA = 3; 
    
    %% Epsilon definition
    alt_eps = 5;
    alt_risk = 1;

    %% Contact Check
    count = 0;
    if ~command.contact1 || ~command.contact2 || ~command.contact3 || ~command.contact4
        command.following = false;
        if ~command.contact1
            count = count + 1;
        end
        if ~command.contact2
            count = count + 1;
        end
        if ~command.contact3
            count = count + 1;
        end
        if ~command.contact4
            count = count + 1;
        end
        if count > 2 && command.setpoint
            command.emergency = true;
        else
            command.emergency = false;
        end
    end

    %% Altitude Check
    if abs(gg.altitude - x_ekf(IND_H)) < alt_eps
        command.setpoint = true;
        command.following = true;
    end

    if x_ekf(IND_H) < alt_risk
        command.setpoint = false;
        command.following = false;
        command.emergency = true;
    end

        
    %% End of the program
    if step == N
        command.end = true;
    end