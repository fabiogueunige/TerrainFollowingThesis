function command = goal_controller(command, x_ekf, ang, gg, N, c_state, step, dim_a)
    %% Definition 
    IND_H = 1;      ALPHA = 2;      BETA = 3; 
    
    %% Epsilon definition
    alt_eps = 5;
    alt_risk = 0.7;
    
    %% Recovery timeout management (persistent state tracking needed)
    persistent recovery_start_step;
    persistent last_sensor_fail_count;
    persistent sensor_fail_grace_counter;
    
    % Initialize persistent variables
    if isempty(recovery_start_step)
        recovery_start_step = 0;
    end
    if isempty(last_sensor_fail_count)
        last_sensor_fail_count = 0;
    end
    if isempty(sensor_fail_grace_counter)
        sensor_fail_grace_counter = 0;
    end
    
    % Recovery timeout configuration
    recovery_timeout_steps = 5000; % Max steps before timeout (5 seconds at 1ms)
    grace_period_steps = 500; % Grace period before reacting to sensor loss (0.5s)
    
    %% Sensor pattern diagnosis
    % Identify which sensor pair is lost
    command.pitch_sensors_lost = ~command.contact(1) && ~command.contact(2);
    command.roll_sensors_lost = ~command.contact(3) && ~command.contact(4);
    command.diagonal_sensors_lost = (~command.contact(1) && ~command.contact(4)) || ...
                                     (~command.contact(2) && ~command.contact(3));
    
    %% Grace period for sensor failures (avoid reacting to transient glitches)
    if command.sensor_fail >= 2
        if command.sensor_fail == last_sensor_fail_count
            % Failure is persistent
            sensor_fail_grace_counter = sensor_fail_grace_counter + 1;
        else
            % Failure count changed: reset grace counter
            sensor_fail_grace_counter = 0;
        end
    else
        % No significant failure: reset grace counter
        sensor_fail_grace_counter = 0;
    end
    last_sensor_fail_count = command.sensor_fail;
    
    % Mark as persistent failure if beyond grace period
    command.sensor_fail_persistent = (sensor_fail_grace_counter > grace_period_steps);
    
    %% Track recovery state entry and timeout
    if strcmp(c_state, 'MovePitch') || strcmp(c_state, 'MoveRoll') || strcmp(c_state, 'RecoveryAltitude')
        if recovery_start_step == 0
            % Just entered recovery state
            recovery_start_step = step;
        end
        % Check if timeout reached
        if (step - recovery_start_step) > recovery_timeout_steps
            command.recovery_timeout = true;
        else
            command.recovery_timeout = false;
        end
        
        % Track progress: improvement in sensor count since entering recovery
        if command.sensor_fail < last_sensor_fail_count
            command.recovery_progress = true;
        else
            command.recovery_progress = false;
        end
    else
        % Not in recovery state: reset timeout tracking
        recovery_start_step = 0;
        command.recovery_timeout = false;
        command.recovery_progress = false;
    end
    
    %% Altitude state check
    if strcmp(c_state, 'TargetAltitude')
        if abs(gg.altitude - x_ekf(IND_H)) < alt_eps
            command.setpoint = true;
            command.emergency = false;
        end
    end

    %% Reset state check
    if strcmp(c_state, 'Reset')
        command.reset = true;
        for j = 1:dim_a
            if abs(ang(j) - 0) > deg2rad(10) 
                command.reset = false;
                break;
            end
        end
    end

    %% Emergency State
    if strcmp(c_state, 'Emergency')
        command.setpoint = false;
        command.emergency = false; % Clear emergency once in emergency state
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
end