function [next_state] = state_machine(c_state, commands)
    % casi inizio, esplorazione, following, 
    switch c_state
        case 'Idle'
            if commands.start
                next_state = 'TargetAltitude'; % 'ContactSearch';
                fprintf('State changed to TargetAltitude.\n');
            else
                next_state = 'Idle';
            end
        case 'TargetAltitude'
            if commands.setpoint
                next_state = 'ContactSearch';
                fprintf('State changed to ContactSearch.\n');
            else
                next_state = 'TargetAltitude';
            end
        case 'ContactSearch'
            if commands.sensor_fail < 2
                next_state = 'Following';
                fprintf('State changed to Following.\n');
            elseif commands.emergency % more than 3
                next_state = 'Emergency';
                fprintf('State changed to Emergency.\n');
                pause(0.005);
            elseif ~commands.contact1 && ~commands.contact2
                % number 1 and 2 is not detected
                    next_state = 'MovePitch';
                    fprintf('State changed to MovePitch.\n');
            elseif ~commands.contact3 && ~commands.contact4
                    next_state = 'MoveRoll';
                    fprintf('State changed to MoveRoll.\n');
            else
                % No idea when
                next_state = 'ContactSearch';
                fprintf('State remain the same because two different failed.\n');
                pause(0.005);
            end
        case 'MovePitch' %% TO IMPROVE
            if commands.contact1 || commands.contact2
                next_state = 'ContactSearch';
                fprintf('State changed back to ContactSearch.\n');
            elseif commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency.\n');
                pause(0.005);
            else
                next_state = 'MovePitch';
            end
        case 'MoveRoll' %% TO IMPROVE
            if commands.contact3 || commands.contact4
                next_state = 'ContactSearch';
                fprintf('State changed back to ContactSearch.\n');
            elseif commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency.\n');
                pause(0.005);
            else
                next_state = 'MoveRoll';
            end
        case 'Following'
            if commands.following
                next_state = 'Following';
            elseif commands.end
                next_state = 'EndSimulation';
                fprintf('State changed to EndSimulation.\n');
            elseif commands.sensor_fail >= 2
                next_state = 'ContactSearch';
                fprintf('State changed to ContactSearch.\n');
            elseif commands.emergency
                next_state = 'TargetAltitude';
                fprintf('State changed to TargetAltitude to handle emergency.\n');
            else
                next_state = 'Following';
                fprintf('Succede qualcosa non considerato')
                pause(1);
            end
        % case 'Acceleration' %% TO IMPROVE
        % case 'Deceleration' %% TO IMPROVE
        case 'Emergency'
            if commands.sensor_fail >= 3 || ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('From Emergency to Target Altitude');
            else
                % Non capisco quando potrebbe succedere
                next_state = 'Emergency';
            end
        % case 'Reset'
        case 'EndSimulation'
            next_state = 'EndSimulation';
            fprintf('Simulation ended. Obrigado!!\n');
    end
end