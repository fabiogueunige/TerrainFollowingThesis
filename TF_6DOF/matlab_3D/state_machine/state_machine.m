function [next_state] = state_machine(c_state, commands, step)
    % casi inizio, esplorazione, following, 
    switch c_state
        case 'Idle'
            if commands.start
                next_state = 'TargetAltitude'; % 'ContactSearch';
                fprintf('State changed to TargetAltitude %.0f.\n', step);
            else
                next_state = 'Idle';
            end
        case 'Reset'
            if commands.reset
                next_state = 'TargetAltitude';
                fprintf('State changed to TargetAltitude %.0f.\n', step);
            else
                next_state = 'Reset';
            end
        case 'TargetAltitude'
            if commands.setpoint
                next_state = 'ContactSearch';
                fprintf('State changed to ContactSearch. %.0f.\n', step);
            else
                next_state = 'TargetAltitude';
            end
        case 'ContactSearch'
            if commands.sensor_fail == 0
                next_state = 'Following';
                fprintf('State changed to Following. %.0f.\n', step);
            elseif commands.sensor_fail == 4
                % Completely loosed all 4 of them 
                    next_state = 'Reset';
                    fprintf('State changed to Reset. %.0f.\n', step);
            elseif commands.sensor_fail > 1 && commands.sensor_fail < 4
                if ~commands.contact1 && ~commands.contact2  
                    next_state = 'MovePitch';
                    fprintf('State changed to MovePitch. %.0f.\n', step);
                elseif ~commands.contact3 && ~commands.contact4
                    next_state = 'MoveRoll';
                    fprintf('State changed to MoveRoll. %.0f.\n', step);
                else
                    next_state = 'ContactSearch';
                end
            else
                % When one pitch and 1 roll is not working
                next_state = 'ContactSearch';
            end
            % Security Commands
            if ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('State changed to  target altitude %.0f.\n', step);
            end
            if commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency. %.0f.\n', step);
            end
        case 'MovePitch' %% TO IMPROVE
            if commands.contact1 && commands.contact2
                next_state = 'ContactSearch';
                fprintf('Back to ContactSearch. %.0f.\n', step);
            else
                next_state = 'MovePitch';
            end
            % Security Commands
            if ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('State changed to  target altitude %.0f.\n', step);
            end
            if commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency %.0f.\n', step);
            end
        case 'MoveRoll' %% TO IMPROVE
            if commands.contact3 && commands.contact4
                next_state = 'ContactSearch';
                fprintf('State changed back to ContactSearch %.0f.\n', step);
            else
                next_state = 'MoveRoll';
            end
            % Security Commands
            if ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('State changed to  target altitude %.0f.\n', step);
            end
            if commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency %.0f.\n', step);
            end
        % case SlowFollowing when 2 different echosonar are not wotking
        case 'Following'
            if commands.sensor_fail >= 2 && commands.sensor_fail < 4
                if ~commands.contact1 && ~commands.contact2  
                    next_state = 'MovePitch';
                    fprintf('State changed to MovePitch from Following %.0f.\n', step);
                elseif ~commands.contact3 && ~commands.contact4
                    next_state = 'MoveRoll';
                    fprintf('State changed to MoveRoll from Following %.0f.\n', step);
                else
                    next_state = 'Following';
                end
            elseif commands.sensor_fail == 4
                % Completely loosed all 4 of them 
                    next_state = 'Reset';
                    fprintf('State changed to Reset %.0f.\n', step);
            else
                next_state = 'Following';
            end
            if ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('State changed to  target altitude %.0f.\n', step);
            end
            if commands.end
                next_state = 'EndSimulation';
                fprintf('State changed to EndSimulation %.0f.\n', step);
            end
            if commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency %.0f.\n', step);
            end
        case 'Emergency'
            if ~commands.setpoint || commands.emergency
                next_state = 'TargetAltitude';
                fprintf('State changed to  target altitude %.0f.\n', step);
            else
                % Non capisco quando potrebbe succedere
                next_state = 'EndSimulation';
            end
        
        case 'EndSimulation'
            next_state = 'EndSimulation';
            fprintf('Simulation ended. Obrigado!!\n');
    end
end