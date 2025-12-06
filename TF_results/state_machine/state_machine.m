function [next_state] = state_machine(c_state, commands, step)
    % State machine with improved sensor failure handling and recovery logic
    % Handles loss of sensor pairs (1-2 pitch, 3-4 roll) with timeout and retry limits
    
    switch c_state
        case 'Idle'
            if commands.start
                next_state = 'TargetAltitude';
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
            % All sensors working: proceed to following
            if commands.sensor_fail == 0
                next_state = 'Following';
                fprintf('State changed to Following. %.0f.\n', step);
            % Total loss: reset
            elseif commands.sensor_fail == 4
                next_state = 'Reset';
                fprintf('State changed to Reset (all sensors lost). %.0f.\n', step);
            % Partial loss: identify pattern and recover
            elseif commands.sensor_fail >= 2
                if commands.pitch_sensors_lost
                    next_state = 'MovePitch';
                    fprintf('State changed to MovePitch (sensors 1-2 lost). %.0f.\n', step);
                elseif commands.roll_sensors_lost
                    next_state = 'MoveRoll';
                    fprintf('State changed to MoveRoll (sensors 3-4 lost). %.0f.\n', step);
                elseif commands.diagonal_sensors_lost
                    % Diagonal loss: try altitude adjustment first
                    next_state = 'RecoveryAltitude';
                    fprintf('State changed to RecoveryAltitude (diagonal sensors lost). %.0f.\n', step);
                else
                    % Single sensor from each pair: stay in search
                    next_state = 'ContactSearch';
                end
            else
                % Only 1 sensor lost: continue searching
                next_state = 'ContactSearch';
            end
            
            % Security overrides
            if ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('State changed to target altitude %.0f.\n', step);
            end
            if commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency. %.0f.\n', step);
            end
            
        case 'MovePitch'
            % Check timeout
            if commands.recovery_timeout
                next_state = 'RecoveryAltitude';
                fprintf('MovePitch timeout, switching to RecoveryAltitude. %.0f.\n', step);
            % Success: both pitch sensors recovered
            elseif commands.contact(1) && commands.contact(2)
                next_state = 'ContactSearch';
                fprintf('Pitch sensors recovered, back to ContactSearch. %.0f.\n', step);
            else
                next_state = 'MovePitch';
            end
            
            % Security overrides
            if ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('State changed to target altitude %.0f.\n', step);
            end
            if commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency %.0f.\n', step);
            end
            
        case 'MoveRoll'
            % Check timeout
            if commands.recovery_timeout
                next_state = 'RecoveryAltitude';
                fprintf('MoveRoll timeout, switching to RecoveryAltitude. %.0f.\n', step);
            % Success: both roll sensors recovered
            elseif commands.contact(3) && commands.contact(4)
                next_state = 'ContactSearch';
                fprintf('Roll sensors recovered, back to ContactSearch %.0f.\n', step);
            else
                next_state = 'MoveRoll';
            end
            
            % Security overrides
            if ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('State changed to target altitude %.0f.\n', step);
            end
            if commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency %.0f.\n', step);
            end
            
        case 'RecoveryAltitude'
            % Try to recover by adjusting altitude
            if commands.recovery_timeout
                next_state = 'Reset';
                fprintf('RecoveryAltitude timeout, forcing Reset. %.0f.\n', step);
            elseif commands.sensor_fail == 0
                next_state = 'ContactSearch';
                fprintf('Sensors recovered via altitude, back to ContactSearch. %.0f.\n', step);
            elseif commands.sensor_fail < 4 && commands.recovery_progress
                % Some improvement: continue
                next_state = 'RecoveryAltitude';
            else
                next_state = 'RecoveryAltitude';
            end
            
            % Security overrides
            if commands.emergency
                next_state = 'Emergency';
                fprintf('State changed to Emergency %.0f.\n', step);
            end
            
        case 'Following'
            % Monitor sensor health during following
            if commands.sensor_fail >= 2
                % Grace period check: avoid immediate reaction to transient losses
                if commands.sensor_fail_persistent
                    if commands.pitch_sensors_lost
                        next_state = 'MovePitch';
                        fprintf('State changed to MovePitch from Following %.0f.\n', step);
                    elseif commands.roll_sensors_lost
                        next_state = 'MoveRoll';
                        fprintf('State changed to MoveRoll from Following %.0f.\n', step);
                    elseif commands.diagonal_sensors_lost
                        next_state = 'RecoveryAltitude';
                        fprintf('State changed to RecoveryAltitude from Following %.0f.\n', step);
                    else
                        next_state = 'Following'; % Transient or single loss
                    end
                else
                    % Within grace period: stay in following
                    next_state = 'Following';
                end
            elseif commands.sensor_fail == 4
                next_state = 'Reset';
                fprintf('State changed to Reset (all sensors lost) %.0f.\n', step);
            else
                next_state = 'Following';
            end
            
            % Security overrides
            if ~commands.setpoint
                next_state = 'TargetAltitude';
                fprintf('State changed to target altitude %.0f.\n', step);
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
            if ~commands.emergency
                next_state = 'TargetAltitude';
                fprintf('Emergency cleared, state changed to target altitude %.0f.\n', step);
            else
                next_state = 'Emergency';
            end
        
        case 'EndSimulation'
            next_state = 'EndSimulation';
            fprintf('Simulation ended. Obrigado!!\n');
            
        otherwise
            next_state = 'Idle';
            fprintf('Unknown state, reverting to Idle. %.0f.\n', step);
    end
end