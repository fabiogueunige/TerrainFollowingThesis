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
            if commands.contact1 && commands.contact2 && commands.contact3 && commands.contact4
                next_state = 'Following';
                fprintf('State changed to Following.\n');
            elseif ~commands.contact1 || ~commands.contact2 && commands.contact3 && commands.contact4
                % number 1 and 2 is not detected
                if commands.contact2
                    % just the number 1 not detected %% TO IMPROVE
                    next_state = 'MovePitch';
                    fprintf('State changed to MovePitch 1.\n');
                else
                    % also the number 2 not dected %% TO IMPROVE
                    next_state = 'MovePitch';
                    fprintf('State changed to MovePitch 2.\n');
                end
            elseif commands.contact1 && commands.contact2 && ~commands.contact3 || ~commands.contact4
                % number 3 or 4 is not detected
                if commands.contact4
                    % just the number 3 not detected %% TO IMPROVE
                    next_state = 'MoveRoll';
                    fprintf('State changed to MoveRoll 3.\n');
                else
                    % also the number 4 not detected %% TO IMPROVE
                    next_state = 'MoveRoll';
                    fprintf('State changed to MoveRoll 4.\n');
                end
            else
                % No contact point detected or at least 3 contact point not detected
                next_state = 'Emergency';
                fprintf('State changed to Emergency.\n');
            end
        case 'MovePitch' %% TO IMPROVE
            if commands.contact1 && commands.contact2
                next_state = 'ContactSearch';
                fprintf('State changed back to ContactSearch.\n');
            else
                next_state = 'MovePitch';
            end
        case 'MoveRoll' %% TO IMPROVE
            if commands.contact3 && commands.contact4
                next_state = 'ContactSearch';
                fprintf('State changed back to ContactSearch.\n');
            else
                next_state = 'MoveRoll';
            end
        case 'Following'
            if commands.following
                next_state = 'Following';
            elseif commands.end
                next_state = 'EndSimulation';
                fprintf('State changed to EndSimulation.\n');
            elseif ~commands.contact1 || ~commands.contact2 || ~commands.contact3 || ~commands.contact4
                next_state = 'ContactSearch';
                fprintf('State changed to ContactSearch.\n');
            elseif ~commands.setpoint && commands.emergency
                next_state = 'TargetAltitude';
                fprintf('State changed to TargetAltitude to handle emergency.\n');
            else
                next_state = 'ContactSearch';
                fprintf('Succede qualcosa non considerato')
                pause(1);
            end
        % case 'Acceleration' %% TO IMPROVE
        % case 'Deceleration' %% TO IMPROVE
        case 'Emergency'
            if commands.emergency
            % for now it will just show the problem and stop the simulation!!
                next_state = 'Emergency';
            else
                next_state = 'ContactSearch';
            end
        case 'EndSimulation'
            next_state = 'EndSimulation';
            fprintf('Simulation ended. Obrigado!!\n');
    end
end