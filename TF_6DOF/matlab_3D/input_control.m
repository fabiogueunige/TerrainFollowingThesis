function [pid, int_term, pre_err, err_i, acc, term_sum] = input_control(ggg, x, angles, old_pid, int_term, speed, old_speed, ...
                        acc, old_t_s, wRr, wRt, Ts, dim_i, pre_err, err_i, Kp, Ki, Kd, Kt)
    %% Definitions
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;

    printDebug('       Input Control\n');
    
    %% PID parameters
    %%%%%%%%%% STUP CONTROLLER %%%%%%%%%%%%%%%
    type = "A+D";
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % type = "PID";
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Limitation Parameters
    max_pid = ones(dim_i, 1);
    term_sum = zeros(dim_i, 1);
    pid = zeros(dim_i, 1);

    %% Errors
    acc = derivator(acc, speed, old_speed, Ts); 
    s_speed = wRt' * wRr * speed(SURGE:HEAVE);
    s_acc = wRt' * wRr * acc(SURGE:HEAVE);

    err = zeros (dim_i, 1);
    err(SURGE) = (ggg.surge - s_speed(SURGE));
    err(SWAY) = (ggg.sway - s_speed(SWAY));
    err(HEAVE) = (ggg.altitude - x(IND_H));
    err(ROLL) = (ggg.roll - angles(PHI));
    err(PITCH) = (ggg.pitch - angles(THETA));
    % err(YAW) = ???

    %% Controller setup
    if type == "A+D"
        % Delta + Antiwindup
        for j = 1:dim_i
            i_err = Ki(j) * err(j);
            if j == SURGE || j == SWAY
                p_err = (Kp(j) * s_acc(j));
                d_err = 0;
            elseif j == HEAVE
                p_err = (Kp(j) * s_speed(j));
                d_err = (Kd(j) * s_acc(j));
            else
                p_err = (Kp(j) * speed(j));
                d_err = (Kd(j) * acc(j));
            end
            term_sum(j) = i_err - p_err - d_err;
        end
        % Computation on robot frame for 
        tp_speed = wRr' * wRt * [term_sum(SURGE); term_sum(SWAY); term_sum(HEAVE)];
        % Computing final controller
        for j = 1:dim_i
            if j < ROLL
                term_sum(j) = tp_speed(j);
            end
            term_sum(j) = term_sum(j) - Kt(j)*old_pid(j);
            int_term(j) = integrator(int_term(j), term_sum(j), old_t_s(j), Ts);
            pid_sat = max(min(int_term(j), max_pid(j)), -max_pid(j));
            pid(j) = int_term(j) - pid_sat;
        end
    else
        % ERROR CALCULATION PID CLASSICO
        for j = 1:dim_i
            err_i(j) = integrator(err_i(j), err(j), pre_err(j), Ts);
            i_err = Ki(j) * err_i(j);
            p_err = Kp(j) * err(j);
            d_err = 0;
            if j == HEAVE
                d_err = -Kd(j) * s_speed(j);
            end
            if j == ROLL || j == PITCH
                d_err = -Kd(j) * speed(j);   
            end
            pid(j) = i_err + p_err + d_err;
    
            % Memory update
            pre_err(j) = err(j);     
        end
        tp_s_speed = wRr' * wRt * [pid(SURGE); pid(SWAY); pid(HEAVE)];
            
        for j = 1:HEAVE
            pid(j) = tp_s_speed(j);
        end
    end

    %% FINAL
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SURGE), pid(SURGE));
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SWAY), pid(SWAY));
    printDebug('Error: %.2f | w_ref: %.3f m/s\n', err(HEAVE), pid(HEAVE));
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err(ROLL)), rad2deg(pid(ROLL)));
    printDebug('Error: %.2f | q_ref: %.3f rad/s\n', rad2deg(err(PITCH)), rad2deg(pid(PITCH)));
    printDebug('Error: %.2f | r_ref: %.3f rad/s\n', rad2deg(err(YAW)), rad2deg(pid(YAW)));
end