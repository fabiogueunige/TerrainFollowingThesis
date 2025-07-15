function [pid, int_term, prev_err, int_err] = input_control(x, angles, old_pid, int_term, speed, acc, wRr, wRt,...
                                    Ts, dim_i, prev_err, int_err)
    %% Definitions
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;

    printDebug('       Input Control\n');
    
    %% PID parameters
    % --- for now random parameters -- %
    [Kp, Ki, Kd, Ti, Td, Kt] = gainComputation(speed, dim_i);

    %% Desiired Parameters
    u_star = 0.3;
    v_star = 0.0;
    global h_ref;
    yaw_des = 0;

    %% Limitation Parameters
    max_pid = ones(dim_i, 1);
    term_sum = zeros(dim_i, 1);
    pid = zeros(dim_i, 1);

    integral_max = 1.0;

    %% Errors
    s_speed = wRt' * wRr * speed(SURGE:HEAVE);
    s_acc = wRt' * wRr * acc(SURGE:HEAVE);

    err = zeros (dim_i, 1);
    err(SURGE) = (u_star - s_speed(SURGE));
    err(SWAY) = (v_star - s_speed(SWAY));
    err(HEAVE) = (h_ref - x(IND_H));
    err(ROLL) = (x(ALPHA) - angles(PHI));
    err(PITCH) = (x(BETA) - angles(THETA));
    err(YAW) = yaw_des - angles(PSI);

    %% QUESTO CONTROLLO NON FUNZIONA
    % for j = 1:dim_i
    %     i_pid = Ki(j) * err(j);
    %     if j == SURGE || j == SWAY
    %         p_pid = -(Kp(j) * s_acc(j));
    %         d_pid = 0;
    %     elseif j == HEAVE
    %         p_pid = -(Kp(j) * s_speed(j));
    %         d_pid = -(Kd(j) * s_acc(j));
    %     else
    %         p_pid = -(Kp(j) * speed(j));
    %         d_pid = -(Kd(j) * acc(j));
    %     end
    %     term_sum(j) = i_pid - p_pid - d_pid;
    % end
    % % Computation on robot frame for 
    % tp_speed = wRr' * wRt * [term_sum(SURGE); term_sum(SWAY); term_sum(HEAVE)];
    % % Computing final controller
    % for j = 1:dim_i
    %     if j < ROLL
    %         term_sum(j) = tp_speed(j);
    %     end
    % 
    %     term_sum(j) = term_sum(j) - Kt(j)*old_pid(j);
    %     int_term(j) = int_term(j) + term_sum(j)*Ts;
    % 
    %     %u_sat = max(min(int_term(j), max_pid(j)), -max_pid(j));
    % 
    %     pid(j) = int_term(j); % - u_sat;
    % end


    %% SIMPLE WORKING CONTROLLER
    % Pid Computation
    for j = 1:dim_i
        int_err(j) = int_err(j) + err(j) * Ts;
        der_err = (err(j) - prev_err(j)) / Ts;
        % Anti-windup for the integrator
        int_err(j) = max(min(int_err(j), integral_max), -integral_max); % Clamp integral error

        % sum of the terms
        if j == SURGE || j == SWAY 
            term_sum(j) = (Kp(j) * err(j) + Ki(j) * int_err(j));
        else
            term_sum(j) = (Kp(j) * err(j) + Ki(j) * int_err(j) + Kd(j) * der_err);
        end
    end
    tp_speed = wRr' * wRt * [term_sum(SURGE); term_sum(SWAY); term_sum(HEAVE)];
    for j = 1:dim_i
        if j < ROLL
            term_sum(j) = tp_speed(j);
        end
        pid(j) = max(min(term_sum(j), max_pid(j)), -max_pid(j));

        % Memory update
        prev_err(j) = err(j);        
    end

    %% FINAL
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SURGE), pid(SURGE));
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SWAY), pid(SWAY));
    printDebug('Error: %.2f | w_ref: %.3f m/s\n', err(HEAVE), pid(HEAVE));
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err(ROLL)), rad2deg(pid(ROLL)));
    printDebug('Error: %.2f | q_ref: %.3f rad/s\n', rad2deg(err(PITCH)), rad2deg(pid(PITCH)));
end


function [kp, ki, kd, Ti, Td, Kt] = gainComputation(speed0, dim_i)
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;
    %% Linearization Parameters
    wn = 0.3;
    damp = 0.6;
    p = 10;

    kp = zeros(dim_i, 1);
    ki = zeros(dim_i, 1);
    kd = zeros(dim_i, 1);
    Ti = zeros(dim_i, 1);
    Td = zeros(dim_i, 1);
    Kt = zeros(dim_i, 1);

    %% Model
    m = 11.5; % massa totale [kg]
    I = diag([0.21, 0.245, 0.245]);

    % Added mass
    tau_a = [27.08; 25.952; 29.9081; 1; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

    % Dissipative forces (con v0 = 0.1 solo nel surge)
    dv = -tau_r - 2 * tau_d .* abs(speed0);
    
    %% Computation
    for l = 1:dim_i
        if l == SURGE || l == SWAY
            kp(l) = 2*damp*wn*mv(l) - dv(l);
            ki(l) = wn^2 * mv(l);
            kd(l) = 0;
            Ti(l) = kp(l)/ki(l);
            Kt(l) = 1/Ti(l);
        else
            kp(l) = mv(l)*((wn^2) + 2*damp*p*wn);
            ki(l) = p*(wn^2)*mv(l);
            kd(l) = (p + 2*damp*wn)*mv(l) - dv(l);
            Ti(l) = kp(l)/ki(l);
            Td(l) = kd(l)/kp(l);
            Kt(l) = 1 / sqrt(Ti(l)*Td(l));
        end
    end
end