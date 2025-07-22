function [pid, int_term, pre_err, err_i, acc, term_sum] = following_control(x, angles, old_pid, int_term, speed, old_speed, ...
                        acc, old_t_s, speed0, wRr, wRt, Ts, dim_i, pre_err, err_i, step)
    %% Definitions
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;

    printDebug('       Following Control\n');
    
    %% PID parameters
    % --- for now random parameters -- %
    [Kp, Ki, Kd, Kt] = gainComputation(speed0, dim_i);

    %% Desiired Parameters
    u_star = 0.3;
    v_star = 0.0;
    global h_ref;

    %% Limitation Parameters
    max_pid = ones(dim_i, 1);
    term_sum = zeros(dim_i, 1);
    pid = zeros(dim_i, 1);

    %% Errors
    acc = derivator(acc, speed, old_speed, Ts); 
    s_speed = wRt' * wRr * speed(SURGE:HEAVE);
    s_acc = wRt' * wRr * acc(SURGE:HEAVE);

    err = zeros (dim_i, 1);
    err(SURGE) = (u_star - s_speed(SURGE));
    err(SWAY) = (v_star - s_speed(SWAY));
    err(HEAVE) = (h_ref(step) - x(IND_H));
    err(ROLL) = (x(ALPHA) - angles(PHI));
    err(PITCH) = (x(BETA) - angles(THETA));
    % err(YAW) = ???

    %% QUESTO CONTROLLO NON FUNZIONA
    % for j = 1:dim_i
    %     i_err = Ki(j) * err(j);
    %     if j == SURGE || j == SWAY
    %         p_err = (Kp(j) * s_acc(j));
    %         d_err = 0;
    %     elseif j == HEAVE
    %         p_err = (Kp(j) * s_speed(j));
    %         d_err = (Kd(j) * s_acc(j));
    %     else
    %         p_err = (Kp(j) * speed(j));
    %         d_err = (Kd(j) * acc(j));
    %     end
    %     term_sum(j) = i_err - p_err - d_err;
    % end
    % % Computation on robot frame for 
    % tp_speed = wRr' * wRt * [term_sum(SURGE); term_sum(SWAY); term_sum(HEAVE)];
    % % Computing final controller
    % for j = 1:dim_i
    %     if j < ROLL
    %         term_sum(j) = tp_speed(j);
    %     end
    %     term_sum(j) = term_sum(j) - Kt(j)*old_pid(j);
    %     int_term(j) = integrator(int_term(j), term_sum(j), old_t_s(j), Ts);
    %     pid_sat = max(min(int_term(j), max_pid(j)), -max_pid(j));
    %     pid(j) = int_term(j) - pid_sat;
    % end

    %% ERROR CALCULATION PID CLASSICO
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
    tp_speed = wRr' * wRt * [pid(SURGE); pid(SWAY); pid(HEAVE)];
    for j = 1:HEAVE
        pid(j) = tp_speed(j);
    end

    %% FINAL
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SURGE), pid(SURGE));
    printDebug('Error: %.2f | u_ref: %.3f m/s\n', err(SWAY), pid(SWAY));
    printDebug('Error: %.2f | w_ref: %.3f m/s\n', err(HEAVE), pid(HEAVE));
    printDebug('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err(ROLL)), rad2deg(pid(ROLL)));
    printDebug('Error: %.2f | q_ref: %.3f rad/s\n', rad2deg(err(PITCH)), rad2deg(pid(PITCH)));
end


function [kp, ki, kd, kt] = gainComputation(speed0, dim_i)
    global PHI; global THETA; global PSI; 
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;
    %% Linearization Parameters
    wn = 0.2;
    damp = 0.6;
    p = 10;

    kp = zeros(dim_i, 1);
    ki = zeros(dim_i, 1);
    kd = zeros(dim_i, 1);
    Ti = zeros(dim_i, 1);
    Td = zeros(dim_i, 1);
    kt = zeros(dim_i, 1);

    %% Model
    m = 11.5; % massa totale [kg]
    I = diag([0.21, 0.245, 0.245]);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% TO CHANGE WITH YAW ACTUATION %%%%%%%%%%%%%
    % Added mass
    tau_a = -[27.08; 25.952; 29.9081; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; m; I(1,1); I(2,2)] - tau_a;

    % Dissipative forces (con v0 = 0.1 solo nel surge)
    dv = -tau_r - tau_d .* abs(speed0);
    dv_lin = -tau_r - 2 * tau_d .* abs(speed0);
    
    %% Computation
    for l = 1:dim_i
        if l == SURGE || l == SWAY
            kp(l) = 2*damp*wn*mv(l) - dv_lin(l);
            ki(l) = wn^2 * mv(l);
            kd(l) = 0;
            Ti(l) = kp(l)/ki(l);
            kt(l) = 1/Ti(l);
        else
            kp(l) = mv(l)*((wn^2) + 2*damp*p*wn);
            ki(l) = p*(wn^2)*mv(l);
            kd(l) = (p + 2*damp*wn)*mv(l) - dv_lin(l);
            Ti(l) = kp(l)/ki(l);
            Td(l) = kd(l)/kp(l);
            kt(l) = 1 / sqrt(Ti(l)*Td(l));
        end
    end
end