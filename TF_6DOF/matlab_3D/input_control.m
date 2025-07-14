function [pid, du_int, prev_err, int_err] = input_control(x, angles, old_pid, old_du_int, speed, acc, wRr, wRt,...
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
    global h_ref

    %% Limitation Parameters
    max_pid = ones(dim_i, 1);
    du_int = zeros(dim_i, 1);
    term_sum = zeros(dim_i, 1);
    pid = zeros(dim_i, 1);

    integral_max = 1.0;

    %% Errors
    err = zeros (dim_i, 1);
    s_speed = wRt' * wRr * speed(SURGE:HEAVE);
    % s_acc = wRt' * wRr * acc(SURGE:HEAVE);

    err(SURGE) = (u_star - s_speed(SURGE));
    err(SWAY) = (v_star - s_speed(SWAY));
    err(HEAVE) = (h_ref - x(IND_H));
    err(ROLL) = (x(ALPHA) - angles(PHI));
    err(PITCH) = (x(BETA) - angles(THETA));
    % err(YAW) = ???

    %% QUESTO CONTROLLO NON FUNZIONA
    % for j = 1:SURGE % Gli altri non funzionano, come anche il surge
    %     d_i = Ki(j) * err(j);
    %     if j == SURGE % PI Controller
    %         d_p = Kp(j) * acc(j);
    %         d_d = 0;
    %     else % attenzione a come gestire heave
    %         d_p = Kp(j) * speed(j);
    %         d_d = Kd(j) * acc(j);
    %     end
    %     % All term toghether plus old_pid
    %     term_sum = d_i + d_p + d_d - Kt(j) * old_pid(j);
    %     du_int(j) = old_du_int(j) + Ts*term_sum;
    %     % saturation 
    %     du_sat = max(min(du_int(j), max_pid(j)), -max_pid(j));
    % 
    %     pid(j) = du_int(j) - du_sat;
    % end

    %% ERROR CALCULATION FOR THE OTHERS
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

    % Trasformation to robot frame
    tp_speed = wRr' * wRt * term_sum(SURGE:HEAVE);

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
    wn = 0.2;
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

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% TO CHANGE WITH YAW ACTUATION %%%%%%%%%%%%%
    % Added mass
    tau_a = [27.08; 25.952; 29.9081; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; m; I(1,1); I(2,2)] - tau_a;

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