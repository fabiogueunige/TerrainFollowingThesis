function [pid, du_int, prev_err, int_err] = input_control(x, angles, old_pid, old_du_int, speed, acc, wRr, wRt,...
                                    Ts, dim_i, prev_err, int_err)
    %% Definitions
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    % angles               
    PHI = 1;        THETA = 2;      PSI = 3;  
    % input
    SURGE = 1;      HEAVE = 2;      ROLL = 3;      PITCH = 4;
    C_U = 1;        C_H = 2;        C_ROLL = 3;     C_PITCH = 4;

    global DEBUG
    printDebug('       Input Control\n');
    
    %% PID parameters
    % --- for now random parameters -- %
    [Kp, Ki, Kd] = gainComputation(speed, dim_i);
    Ti = zeros(dim_i, 1);
    Td = zeros(dim_i, 1);
    Kt = zeros(dim_i, 1);

    for j = 1:dim_i
        Ti(j) = Kp(j)/Ki(j);
        Td(j) = Kd(j)/Kp(j);
        if j == SURGE
            Kt(j) = 1/Ti(j);
        else
            Kt(j) = 1 / sqrt(Ti(j)*Td(j));
        end
    end

    %% Desiired Parameters
    u_star = 0.3;
    h_star = 7;

    %% Limitation Parameters
    max_pid = [0.5; 1; 0.5; 0.5];
    du_int = zeros(dim_i, 1);
    pid = zeros(dim_i, 1);

    integral_max = 1.0;

    %% Errors
    err = zeros (dim_i, 1);
    err(SURGE) = (u_star - speed(SURGE)); % perchè il meno??
    err(HEAVE) = (h_star - x(IND_H));
    err(ROLL) = (x(ALPHA) - angles(PHI));
    err(PITCH) = (x(BETA) - angles(THETA));

    %% Controller SURGE
    for j = 1:SURGE % Gli altri non funzionano, come anche il surge
        d_i = Ki(j) * err(j);
        if j == SURGE % PI Controller
            d_p = Kp(j) * acc(j);
            d_d = 0;
        else % attenzione a come gestire heave
            d_p = Kp(j) * speed(j);
            d_d = Kd(j) * acc(j);
        end
        % All term toghether plus old_pid
        term_sum = d_i + d_p + d_d - Kt(j) * old_pid(j);
        du_int(j) = old_du_int(j) + Ts*term_sum;
        % saturation 
        du_sat = max(min(du_int(j), max_pid(j)), -max_pid(j));

        pid(j) = du_int(j) - du_sat;
    end

    %% ERROR CALCULATION FOR THE OTHERS
    % Pid Computation
    for j = SURGE:dim_i
        int_err(j) = int_err(j) + err(j) * Ts;
        der_err = (err(j) - prev_err(j)) / Ts;
        % Anti-windup for the integrator
        int_err(j) = max(min(int_err(j), integral_max), -integral_max); % Clamp integral error
        
        % sum of the terms
        term_sum = (Kp(j) * err(j) + Ki(j) * int_err(j) + Kd(j) * der_err);
        if j == HEAVE
            tp_speed = wRr' * wRt * [0; 0; term_sum];
            pid(j) = tp_speed(3); % I_IND_W
            term_sum = pid(j);
        end
        pid(j) = max(min(term_sum, max_pid(j)), -max_pid(j));

        % Memory update
        prev_err(j) = err(j);
    end
    
    % pid(SURGE) = u_star;
    % pid(HEAVE) = 0;
    % pid(ROLL) = 0;
    % pid(PITCH) = 0;


    %% FINAL
    fprintf('Error: %.2f | u_ref: %.3f m/s\n', err(SURGE), pid(SURGE));
    fprintf('Error: %.2f | w_ref: %.3f m/s\n', err(HEAVE), pid(HEAVE));
    fprintf('Error: %.2f | p_ref: %.3f rad/s\n', rad2deg(err(ROLL)), rad2deg(pid(ROLL)));
    fprintf('Error: %.2f | q_ref: %.3f rad/s\n', rad2deg(err(PITCH)), rad2deg(pid(PITCH)));
end


function [kp, ki, kd] = gainComputation(speed0, dim_i)
    SURGE = 1;      HEAVE = 2;      ROLL = 3;      PITCH = 4;
    wn = 0.2;
    damp = 0.6;
    p = 10;

    kp = zeros(dim_i, 1);
    ki = zeros(dim_i, 1);
    kd = zeros(dim_i, 1);

    %% Model
    m = 11.5; % massa totale [kg]
    I = diag([0.21, 0.245, 0.245]);

    % [surge, heave, roll, pitch]  aggiungere valori ROLL blue rov
    % Added mass
    tau_a = [27.08; 29.9081; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1130; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -50.2780; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; I(1,1); I(2,2)] - tau_a;

    % Dissipative forces (con v0 = 0.1 solo nel surge)
    dv = -tau_r - 2 * tau_d .* abs(speed0);
    
    %% Computation
    for l = 1:dim_i
        if l == SURGE
            kp(l) = 2*damp*wn*mv(l) - dv(l);
            ki(l) = wn^2 * mv(l);
            kd(l) = 0;
        else
            kp(l) = mv(l)*((wn^2) + 2*damp*p*wn);
            ki(l) = p*(wn^2)*mv(l);
            kd(l) = (p + 2*damp*wn)*mv(l) - dv(l);
        end
    end
end