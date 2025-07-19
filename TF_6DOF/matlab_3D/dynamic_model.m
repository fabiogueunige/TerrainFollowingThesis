function [new_vel] = dynamic_model(tau, tau0, speed0, old_vel, Ts, i_dim, old_acc)
    % updated to bluerov model
    printDebug('       Dynamic Model\n');
    %% Definition
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;

    %% Model
    m = 11.5; % massa totale [kg]
    I = diag([0.21, 0.245, 0.245]);

    % [surge, heave, roll, pitch]  aggiungere valori ROLL blue rov
    % Added mass
    tau_a = -[27.08; 25.952; 29.9081; 1; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% TO CHANGE WITH YAW ACTUATION %%%%%%%%%%%%%
    sp0 = [speed0(SURGE), speed0(SWAY), speed0(HEAVE), speed0(ROLL), speed0(PITCH), 0];
    dv = -tau_r - tau_d .* abs(sp0);
    dv_lin = -tau_r - 2 * tau_d .* abs(sp0);

    delta = old_vel - speed0;

    if i_dim ~= size(speed0)
        error('Wrong size between i_dim and generated values');
    end

    % Accelerations
    % 6 DOF MODEL Linearized
    s_dotdot = zeros(i_dim,1);
    
    % surge
    s_dotdot(SURGE) = (tau(SURGE) - tau0(SURGE) - dv_lin(U)*delta(SURGE)) / mv(U);

    % sway
    s_dotdot(SWAY) = (tau(SWAY) - tau0(SWAY) - dv_lin(V)*delta(SWAY)) / mv(V);
 
    % heave
    s_dotdot(HEAVE) = (tau(HEAVE) - tau0(HEAVE) - dv_lin(W)*delta(HEAVE)) / mv(W);

    % roll
    s_dotdot(ROLL) = (tau(ROLL) - tau0(ROLL) - dv_lin(P)*delta(ROLL)) / mv(P);
    
    % pitch
    s_dotdot(PITCH) = (tau(PITCH) - tau0(PITCH) - dv_lin(Q)*delta(PITCH)) / mv(Q);

    % yaw ...
  
    new_vel = integrator(old_vel, s_dotdot, old_acc, Ts);

    printDebug('surge: %.2f | sway: %.2f | heave: %.2f ', new_vel(SURGE), new_vel(SWAY), new_vel(HEAVE));
    printDebug('| p: %.3f | q: %.3f\n', new_vel(ROLL), new_vel(PITCH));
    printDebug('a_surge: %.2f | a_sway: %.2f | a_heave: %.2f ', s_dotdot(SURGE), s_dotdot(SWAY), s_dotdot(HEAVE));
    printDebug('| a_p: %.3f | a_q: %.3f\n', s_dotdot(ROLL), s_dotdot(PITCH));
end