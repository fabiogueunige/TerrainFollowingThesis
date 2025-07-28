function [new_vel] = dynamic_model(tau, tau0, sp0, ang, v_old, Ts, i_dim, old_acc)
    % updated to bluerov model
    printDebug('       Dynamic Model\n');
    %% Definition
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    global SURGE; global SWAY; global HEAVE;
    global ROLL; global PITCH; global YAW;
    global PHI; global THETA; global PSI; 

    %% Model
    m = 11.5; % massa totale [kg]
    I = diag([0.21, 0.245, 0.245]);
    rho = 1028;
    volume = 0.011054;
    g = 9.81;
    % restoring
    z = 0.0420;
    B = rho * volume * g;

    % [surge, heave, roll, pitch]  aggiungere valori ROLL blue rov
    % Added mass
    tau_a = -[27.08; 25.952; 29.9081; 1; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;
    
    dv = -tau_r - tau_d .* abs(v_old);
    
    % Accelerations
    s_dotdot = zeros(i_dim,1);

    %% LINEARIZED
    dv_lin = -tau_r - 2 * tau_d .* abs(sp0);
    delta = v_old - sp0;
    
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
    s_dotdot(YAW) = (tau(YAW) - tau0(YAW) - dv_lin(R)*delta(YAW)) / mv(R);

    %% NEW NOT LINEARIZED
    % % surge
    % s_dotdot(U) = (tau(U) + mv(V)*v_old(V)*v_old(R) - mv(W)*v_old(W)*v_old(Q) - dv(U)*v_old(U)) / mv(U);
    % % sway
    % s_dotdot(V) = (tau(V) - mv(U)*v_old(U)*v_old(R) + mv(W)*v_old(W)*v_old(P) - dv(V)*v_old(V)) / mv(V);
    % % heave
    % s_dotdot(W) = (tau(W) + mv(U)*v_old(U)*v_old(Q) - mv(V)*v_old(V)*v_old(P) - dv(W)*v_old(W)) / mv(W);
    % % roll
    % s_dotdot(P) = (tau(P) + (mv(V)-mv(W))*v_old(V)*v_old(W) + (mv(Q)-mv(R))*v_old(Q)*v_old(R) ...
    %             - dv(P)*v_old(P) - z*B*cos(ang(THETA))*sin(ang(PHI))) / mv(P);
    % % pitch
    % s_dotdot(Q) = (tau(Q) - (mv(U)-mv(W))*v_old(U)*v_old(W) - (mv(P)-mv(R))*v_old(P)*v_old(R) ...
    %             - dv(Q)*v_old(Q) + z*B*sin(ang(THETA))) / mv(Q);
    % % yaw ...
    % % s_dotdot(R) = (tau(R) + (mv(U)-mv(V))*v_old(U)*v_old(V) + (mv(P)-mv(Q))*v_old(P)*v_old(Q) ...
    % %             - dv(R)*v_old(R)) / mv(R);

    %% Velocities Computation
    new_vel = integrator(v_old, s_dotdot, old_acc, Ts);

    printDebug('surge: %.2f | sway: %.2f | heave: %.2f ', new_vel(SURGE), new_vel(SWAY), new_vel(HEAVE));
    printDebug('| p: %.3f | q: %.3f | r: %.3f\n', new_vel(ROLL), new_vel(PITCH), new_vel(YAW));
    printDebug('a_surge: %.2f | a_sway: %.2f | a_heave: %.2f ', s_dotdot(SURGE), s_dotdot(SWAY), s_dotdot(HEAVE));
    printDebug('| a_p: %.3f | a_q: %.3f | a_r: %.3f\n', s_dotdot(ROLL), s_dotdot(PITCH), s_dotdot(YAW));
end