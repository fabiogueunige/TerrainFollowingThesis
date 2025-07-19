function [new_vel, s_dotdot] = dynamic_model(tau, ang, v_old, Ts, i_dim)
    % updated to bluerov model
    printDebug('       Dynamic Model\n');
    %% Definition
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
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

    % Added mass
    tau_a = -[27.08; 25.952; 29.9081; 1; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

    dv = -tau_r - tau_d .* abs(v_old);

%%%%%% EQUAZIONI SBAGLIATE PERCHE MODELLO TROPPO SEMPLIFICATO %%%%%%

    % 6 DOF MODEL NOT Linearized
    s_dotdot = zeros(i_dim,1);
    
    % surge
    s_dotdot(U) = (tau(U) + mv(V)*v_old(V)*v_old(R) - mv(W)*v_old(W)*v_old(Q) - dv(U)*v_old(U)) / mv(U);

    % sway
    s_dotdot(V) = (tau(V) - mv(U)*v_old(U)*v_old(R) + mv(W)*v_old(W)*v_old(P) - dv(V)*v_old(V)) / mv(V);
 
    % heave
    s_dotdot(W) = (tau(W) + mv(U)*v_old(U)*v_old(Q) - mv(V)*v_old(V)*v_old(P) - dv(W)*v_old(W)) / mv(W);

    % roll
    s_dotdot(P) = (tau(P) + (mv(V)-mv(W))*v_old(V)*v_old(W) + (mv(Q)-mv(R))*v_old(Q)*v_old(R) ...
                - dv(P)*v_old(P) - z*B*cos(ang(THETA))*sin(ang(PHI))) / mv(P);
    
    % pitch
    s_dotdot(Q) = (tau(Q) - (mv(U)-mv(W))*v_old(U)*v_old(W) - (mv(P)-mv(R))*v_old(P)*v_old(R) ...
                - dv(Q)*v_old(Q) + z*B*sin(ang(THETA))) / mv(Q);

    % yaw ...
    s_dotdot(R) = (tau(R) + (mv(U)-mv(V))*v_old(U)*v_old(V) + (mv(P)-mv(Q))*v_old(P)*v_old(Q) ...
                - dv(R)*v_old(R)) / mv(R);
  
    new_vel = v_old + Ts * s_dotdot;

    fprintf('surge: %.2f | sway: %.2f | heave: %.2f ', new_vel(U), new_vel(V), new_vel(W));
    fprintf('| p: %.3f | q: %.3f | r: %.3f\n', new_vel(P), new_vel(Q), new_vel(R));
    printDebug('a_surge: %.2f | a_sway: %.2f | a_heave: %.2f ', s_dotdot(U), s_dotdot(V), s_dotdot(W));
    printDebug('| a_p: %.3f | a_q: %.3f | a_r: %.3f\n', s_dotdot(P), s_dotdot(Q), s_dotdot(R));
end