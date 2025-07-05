function [new_vel, s_dotdot] = dynamic_model(tau, tau0, speed0, old_vel, Ts, i_dim)
    % updated to bluerov model
    fprintf('       Dynamic Model\n');
    %% Definition
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;

    %% Model
    m = 11.5; % massa totale [kg]
    I = diag([0.21, 0.245, 0.245]);

    % [surge, heave, roll, pitch]  aggiungere valori ROLL blue rov
    % Added mass
    tau_a = [27.08; 25.952; 29.9081; 1; 1; 1]; % kl_dot 
    
    % Linear damping
    tau_r = [-0.1213; -1.1732; -1.1130; -0.5; -0.5; -0.5]; % linear_damping (kl)
    
    % Quadratic damping
    tau_d = [-23.9000; -46.2700; -50.2780; -1; -1; -1]; % quadratic_damping (kl_modl)
    
    % Virtual mass
    mv = [m; m; m; I(1,1); I(2,2); I(3,3)] - tau_a;

    % Dissipative forces (con v0 = 0.1 solo nel surge)
    sp0 = [speed0(I_IND_U), 0, speed0(I_IND_W), speed0(I_IND_P), speed0(I_IND_Q), 0];
    dv = -tau_r - 2 * tau_d .* abs(sp0);

    delta = old_vel - speed0;

    if i_dim ~= size(speed0)
        error('Wrong size between i_dim and generated values');
    end

    % Accelerations
    % 6 DOF MODEL Linearized
    s_dotdot = zeros(i_dim,1);
    
    % surge
    s_dotdot(I_IND_U) = (tau(I_IND_U) - tau0(I_IND_U) - dv(U)*delta(I_IND_U)) / mv(U);

    % sway ...
 
    % heave
    s_dotdot(I_IND_W) = (tau(I_IND_W) - tau0(I_IND_W) - dv(W)*delta(I_IND_W)) / mv(W);

    % roll
    s_dotdot(I_IND_P) = (tau(I_IND_P) - tau0(I_IND_P) - dv(P)*delta(I_IND_P)) / mv(P);
    
    % pitch
    s_dotdot(I_IND_Q) = (tau(I_IND_Q) - tau0(I_IND_Q) - dv(Q)*delta(I_IND_Q)) / mv(Q);

    % roll ...
  
    new_vel = old_vel + Ts * s_dotdot;

    fprintf('surge: %.2f | heave: %.2f | p: %.3f | q: %.3f\n', new_vel(I_IND_U), new_vel(I_IND_W), new_vel(I_IND_P), new_vel(I_IND_Q));
    fprintf('a_surge: %.2f | a_heave: %.2f | a_p: %.3f | a_q: %.3f\n', s_dotdot(I_IND_U), s_dotdot(I_IND_W), s_dotdot(I_IND_P), s_dotdot(I_IND_Q));
end