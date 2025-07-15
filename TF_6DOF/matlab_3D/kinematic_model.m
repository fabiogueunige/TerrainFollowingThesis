function [st_new] = kinematic_model(u, Ts, sto, dim_r)
    %% TO KNOW:

%%%%%% EQUAZIONI SBAGLIATE PERCHE MODELLO TROPPO SEMPLIFICATO %%%%%%

    % sto = old state of the robot
    %% DEFINE
    % Kinematics
    X = 1; Y = 2; Z = 3;
    PH = 4; TH = 5; PS = 6;
    % Dynamics
    U = 1;      V = 2;      W = 3;
    P = 4;      Q = 5;      R = 6;
    %% OPERATIONS
    % rapresent the kinematic of the robot
    k_vel = zeros(dim_r, 1);
    % k_vel(X:Z) = for now not doing it
    k_vel(PH) = u(P) + u(Q)*sin(sto(PH))*tan(sto(TH)) + u(R)*cos(sto(PH))*tan(sto(TH));
    k_vel(TH) = u(Q)*cos(sto(PH)) - u(R)*sin(sto(PH));
    k_vel(PS) = (u(Q)*sin(sto(PH)))/cos(sto(TH)) + u(R)*cos(sto(PH))/cos(sto(TH));

    st_new = sto + k_vel * Ts;
end