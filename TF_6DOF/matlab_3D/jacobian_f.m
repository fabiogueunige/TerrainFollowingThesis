function F = jacobian_f(x, u, Ts, num_n, psi, wRt, wRr)
    % Jacobian of the dynamics with respect to x and u
    %% Definitions
    IND_H = 1;                  I_IND_U = 1;
    ALPHA = 2;                  I_IND_W = 2;
    BETA = 3;
    PHI = 4;      M_PHI = 5;
    THETA = 5;    M_THETA = 6;
    IND_P = 6;    M_IND_P = 7;  I_IND_P = 3;
    IND_Q = 7;    M_IND_Q = 8;  I_IND_Q = 4; 
    fprintf('       jacobian F Covariance\n')

    %% Computations
    F = eye(num_n);
    v = 0;
    r_speedTs = [u(I_IND_U); v; u(I_IND_W)] * Ts;

    %% Derivatives for the altitude
    % F(IND_H, IND_H) = 1;
    h_alpha = rotx(pi) * (d_rotx(x(ALPHA)))' * (roty(x(BETA)))' * wRr*r_speedTs;
    h_beta = rotx(pi) * (rotx(x(ALPHA)))' * (d_roty(x(BETA)))' * wRr*r_speedTs;
    F(IND_H, ALPHA) = h_alpha(3);
    F(IND_H, BETA) = h_beta(3);
    %% ---- CONTRTOLLARE SE GIUSTO IL TERMINE FINALE ---- % RIMETTEREEE
    h_phi = (wRt' * (rotz(psi)*roty(x(THETA))*d_rotx(x(PHI))) * r_speedTs); %*x(IND_P);
    h_theta = (wRt' * (rotz(psi)*d_roty(x(THETA))*rotx(x(PHI))) * r_speedTs); %*x(IND_Q);
    F(IND_H, PHI) = h_phi(3);
    F(IND_H, THETA) = h_theta(3);

    %% Derivatives for the angles
    F(PHI, IND_P) = Ts;
    F(THETA, IND_Q) = Ts;
end