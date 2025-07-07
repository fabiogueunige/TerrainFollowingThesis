function F = jacobian_f(x, u_input, Ts, num_n, psi, wRt, wRr)
    % Jacobian of the dynamics with respect to x and u
    %% Definitions
    IND_H = 1;                  
    ALPHA = 2;                  
    BETA = 3;
    PHI = 4;        M_PHI = 5;    
    THETA = 5;      M_THETA = 6;  
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;
    fprintf('       jacobian F Covariance\n')

    %% Computations
    F = eye(num_n);
    v = 0;
    r_speedTs = [u_input(I_IND_U); v; u_input(I_IND_W)] * Ts;

    %% Derivatives for the altitude
    % F(IND_H, IND_H) = 1;
    h_alpha = (roty(x(BETA)) * d_rotx(x(ALPHA)) * rotx(pi))' * wRr*r_speedTs;
    h_beta = (d_roty(x(BETA)) * rotx(x(ALPHA)) * rotx(pi))' * wRr*r_speedTs;
    F(IND_H, ALPHA) = h_alpha(3);
    F(IND_H, BETA) = h_beta(3);
    %% ---- CONTRTOLLARE SE GIUSTO IL TERMINE FINALE ---- % RIMETTEREEE
    h_phi = (wRt' * (rotz(psi)*roty(x(THETA))*d_rotx(x(PHI))) * r_speedTs); %*x(IND_P);
    h_theta = (wRt' * (rotz(psi)*d_roty(x(THETA))*rotx(x(PHI))) * r_speedTs); %*x(IND_Q);
    F(IND_H, PHI) = h_phi(3);
    F(IND_H, THETA) = h_theta(3);
end