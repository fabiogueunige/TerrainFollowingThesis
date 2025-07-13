function F = jacobian_f(x, u_input, Ts, num_n, wRr)
    % Jacobian of the dynamics with respect to x and u
    %% Definition
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    % angles               
    global SURGE; global SWAY; global HEAVE;
    printDebug('       jacobian F Covariance\n')

    %% Computations
    F = eye(num_n);
    r_speedTs = u_input(SURGE:HEAVE) * Ts;

    %% Derivatives for the altitude
    % F(IND_H, IND_H) = 1;
    h_alpha = (roty(x(BETA)) * d_rotx(x(ALPHA)) * rotx(pi))' * wRr*r_speedTs;
    h_beta = (d_roty(x(BETA)) * rotx(x(ALPHA)) * rotx(pi))' * wRr*r_speedTs;
    F(IND_H, ALPHA) = h_alpha(3);
    F(IND_H, BETA) = h_beta(3);
end