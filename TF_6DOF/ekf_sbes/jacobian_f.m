function F = jacobian_f(x, u_input, Ts, num_n, wRr)
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global SURGE; global SWAY; global HEAVE;
    
    printDebug('       jacobian F Covariance\n')

    %% Initialize Jacobian
    % Start with identity (α and β rows remain unchanged)
    F = zeros(num_n, num_n);

    %% Altitude Partial Derivatives
    % ∂h/∂α: derivative with respect to terrain roll angle
    h_alpha = (roty(x(BETA)) * d_rotx(x(ALPHA)) * rotx(pi))' * wRr*u_input(SURGE:HEAVE);
    
    % ∂h/∂β: derivative with respect to terrain pitch angle
    h_beta = (d_roty(x(BETA)) * rotx(x(ALPHA)) * rotx(pi))' * wRr*u_input(SURGE:HEAVE);
    
    % Populate first row of Jacobian (altitude derivatives)
    F(IND_H, ALPHA) = h_alpha(3);  % Extract z-component
    F(IND_H, BETA) = h_beta(3);    % Extract z-component

    F = eye(num_n) + F * Ts;
end