function [x_next, wRt, wRr] = f(x, u_input, Ts, psi)
    %This function calculates the next state of the system based on the current state,
        % control input, and sampling time.    
    %% Definitions
    IND_H = 1;                  
    ALPHA = 2;                  
    BETA = 3;
    PHI = 4;        M_PHI = 5;    
    THETA = 5;      M_THETA = 6;  
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;  
    fprintf('       f State Prediction\n');
    %% Rotation Computation
    % Terrain
    wRt = (rotz(0)*roty(x(BETA))*rotx(x(ALPHA)))*(rotx(pi)');

    % robot
    wRr = rotz(psi)*roty(x(THETA))*rotx(x(PHI));
    
    % speed in terrain frame
    v = 0; % No attuation in sway for now
    w_speed = wRr * [u_input(I_IND_U); v; u_input(I_IND_W)];
    s_speed = (wRt)' * w_speed;

    %% Update of the values
    % h_new represents the change in altitude (h)
    h_new = x(IND_H) + s_speed(3)*Ts;

    % change of the slope of the terrain
    alpha_new = wrapToPi(x(ALPHA));
    beta_new = wrapToPi(x(BETA));
    % Change of the angles of the robot
    phi_new = wrapToPi(x(PHI) + u_input(I_IND_P)*Ts);
    theta_new = wrapToPi(x(THETA) + u_input(I_IND_Q)*Ts);
    
    % Display the predicted state for debugging or monitoring purposes.
    fprintf('Predicted h: %.2f m | a: %.2f | b: %.2f ', h_new, rad2deg(alpha_new), rad2deg(beta_new));
    fprintf('| phi: %.2f | theta: %.2f\n', rad2deg(phi_new), rad2deg(theta_new));
    %% Send info
    % ---- x0 = [h, alpha, beta, phi, theta] ---- %
    x_next = [h_new, alpha_new, beta_new, phi_new theta_new]';
end