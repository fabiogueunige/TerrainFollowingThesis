function [x_next, wRt_p] = f(x, u_input, Ts, wRr)
    %This function calculates the next state of the system based on the current state,
        % control input, and sampling time.    
    %% Definition
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global SURGE; global SWAY; global HEAVE;

    printDebug('       f State Prediction\n');
    
    %% Update of the values
    % change of the slope of the terrain
    alpha_new = wrapToPi(x(ALPHA));
    beta_new = wrapToPi(x(BETA));
    
    %% Rotation Computation
    % Terrain
    wRt_p = rotz(0)*roty(beta_new)*rotx(alpha_new)*rotx(pi);
    
    % speed in terrain frame
    w_speed = wRr * u_input(SURGE:HEAVE);
    s_speed = (wRt_p)' * w_speed;

    % The new altitude (h)
    h_new = x(IND_H) + s_speed(HEAVE)*Ts;

    % Display the predicted state for debugging or monitoring purposes.
    printDebug('Predicted h: %.2f m | a: %.2f | b: %.2f ', h_new, rad2deg(alpha_new), rad2deg(beta_new));
    
    %% Send info
    % ---- x0 = [h, alpha, beta, phi, theta] ---- %
    x_next = [h_new, alpha_new, beta_new]';
end