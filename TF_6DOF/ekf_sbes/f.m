function [x_next, wRt_p] = f(x, u_input, Ts, wRr, wRt)
    %% State Indices
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    global SURGE; global SWAY; global HEAVE;

    printDebug('       f State Prediction\n');
     
    %% Velocity Transformation
    % Transform body velocities to world frame
    w_speed = wRr * u_input(SURGE:HEAVE);
    
    % Transform to terrain-aligned frame
    s_speed = (wRt)' * w_speed;

    %% Altitude Update
    % Integrate vertical velocity in terrain frame
    h_new = x(IND_H) + s_speed(HEAVE)*Ts;

    %% Angle Wrapping
    % Ensure terrain angles remain in [-π, π] range
    alpha_new = wrapToPi(x(ALPHA));
    beta_new = wrapToPi(x(BETA));

    wRt_p = rotz(0)*roty(beta_new)*rotx(alpha_new)*rotx(pi);

    % Debug output for monitoring prediction
    printDebug('Predicted h: %.2f m | a: %.2f | b: %.2f \n', h_new, rad2deg(alpha_new), rad2deg(beta_new));
    
    %% Output Predicted State
    x_next = [h_new, alpha_new, beta_new]';
end