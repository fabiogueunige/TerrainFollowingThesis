function x_next = f(x, u, Ts)
    % This function calculates the next state of the system based on the current state,
    % control input, and sampling time.

    % h_dot represents the change in altitude (h)
    h_dot = x(1) - u(1)*sin((x(2)-x(3)))*Ts - u(2)*cos((x(2)-x(3)))*Ts;

    % beta_dot represents the change iof the slope of the terrain
    beta_dot = wrapToPi(x(2));
    
    theta_dot = wrapToPi(x(3)) +  wrapToPi(x(4)*Ts);

    q_dot = x(4);
    % Display the predicted state for debugging or monitoring purposes.
    fprintf('Predicted (f) h: %.2f m | b: %.2f | th: %.2f | q: %.2f\n', h_dot, rad2deg(beta_dot), rad2deg(theta_dot), q_dot);

    x_next = [h_dot, beta_dot, theta_dot, q_dot]';
end