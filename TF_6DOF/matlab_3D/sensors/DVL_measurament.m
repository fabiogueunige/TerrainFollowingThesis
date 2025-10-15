function [w_speed, p_rob] = DVL_measurament(pr_old, input, wRr, Ts)
    %% Definition
    global SURGE; global SWAY; global HEAVE;
    printDebug('       DVL Measurament:\n');

    % Okay just because no noise in simulation
    w_speed = wRr * input(SURGE:HEAVE);
    p_rob = pr_old + w_speed*Ts;

    %% Add noise in velocity estimation
    % TODO

    printDebug('Punto x: %.2f | y: %.2f | z: %.2f\n', p_rob(1), p_rob(2), p_rob(3));
end