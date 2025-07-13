function [new_angles, wRr] = AHRS_measurement(angles, input, Ts)
    % This function implement the values that are given by the AHRS sensor.
    %% Definition                
    global PHI; global THETA; global PSI;  
    global ROLL; global PITCH; global YAW;

    %% Implementation
    phim = angles(PHI) + input(ROLL)*Ts;
    thetam = angles(THETA) + input(PITCH)*Ts;
    psim = 0;
    % psim = angles(PSI) + input(YAW)*Ts;

    wRr = rotz(psim)*roty(thetam)*rotx(phim);
    new_angles = [phim; thetam; psim];

    printDebug('phi mes new: %.2f | p_gyr: %.2f \n', rad2deg(phim), input(ROLL));
    printDebug('theta mes new: %.2f | q_gyr: %.2f \n', rad2deg(thetam), input(PITCH));
end