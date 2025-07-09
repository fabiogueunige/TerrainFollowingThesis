function [new_angles, wRr] = AHRS_measurement(angles, input, Ts)
    % This function implement the values that are given by the AHRS sensor.
    %% Definition
    % state
    IND_H = 1;      ALPHA = 2;      BETA = 3;  
    % angles               
    PHI = 1;        THETA = 2;      PSI = 3;  
    % input
    I_IND_U = 1;    I_IND_W = 2;    I_IND_P = 3;    I_IND_Q = 4;

    %% Implementation
    phim = angles(PHI) + input(I_IND_P)*Ts;
    thetam = angles(THETA) + input(I_IND_Q)*Ts;
    psim = angles(PSI) + 0;
    % psim = angles(PSI) + input(I_IND_R)*Ts;

    wRr = rotz(psim)*roty(thetam)*rotx(phim);
    new_angles = [phim; thetam; psim];

    printDebug('phi mes new: %.2f | p_gyr: %.2f \n', rad2deg(phim), input(I_IND_P));
    printDebug('theta mes new: %.2f | q_gyr: %.2f \n', rad2deg(thetam), input(I_IND_Q));
end