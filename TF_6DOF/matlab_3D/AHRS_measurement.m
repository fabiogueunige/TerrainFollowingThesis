function [clean_angles, new_angles, wRr] = AHRS_measurement(x_cart, angles, input, Ts, v_a)
    % This function implement the values that are given by the AHRS sensor.
    %% Definition                
    global PHI; global THETA; global PSI;  
    global ROLL; global PITCH; global YAW;

    %% LINEARIZED
    phim = angles(PHI) + input(ROLL)*Ts;
    thetam = angles(THETA) + input(PITCH)*Ts;
    psim = angles(PSI) + input(YAW)*Ts;

    %% NOT LINEARIZED
    % VA TOTALMENTE CAMBIATO 
    % phim = x_cart(ROLL);
    % thetam = x_cart(PITCH);
    % psim = x_cart(YAW);

    wRr = rotz(psim)*roty(thetam)*rotx(phim);

    clean_angles = [phim; thetam; psim];
    %%%%%%%%%%%%%%% NO NOISE %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%
    new_angles = clean_angles + v_a;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    printDebug('phi mes new: %.2f | p_gyr: %.2f \n', rad2deg(phim), input(ROLL));
    printDebug('theta mes new: %.2f | q_gyr: %.2f \n', rad2deg(thetam), input(PITCH));
    printDebug('yaw mes new: %.2f | q_gyr: %.2f \n', rad2deg(psim), input(YAW));
end