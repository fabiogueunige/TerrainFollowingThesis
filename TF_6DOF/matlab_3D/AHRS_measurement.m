function [clean_angles, new_angles, wRr] = AHRS_measurement(x_cart, Ts, v_a, angles, input)
    % This function implement the values that are given by the AHRS sensor.
    %% Definition                
    global PHI; global THETA; global PSI;  
    global ROLL; global PITCH; global YAW;

    %% Implementation

    %% LINEARIZED
    phim = angles(PHI) + input(ROLL)*Ts;
    thetam = angles(THETA) + input(PITCH)*Ts;
    psim = 0;
    %% NOT LINEARIZED
    % phim = x_cart(ROLL);
    % thetam = x_cart(PITCH);
    % psim = x_cart(YAW);

    %% Real Rotation
    wRr = rotz(psim)*roty(thetam)*rotx(phim);
    
    
    clean_angles = [phim; thetam; psim];
    %%%%%%%%%%%%%%% NO NOISE %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%% YES NOISE %%%%%%%%%%%%%%%%%%%%%%
    new_angles = clean_angles + v_a;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    printDebug('phi mes new: %.2\n', rad2deg(phim));
    printDebug('theta mes new: %.2f\n', rad2deg(thetam));
end