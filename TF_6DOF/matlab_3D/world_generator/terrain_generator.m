function [pl] = terrain_generator(old_pl, vel_w, ii, step_length)
    %% Initialization of terrain planes
    delta_limit = pi/3;
    angle_range = [-pi/4, pi/4]; % limitation in changes
    n0 = [0; 0; 1]; 
    pl = old_pl;
    % Se cambi rate of change -> cambia condizione SBES !!!
    rate_of_change = 10;

    %% Computation of terrain planes
    valid = false;
    if mod(ii, rate_of_change) == 0
        while ~valid
            pl.alpha = (angle_range(2)-angle_range(1))*rand + angle_range(1);
            pl.beta = (angle_range(2)-angle_range(1))*rand + angle_range(1);
            if abs(pl.alpha - old_pl.alpha) <= delta_limit && abs(pl.beta - old_pl.beta) <= delta_limit
                valid = true;
            end
        end   
    else
        % Se reintroddotte -> togliere controllo piano ogni tot.
        % small angle changes !!! NON ADESSO !!!
        % pl.alpha = old_pl.alpha + 0.1 * sin(0.1 * ii) + 0.1 * randn;
        % pl.beta = old_pl.beta + 0.1 * sin(0.1 * ii) + 0.1 * randn;
    end

    % Definition in inertial frame
    wRs = (rotz(0)*roty(pl.beta)*rotx(pl.alpha))*rotx(pi);
    pl.n_w = wRs * n0; % in world frame    
    if (norm(pl.n_w) ~= 1)
        pl.n_w = vector_normalization(pl.n_w);
        printDebug('n: [%.4f; %.4f; %.4f]\n', pl.n_w(1), pl.n_w(2), pl.n_w(3));
    end

    %% Plane point definition
    % direzione avanzamento !!! vel_dir deve essere del robot nel frame del terreno !!!
    dir_w = vel_w - (vel_w' * pl.n_w) * pl.n_w; % componente della velocità sul piano
    dir_w = vector_normalization(dir_w);
    pl.point_w = old_pl.point_w + step_length * dir_w; % punto sul piano nel frame del mondo
    % point in terrain frame
    pl.point = wRs' * pl.point_w;
end 