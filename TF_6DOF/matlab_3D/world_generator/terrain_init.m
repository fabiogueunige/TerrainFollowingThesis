function [planes] = terrain_init(p_init, ext_mem, nIte, st_length)
    % Initialization of terrain planes
    n0 = [0; 0; 1]; 

    %% Struct initial definition
    pl.alpha = 0; 
    pl.beta = 0; 
    pl.point = p_init;
    pl.dir_w = [1, 1, 0]';

    %% n in inertial frame
    wRs = (rotz(0)*roty(pl.beta)*rotx(pl.alpha))*rotx(pi);
    pl.n_w = wRs*n0; % in world frame    
    if (norm(pl.n_w) ~= 1)
        pl.n_w = vector_normalization(pl.n_w);
        printDebug('n: [%.4f; %.4f; %.4f]\n', pl.n_w(1), pl.n_w(2), pl.n_w(3));
    end
    pl.point_w = wRs*pl.point; % in world frame


    dim = nIte + ext_mem * 2; % Extra space for prev and future planes
    planes(1:dim) = pl;

    %% Generate the first ext_mem planes before the start point
    for ii = 2:ext_mem
        planes(ii) = terrain_generator(planes(ii-1), [1;1;0], ii, st_length);
    end
end