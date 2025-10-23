function [planes, current_idx] = terrain_init(p_init, p_robot, max_planes, step_length, n0, ...
                                    angle_range, rate_of_change)
    % Initialization of terrain planes with circular buffer

    %% Struct initial definition - First plane
    pl.alpha = 0; 
    pl.beta = 0; 
    pl.point_w = p_init;
    pl.dir_w = [1; 1; 0];

    %% n in inertial frame
    wRs = (rotz(0)*roty(pl.beta)*rotx(pl.alpha))*rotx(pi);
    pl.n_w = wRs*n0; % in world frame    
    if (norm(pl.n_w) ~= 1)
        pl.n_w = vector_normalization(pl.n_w);
        printDebug('n: [%.4f; %.4f; %.4f]\n', pl.n_w(1), pl.n_w(2), pl.n_w(3));
    end

    %% Pre-allocate array with first plane
    planes(1:max_planes) = pl;
    current_idx = 1;

    %% Generate initial planes using terrain_generator
    % Initial velocity direction for generation
    initial_vel = [1; 1; 0]; % in world frame
    initial_vel = vector_normalization(initial_vel);
    
    % Single call to terrain_generator - it will generate all needed planes
    % with the while loop until 20m coverage is reached
    [planes, current_idx] = terrain_generator(planes, p_robot, initial_vel, current_idx, step_length, ...
                max_planes, 1, angle_range, rate_of_change);
    
    printDebug('Terrain initialized with %d planes\n', current_idx);
end