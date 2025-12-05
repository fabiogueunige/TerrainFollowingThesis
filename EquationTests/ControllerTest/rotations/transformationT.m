function T = transformationT(rot)
    ROLL = 1; PITCH = 2;
    s_phi = sin(rot(ROLL));
    c_phi = cos(rot(ROLL));
    c_theta = cos(rot(PITCH));
    t_theta = tan(rot(PITCH)); 

    % Check for singularity (gimbal lock at θ = ±π/2)
    if abs(c_theta) < 1e-6
        warning('Kinematic model: Near gimbal lock (theta ≈ ±π/2). T matrix singular!');
        % Use small regularization to prevent division by zero
        c_theta = sign(c_theta) * max(abs(c_theta), 1e-6);
    end

    T = [1,   s_phi * t_theta,   c_phi * t_theta;
        0,        c_phi,              -s_phi;
        0,   s_phi / c_theta,   c_phi / c_theta];
end