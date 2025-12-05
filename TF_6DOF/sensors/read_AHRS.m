function [z_AHRS] = read_AHRS(eta2, nu2, R_AHRS)
    dim_AHRS = 6; % [phi; theta; psi; p; q; r]
    v_AHRS = mvnrnd(zeros(dim_AHRS,1), R_AHRS)';
    z_AHRS(1:3) = eta2(:) + v_AHRS(1:3); 
    z_AHRS(4:6) = nu2(:) + v_AHRS(4:6);
end