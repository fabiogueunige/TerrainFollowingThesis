function [z_AHRS] = read_AHRS(eta2, R_AHRS)
    dim_AHRS = 6;
    v_AHRS = mvnrnd(zeros(dim_AHRS,1), R_AHRS)';
    z_AHRS = eta2(:) + v_AHRS; 
end