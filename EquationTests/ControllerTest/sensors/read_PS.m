function [z_PS] = read_PS(ps, R_PS)
    dim_PS = 1;
    v_PS = mvnrnd(zeros(dim_PS,1), R_PS)';
    z_PS = ps(:) + v_PS; 
end