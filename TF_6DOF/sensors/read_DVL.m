function [z_DVL] = read_DVL(spd, R_DVL)
    dim_DVL = 3;
    v_DVL = mvnrnd(zeros(dim_DVL,1), R_DVL)';
    z_DVL = spd(:) + v_DVL; 
end