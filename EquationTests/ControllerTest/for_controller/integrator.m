function [out] = integrator (pre_out, inp, pre_inp, TS)
    % param for the gain
    a = 1;
    out = pre_out + ((inp + pre_inp) * (TS/2));
end