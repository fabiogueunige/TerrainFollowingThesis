function [out] = derivator (pre_out, inp, pre_inp, TS)
    % parameter of gain that may be changed
    a = 1;
    out = ((2 - a*TS)*pre_out + 2*a*inp - 2*a*pre_inp) * (1/(2 + a*TS));
end