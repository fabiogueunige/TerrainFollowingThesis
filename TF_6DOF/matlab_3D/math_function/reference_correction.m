function [a, b] = reference_correction(n_cap, alph, bet, wRr_rob)
    epsilon = 1e-3;
    w_body_z = wRr_rob(:,3);

    %% Computation
    d = dot(n_cap, w_body_z);
    if d > epsilon
        % I should invert the axis
        fprintf('Sto INVERTENDOOOO\n');
        pause(0.005);
        a = wrapToPi(alph + pi);
        b = - bet;
    else
        a = alph;
        b = bet;
    end
end