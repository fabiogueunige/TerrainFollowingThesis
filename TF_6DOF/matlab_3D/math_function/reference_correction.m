function [a, b] = reference_correction(n_cap, alph, bet)
    if n_cap(3) > 0
        fprintf('Inverto Inverto per bene\n');
        new_n_cap = -n_cap;
        new_n = vector_normalization(new_n_cap);
        bet_part = sqrt(new_n(2)^2 + new_n(3)^2);

        beta_corr  = atan2(-new_n(1), bet_part);
        alpha_corr = atan2(new_n(2), new_n(3));

        % wrap to [-pi,pi]
        b = mod(beta_corr + pi, 2*pi) - pi;
        a = mod(alpha_corr+ pi, 2*pi) - pi;
    else
        a = alph;
        b = bet;
    end

    % %% Computation
    % d = dot(n_cap, w_body_z);
    % if d > epsilon
    %     % I should invert the axis
    %     fprintf('Sto INVERTENDOOOO\n');
    %     % pause(0.005);
    %     new_n_cap = -n_cap;
    %     new_n = vector_normalization(new_n_cap);
    % 
    %     bet_part = sqrt(new_n(2)^2 + new_n(3)^2);
    % 
    %     beta_corr  = atan2(-new_n(1), bet_part);
    %     alpha_corr = atan2(new_n(2), new_n(3));
    % 
    %     % wrap to [-pi,pi]
    %     b = mod(beta_corr + pi, 2*pi) - pi;
    %     a = mod(alpha_corr+ pi, 2*pi) - pi;
    % else
    %     a = alph;
    %     b = bet;
    % end
end

% Help for pi/2
% if bet_part < 1e-6
%   % Sono in quasi-gimbal-lock: decido beta = sign(-n(1))*pi/2
%   beta_corr  = sign(-n(1))*pi/2;
% else
%     beta_corr  = atan2(-new_n(1), bet_part);
% end