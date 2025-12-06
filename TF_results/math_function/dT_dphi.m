function A = dT_dphi(phi, theta)
% ∂(T(η2))/∂φ
% Inputs:
%   phi   - roll angle (rad)
%   theta - pitch angle (rad)
% Output:
%   A     - 3x3 matrix of partial derivatives w.r.t. phi

A = [ 0,              cos(phi)*tan(theta),   -sin(phi)*tan(theta);
      0,             -sin(phi),              -cos(phi);
      0,              cos(phi)/cos(theta),   -sin(phi)/cos(theta) ];
end
