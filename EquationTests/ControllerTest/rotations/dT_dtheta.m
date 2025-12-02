function B = dT_dtheta(phi, theta)
% ∂(T(η2))/∂θ
% Inputs:
%   phi   - roll angle (rad)
%   theta - pitch angle (rad)
% Output:
%   B     - 3x3 matrix of partial derivatives w.r.t. theta

B = [ 0,              sin(phi)/(cos(theta)^2),           cos(phi)/(cos(theta)^2);
      0,              0,                                 0;
      0,              (sin(phi)*tan(theta))/cos(theta),  (cos(phi)*tan(theta))/cos(theta) ];
end
