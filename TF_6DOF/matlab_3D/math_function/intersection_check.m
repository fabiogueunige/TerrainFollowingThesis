%% INTERSECTION_CHECK - Validate if intersection point lies within segment bounds
%
% Checks whether a computed ray-plane intersection point falls within the
% bounds of a directed line segment. Used to validate sensor measurements
% in terrain following applications.
%
% SYNTAX:
%   [is_intersect] = intersection_check(p_end, p_start, p_int, dir)
%
% INPUTS:
%   p_end   - End point of the segment [3x1]
%   p_start - Start point of the segment [3x1]
%   p_int   - Intersection point to validate [3x1]
%   dir     - Direction vector of the segment [3x1] (should be unit vector)
%
% OUTPUTS:
%   is_intersect - Boolean indicating if p_int is within bounds
%                  true: intersection is valid (between start and end)
%                  false: intersection is outside segment bounds
%
% ALGORITHM:
%   1. Compute vector from start to intersection: v_si = p_int - p_start
%   2. Compute vector from start to end: v_se = p_end - p_start
%   3. Project both onto direction: proj_int = v_si·dir, proj_end = v_se·dir
%   4. Check if 0 ≤ proj_int ≤ proj_end
%
% GEOMETRY:
%   The segment is parameterized as: p(t) = p_start + t*dir, t ∈ [0, L]
%   where L = ||p_end - p_start||
%
%   Intersection valid if: t_int ∈ [0, t_end]
%
% USE CASE:
%   In SBES measurements, validates that the computed terrain intersection
%   falls within the sensor's maximum range.
%
% See also: SBES_measurament, plane_computation

function [is_intersect] = intersection_check(p_end, p_start, p_int, dir)
    %% Vector Computations
    % Vector from p_start to intersection point
    v_start_int = p_int - p_start;
    
    % Vector from p_start to end point
    v_start_end = p_end - p_start;
    
    %% Projection onto Direction
    % Project intersection vector onto direction
    proj_int = dot(v_start_int, dir);
    
    % Project segment vector onto direction
    proj_end = dot(v_start_end, dir);
    
    %% Bounds Check
    % Intersection valid if projection lies between 0 and proj_end
    % This ensures: p_start ≤ p_int ≤ p_end along direction
    if proj_int >= 0 && proj_int <= proj_end
        is_intersect = true;
    else
        is_intersect = false;
    end
end
    
