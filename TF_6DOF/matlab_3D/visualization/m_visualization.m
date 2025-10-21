% %% New non prende spesso i punti, ma disegna al centro
% function m_visualization(pr, pplane, n, num_s, p_int, wRr, j, planes, plane_contact_idx, sensor_valid)
%     % Visualization of AUV and local terrain
%     % pr: robot position (3x1)
%     % pplane: a reference point on a plane (3x1)
%     % n: plane normal (3x1)
%     % num_s: number of sensors
%     % p_int: intersection points (3 x num_s)
%     % wRr: robot rotation (3x3)
%     % j: iteration index
%     % planes: array of terrain planes (struct with fields point_w, n_w, dir_w)
%     % plane_contact_idx: indices of planes touched by sensors (1 x num_s)
%     % sensor_valid: logical array indicating valid sensors (1 x num_s)
% 
%     % Optional trailing inputs for backward compatibility
%     if nargin < 10, sensor_valid = true(1, num_s); end
%     if nargin < 9, plane_contact_idx = []; end
%     if nargin < 8, planes = []; end
% 
%     figure; hold on; grid on; box on;
%     colors = lines(max(num_s, 6));
% 
%     %% 1) Draw robot as an oriented rectangular box
%     % Assumptions for robot dimensions (meters)
%     Lx = 0.8;   % length along robot x
%     Wy = 0.4;   % width along robot y
%     Hz = 0.2;   % height along robot z (small height)
% 
%     % Local cube vertices centered at origin
%     hx = Lx/2; hy = Wy/2; hz = Hz/2;
%     V_local = [
%         -hx, -hy, -hz;
%          hx, -hy, -hz;
%          hx,  hy, -hz;
%         -hx,  hy, -hz;
%         -hx, -hy,  hz;
%          hx, -hy,  hz;
%          hx,  hy,  hz;
%         -hx,  hy,  hz;
%     ]'; % 3x8
% 
%     % Rotate and translate to world
%     V_world = wRr * V_local + pr;
% 
%     % Faces of the box (indices into vertices)
%     F = [
%         1 2 3 4;  % bottom
%         5 6 7 8;  % top
%         1 2 6 5;  % side x+
%         2 3 7 6;  % side y+
%         3 4 8 7;  % side x-
%         4 1 5 8;  % side y-
%     ];
% 
%     % Draw box
%     patch('Vertices', V_world', 'Faces', F, 'FaceColor', [0.2 0.6 1.0], ...
%           'FaceAlpha', 0.2, 'EdgeColor', [0 0.2 0.6], 'LineWidth', 1.0, 'DisplayName', 'Robot');
% 
%     %% 2) Robot axes
%     t_tmp = 1.0; % axis length
%     x_dir = wRr * [1; 0; 0];
%     y_dir = wRr * [0; 1; 0];
%     z_dir = wRr * [0; 0; 1];
%     p_x_dir = pr + t_tmp * x_dir;
%     p_y_dir = pr + t_tmp * y_dir;
%     p_z_dir = pr + t_tmp * z_dir;
%     plot3([pr(1), p_x_dir(1)], [pr(2), p_x_dir(2)], [pr(3), p_x_dir(3)], 'r-', 'LineWidth', 1.5, 'DisplayName', 'X_r');
%     plot3([pr(1), p_y_dir(1)], [pr(2), p_y_dir(2)], [pr(3), p_y_dir(3)], 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y_r');
%     plot3([pr(1), p_z_dir(1)], [pr(2), p_z_dir(2)], [pr(3), p_z_dir(3)], 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z_r');
% 
%     %% 3) Sensor rays and intersection points
%     for k = 1:num_s
%         % Only draw if sensor has valid contact
%         if sensor_valid(k)
%             plot3([pr(1), p_int(1, k)], [pr(2), p_int(2, k)], [pr(3), p_int(3, k)], ...
%                   'LineWidth', 2, 'Color', colors(k, :), 'DisplayName', sprintf('Ray y%d', k));
%             plot3(p_int(1, k), p_int(2, k), p_int(3, k), 'x', 'Color', colors(k, :), ...
%                     'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', sprintf('Int y%d', k));
%         end
%     end
% 
%     %% 4) Projection of pr onto the reference plane (n, pplane)
%     if numel(n) == 3 && abs(norm(n) - 1) < 1e-3
%         p_proj = pr - dot(n, pr - pplane) * n; % orthogonal projection
%         plot3(pr(1), pr(2), pr(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, 'DisplayName', 'Robot CM');
%         plot3([pr(1), p_proj(1)], [pr(2), p_proj(2)], [pr(3), p_proj(3)], ...
%               'k--', 'LineWidth', 1.5, 'DisplayName', 'Altitude h');
%         plot3(p_proj(1), p_proj(2), p_proj(3), 'kd', 'MarkerSize', 6, 'LineWidth', 1.5, 'DisplayName', 'Proj(h)');
%         % Normal arrow
%         quiver3(p_proj(1), p_proj(2), p_proj(3), n(1), n(2), n(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
%     end
% 
%     %% 5) Draw continuous terrain strip from one plane before first contact to one after last contact
%     if exist('planes','var') && exist('plane_contact_idx','var') && ~isempty(planes)
%         maxN = numel(planes);
%         if maxN > 1
%             half_width = 3.0; % lateral half-width in meters
% 
%             % Find valid contact indices
%             valid_contacts = plane_contact_idx(plane_contact_idx > 0 & plane_contact_idx <= maxN);
%             if ~isempty(valid_contacts)
%                 % Determine range: one plane before min contact to one after max contact
%                 min_contact = min(valid_contacts);
%                 max_contact = max(valid_contacts);
% 
%                 % Extend by one plane before and after (with circular buffer wrapping)
%                 start_idx = min_contact - 1;
%                 if start_idx < 1, start_idx = maxN; end
%                 end_idx = max_contact + 1;
%                 if end_idx > maxN, end_idx = 1; end
% 
%                 % Build ordered sequence from start to end, handling circular buffer
%                 % Count how many planes to draw
%                 if start_idx <= max_contact
%                     % Normal case: no wrap in contact range
%                     num_planes = (max_contact - start_idx) + 2; % +2 for start and end
%                 else
%                     % Contacts wrap around buffer end
%                     num_planes = (maxN - start_idx + 1) + max_contact + 1;
%                 end
% 
%                 % Build plane range using circular indexing
%                 plane_range = zeros(1, num_planes);
%                 for i = 1:num_planes
%                     idx = start_idx + i - 1;
%                     if idx > maxN
%                         idx = idx - maxN; % wrap around
%                     end
%                     plane_range(i) = idx;
%                 end
% 
%                 % Draw continuous quads between consecutive planes (no overlap)
%                 prev_vd = [];
%                 for t = 1:(length(plane_range)-1)
%                     ii = plane_range(t);
%                     jj = plane_range(t+1);
% 
%                     if ~isfield(planes(ii),'point_w') || ~isfield(planes(ii),'n_w') || ~isfield(planes(ii),'dir_w') || ...
%                        ~isfield(planes(jj),'point_w') || ~isfield(planes(jj),'n_w') || ~isfield(planes(jj),'dir_w')
%                         continue;
%                     end
% 
%                     piw = planes(ii).point_w;
%                     pjw = planes(jj).point_w;
% 
%                     % Lateral directions with continuity
%                     vdi = lateral_dir(planes(ii), prev_vd);
%                     if isempty(vdi), continue; end
%                     prev_vd = vdi;
%                     vdj = lateral_dir(planes(jj), prev_vd);
%                     if isempty(vdj), continue; end
%                     if dot(vdj, vdi) < 0, vdj = -vdj; end
% 
%                     % Side points (left/right) for each center point
%                     Li = piw + half_width * vdi;
%                     Ri = piw - half_width * vdi;
%                     Lj = pjw + half_width * vdj;
%                     Rj = pjw - half_width * vdj;
% 
%                     % Quad connecting consecutive planes (no overlap)
%                     patch('XData', [Li(1) Ri(1) Rj(1) Lj(1)], ...
%                           'YData', [Li(2) Ri(2) Rj(2) Lj(2)], ...
%                           'ZData', [Li(3) Ri(3) Rj(3) Lj(3)], ...
%                           'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.35, ...
%                           'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 0.5, ...
%                           'DisplayName', sprintf('Terrain %d-%d', ii, jj));
%                 end
%             end
%         end
%     end
% 
%     xlabel('Asse X'); ylabel('Asse Y'); zlabel('Asse Z');
%     set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
%     title(sprintf('AUV Situation with Sensors (it=%d)', j));
%     axis equal; view(3);
%     hold off;
% end
% 
% % Helper to compute lateral direction with continuity for terrain strips
% function vd = lateral_dir(pl, prev_vd)
%     % lateral_dir returns a unit vector lateral to plane direction, consistent across indices
%     if ~isfield(pl,'n_w') || ~isfield(pl,'dir_w') || ~isfield(pl,'point_w')
%         vd = [];
%         return;
%     end
%     nw = pl.n_w; if norm(nw) ~= 0, nw = nw / norm(nw); else, nw = [0;0;1]; end
%     ud = pl.dir_w; if norm(ud) ~= 0, ud = ud / norm(ud); else, ud = [1;0;0]; end
%     vd = cross(nw, ud);
%     if norm(vd) == 0, vd = [0;1;0]; else, vd = vd / norm(vd); end
%     if ~isempty(prev_vd) && dot(vd, prev_vd) < 0
%         vd = -vd;
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Old migliore per i punti e i piani

function m_visualization(pr, pplane, n, num_s, p_int, wRr, j, planes, plane_contact_idx, sensor_valid)
    % Visualization of AUV and local terrain
    % pr: robot position (3x1)
    % pplane: a reference point on a plane (3x1)
    % n: plane normal (3x1)
    % num_s: number of sensors
    % p_int: intersection points (3 x num_s)
    % wRr: robot rotation (3x3)
    % j: iteration index
    % planes: array of terrain planes (struct with fields point_w, n_w, dir_w)
    % plane_contact_idx: indices of planes touched by sensors (1 x num_s)
    % sensor_valid: logical array indicating valid sensors (1 x num_s)

    % Optional trailing inputs for backward compatibility
    if nargin < 10, sensor_valid = true(1, num_s); end
    if nargin < 9, plane_contact_idx = []; end
    if nargin < 8, planes = []; end

    figure; hold on; grid on; box on;
    colors = lines(max(num_s, 6));

    %% 1) Draw robot as an oriented rectangular box
    % Assumptions for robot dimensions (meters)
    Lx = 0.8;   % length along robot x
    Wy = 0.4;   % width along robot y
    Hz = 0.2;   % height along robot z (small height)

    % Local cube vertices centered at origin
    hx = Lx/2; hy = Wy/2; hz = Hz/2;
    V_local = [
        -hx, -hy, -hz;
         hx, -hy, -hz;
         hx,  hy, -hz;
        -hx,  hy, -hz;
        -hx, -hy,  hz;
         hx, -hy,  hz;
         hx,  hy,  hz;
        -hx,  hy,  hz;
    ]'; % 3x8

    % Rotate and translate to world
    V_world = wRr * V_local + pr;

    % Faces of the box (indices into vertices)
    F = [
        1 2 3 4;  % bottom
        5 6 7 8;  % top
        1 2 6 5;  % side x+
        2 3 7 6;  % side y+
        3 4 8 7;  % side x-
        4 1 5 8;  % side y-
    ];

    % Draw box
    patch('Vertices', V_world', 'Faces', F, 'FaceColor', [0.2 0.6 1.0], ...
          'FaceAlpha', 0.2, 'EdgeColor', [0 0.2 0.6], 'LineWidth', 1.0, 'DisplayName', 'Robot');

    %% 2) Robot axes
    t_tmp = 1.0; % axis length
    x_dir = wRr * [1; 0; 0];
    y_dir = wRr * [0; 1; 0];
    z_dir = wRr * [0; 0; 1];
    p_x_dir = pr + t_tmp * x_dir;
    p_y_dir = pr + t_tmp * y_dir;
    p_z_dir = pr + t_tmp * z_dir;
    plot3([pr(1), p_x_dir(1)], [pr(2), p_x_dir(2)], [pr(3), p_x_dir(3)], 'r-', 'LineWidth', 1.5, 'DisplayName', 'X_r');
    plot3([pr(1), p_y_dir(1)], [pr(2), p_y_dir(2)], [pr(3), p_y_dir(3)], 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y_r');
    plot3([pr(1), p_z_dir(1)], [pr(2), p_z_dir(2)], [pr(3), p_z_dir(3)], 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z_r');

    %% 3) Sensor rays and intersection points
    for k = 1:num_s
        % Only draw if sensor has valid contact
        if sensor_valid(k)
            plot3([pr(1), p_int(1, k)], [pr(2), p_int(2, k)], [pr(3), p_int(3, k)], ...
                  'LineWidth', 2, 'Color', colors(k, :), 'DisplayName', sprintf('Ray y%d', k));
            plot3(p_int(1, k), p_int(2, k), p_int(3, k), 'x', 'Color', colors(k, :), ...
                    'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', sprintf('Int y%d', k));
        end
    end

    %% 4) Projection of pr onto the reference plane (n, pplane)
    if numel(n) == 3 && abs(norm(n) - 1) < 1e-3
        p_proj = pr - dot(n, pr - pplane) * n; % orthogonal projection
        plot3(pr(1), pr(2), pr(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, 'DisplayName', 'Robot CM');
        plot3([pr(1), p_proj(1)], [pr(2), p_proj(2)], [pr(3), p_proj(3)], ...
              'k--', 'LineWidth', 1.5, 'DisplayName', 'Altitude h');
        plot3(p_proj(1), p_proj(2), p_proj(3), 'kd', 'MarkerSize', 6, 'LineWidth', 1.5, 'DisplayName', 'Proj(h)');
        % Normal arrow
        quiver3(p_proj(1), p_proj(2), p_proj(3), n(1), n(2), n(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    end

    %% 5) Draw only the contacted plane patch per sensor (centered at intersection)
    if exist('planes','var') && exist('plane_contact_idx','var') && ~isempty(planes)
        maxN = numel(planes);
        if maxN > 1
            half_u = 2.5; % along plane dir (meters)
            half_v = 2.5; % lateral to plane dir (meters)
            for kk = 1:num_s
                % Only draw patch if sensor has valid contact
                if ~sensor_valid(kk), continue; end
                
                ii = plane_contact_idx(kk);
                if ii < 1 || ii > maxN, continue; end
                if ~isfield(planes(ii),'point_w') || ~isfield(planes(ii),'n_w') || ~isfield(planes(ii),'dir_w')
                    continue;
                end

                % Basis of plane ii
                nw = planes(ii).n_w; if norm(nw) ~= 0, nw = nw / norm(nw); else, nw = [0;0;1]; end
                ud = planes(ii).dir_w; if norm(ud) ~= 0, ud = ud / norm(ud); else, ud = [1;0;0]; end
                vd = cross(nw, ud); if norm(vd) ~= 0, vd = vd / norm(vd); else, vd = [0;1;0]; end

                % Center patch exactly at the intersection point of this sensor
                if size(p_int,2) >= kk && all(isfinite(p_int(:,kk))) && any(p_int(:,kk) ~= 0)
                    center = p_int(:,kk);
                else
                    % Fallback to plane center if intersection missing
                    center = planes(ii).point_w;
                end

                % Four corners of the local patch around the contact
                c1 = center +  half_u*ud +  half_v*vd;
                c2 = center +  half_u*ud -  half_v*vd;
                c3 = center -  half_u*ud -  half_v*vd;
                c4 = center -  half_u*ud +  half_v*vd;

                patch('XData', [c1(1) c2(1) c3(1) c4(1)], ...
                      'YData', [c1(2) c2(2) c3(2) c4(2)], ...
                      'ZData', [c1(3) c2(3) c3(3) c4(3)], ...
                      'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.35, ...
                      'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 0.5, ...
                      'DisplayName', sprintf('Contact plane %d', ii));
            end
        end
    end

    xlabel('Asse X'); ylabel('Asse Y'); zlabel('Asse Z');
    set(gca, 'YDir', 'reverse', 'ZDir', 'reverse');
    title(sprintf('AUV Situation with Sensors (it=%d)', j));
    axis equal; view(3);
    hold off;
end

% Helper to compute lateral direction with continuity for terrain strips
function vd = lateral_dir(pl, prev_vd)
    % lateral_dir returns a unit vector lateral to plane direction, consistent across indices
    if ~isfield(pl,'n_w') || ~isfield(pl,'dir_w') || ~isfield(pl,'point_w')
        vd = [];
        return;
    end
    nw = pl.n_w; if norm(nw) ~= 0, nw = nw / norm(nw); else, nw = [0;0;1]; end
    ud = pl.dir_w; if norm(ud) ~= 0, ud = ud / norm(ud); else, ud = [1;0;0]; end
    vd = cross(nw, ud);
    if norm(vd) == 0, vd = [0;1;0]; else, vd = vd / norm(vd); end
    if ~isempty(prev_vd) && dot(vd, prev_vd) < 0
        vd = -vd;
    end
end
