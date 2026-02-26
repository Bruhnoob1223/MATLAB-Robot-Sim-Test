function [ranges, angles] = sim_lidar(state, world, lidar_params)
%SIM_LIDAR  Simulate a 2-D scanning lidar sensor via ray casting.
%
%   [ranges, angles] = sim_lidar(state, world, lidar_params)
%
%   Casts evenly-spaced rays from the robot position against all obstacles
%   (axis-aligned rectangles and circles) and returns the minimum hit
%   distance for each ray plus configurable Gaussian noise.
%
%   Inputs
%   ------
%   state        : [x, y, theta] robot pose
%   world        : world struct from world_build()
%   lidar_params : struct with fields:
%                    .num_rays  – number of rays
%                    .fov       – total field of view [rad]  (2*pi = 360 deg)
%                    .max_range – maximum range [m]
%                    .noise_std – Gaussian noise standard deviation [m]
%
%   Outputs
%   -------
%   ranges : (num_rays x 1) measured distances [m], clamped to [0, max_range]
%   angles : (num_rays x 1) ray angles RELATIVE to robot heading [rad]

ox      = state(1);
oy      = state(2);
heading = state(3);

n         = lidar_params.num_rays;
fov       = lidar_params.fov;
max_r     = lidar_params.max_range;
noise_std = lidar_params.noise_std;

% Ray angles in robot frame
if n == 1
    angles = 0;
else
    angles = linspace(-fov/2, fov/2, n)';
end

ranges = max_r * ones(n, 1);

for i = 1 : n
    ray_angle = heading + angles(i);  % world-frame direction
    dx = cos(ray_angle);
    dy = sin(ray_angle);

    t_min = max_r;

    % Intersect against all rectangles
    for k = 1 : size(world.rects, 1)
        t = ray_rect_intersect(ox, oy, dx, dy, world.rects(k,:));
        if t > 1e-4 && t < t_min
            t_min = t;
        end
    end

    % Intersect against all circles
    for k = 1 : size(world.circs, 1)
        t = ray_circle_intersect(ox, oy, dx, dy, world.circs(k,:));
        if t > 1e-4 && t < t_min
            t_min = t;
        end
    end

    % Intersect against world boundary walls (4 rectangle-like edges)
    t = ray_bounds_intersect(ox, oy, dx, dy, world.bounds);
    if t > 1e-4 && t < t_min
        t_min = t;
    end

    ranges(i) = t_min;
end

% Add zero-mean Gaussian noise and clamp to valid range
ranges = ranges + noise_std * randn(n, 1);
ranges = max(0, min(max_r, ranges));
end

% =========================================================================
%  Helper – ray vs. axis-aligned rectangle
% =========================================================================
function t = ray_rect_intersect(ox, oy, dx, dy, rect)
%   rect = [cx, cy, width, height]
cx = rect(1); cy = rect(2); hw = rect(3)/2; hh = rect(4)/2;

t = Inf;

% The four edges of the rectangle
edges = [ cx-hw, cy-hh,  cx+hw, cy-hh;   % bottom
          cx+hw, cy-hh,  cx+hw, cy+hh;   % right
          cx+hw, cy+hh,  cx-hw, cy+hh;   % top
          cx-hw, cy+hh,  cx-hw, cy-hh ]; % left

for e = 1 : 4
    ti = ray_segment_intersect(ox, oy, dx, dy, ...
                               edges(e,1), edges(e,2), ...
                               edges(e,3), edges(e,4));
    if ti > 1e-6 && ti < t
        t = ti;
    end
end
end

% =========================================================================
%  Helper – ray vs. circle
% =========================================================================
function t = ray_circle_intersect(ox, oy, dx, dy, circ)
%   circ = [cx, cy, r]
cx = circ(1); cy = circ(2); r = circ(3);

ex = ox - cx;  ey = oy - cy;
b  = 2 * (dx*ex + dy*ey);
c  = ex^2 + ey^2 - r^2;
disc = b^2 - 4*c;

if disc < 0
    t = Inf;
else
    t = (-b - sqrt(disc)) / 2;
    if t < 1e-6
        t = (-b + sqrt(disc)) / 2;
    end
    if t < 1e-6
        t = Inf;
    end
end
end

% =========================================================================
%  Helper – ray vs. world boundary (4 edges)
% =========================================================================
function t = ray_bounds_intersect(ox, oy, dx, dy, bounds)
xmin = bounds(1); ymin = bounds(2); xmax = bounds(3); ymax = bounds(4);
t = Inf;

edges = [ xmin, ymin,  xmax, ymin;   % bottom
          xmax, ymin,  xmax, ymax;   % right
          xmax, ymax,  xmin, ymax;   % top
          xmin, ymax,  xmin, ymin ]; % left

for e = 1 : 4
    ti = ray_segment_intersect(ox, oy, dx, dy, ...
                               edges(e,1), edges(e,2), ...
                               edges(e,3), edges(e,4));
    if ti > 1e-6 && ti < t
        t = ti;
    end
end
end

% =========================================================================
%  Helper – ray vs. line segment  (2-D Cramer's rule)
% =========================================================================
function t = ray_segment_intersect(ox, oy, dx, dy, ax, ay, bx, by)
% Ray  : P = (ox,oy) + t*(dx,dy),    t >= 0
% Seg  : Q = (ax,ay) + s*(ex,ey),    s in [0,1]  where (ex,ey) = (bx-ax, by-ay)
% Solve P = Q by Cramer's rule.
ex = bx - ax;  ey = by - ay;
denom = dx*ey - dy*ex;

t = Inf;
if abs(denom) < 1e-12
    return;   % parallel / collinear
end

t = ((ax-ox)*ey - (ay-oy)*ex) / denom;
s = ((ax-ox)*dy - (ay-oy)*dx) / denom;

if t < 1e-6 || s < 0 || s > 1
    t = Inf;
end
end
