function handles = visualize_step(state, world, ranges, ray_angles, ...
                                   trail, waypoints, goal, step, handles)
%VISUALIZE_STEP  Update the real-time 2-D robot simulation plot.
%
%   handles = visualize_step(state, world, ranges, ray_angles, ...
%                             trail, waypoints, goal, step, handles)
%
%   On the first call (handles is empty or the figure has been closed) this
%   function builds the entire figure including static obstacle patches.
%   On subsequent calls it only updates the handles of dynamic graphic
%   objects (trail, robot body, heading arrow, lidar rays) for efficiency.
%
%   Inputs
%   ------
%   state      : [x, y, theta] current robot pose
%   world      : world struct from world_build()
%   ranges     : Nx1 lidar ranges (empty [] to disable lidar overlay)
%   ray_angles : Nx1 ray angles relative to robot heading [rad]
%   trail      : Mx2 history of robot positions [m]
%   waypoints  : Px2 waypoint list (empty [] if Mode B)
%   goal       : [gx, gy] goal position
%   step       : simulation step counter (used for the title)
%   handles    : handle struct from a previous call (or [] on first call)
%
%   Output
%   ------
%   handles : updated handle struct

% ---- First call: build figure -------------------------------------------
if isempty(handles) || ~isfield(handles, 'fig') || ~ishandle(handles.fig)
    handles = build_figure(world, waypoints, goal);
end

% ---- Update dynamic elements --------------------------------------------

% Trail
set(handles.h_trail, 'XData', trail(:,1), 'YData', trail(:,2));

% Robot body (filled circle approximation)
r_body = 0.20;
th_circ = linspace(0, 2*pi, 30);
set(handles.h_robot, ...
    'XData', state(1) + r_body * cos(th_circ), ...
    'YData', state(2) + r_body * sin(th_circ));

% Heading arrow
arrow_len = 0.35;
set(handles.h_arrow, ...
    'XData', [state(1), state(1) + arrow_len * cos(state(3))], ...
    'YData', [state(2), state(2) + arrow_len * sin(state(3))]);

% Lidar rays (NaN-separated polyline for efficiency)
if ~isempty(ranges) && isfield(handles, 'h_lidar')
    n   = numel(ranges);
    lx  = nan(3*n, 1);
    ly  = nan(3*n, 1);
    for i = 1 : n
        abs_ang    = state(3) + ray_angles(i);
        lx(3*i-2)  = state(1);
        ly(3*i-2)  = state(2);
        lx(3*i-1)  = state(1) + ranges(i) * cos(abs_ang);
        ly(3*i-1)  = state(2) + ranges(i) * sin(abs_ang);
        % lx(3*i) stays NaN  → segment break
    end
    set(handles.h_lidar, 'XData', lx, 'YData', ly);
end

% Title
set(handles.h_title, 'String', ...
    sprintf('Step %d   (%.2f, %.2f)   \\theta=%.1f deg', ...
            step, state(1), state(2), rad2deg(state(3))));

drawnow limitrate;
end

% =========================================================================
%  First-call builder: construct static figure + dynamic handle placeholders
% =========================================================================
function handles = build_figure(world, waypoints, goal)

fig = figure('Name', 'Robot Simulation', 'Color', 'w', 'NumberTitle', 'off');
ax  = axes('Parent', fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');

b = world.bounds;
axis(ax, [b(1), b(3), b(2), b(4)]);
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');

% ---- Static: world boundary --------------------------------------------
plot(ax, [b(1), b(3), b(3), b(1), b(1)], ...
         [b(2), b(2), b(4), b(4), b(2)], ...
    'k-', 'LineWidth', 2);

% ---- Static: rectangle obstacles ----------------------------------------
for k = 1 : size(world.rects, 1)
    cx = world.rects(k,1);  cy = world.rects(k,2);
    w  = world.rects(k,3);  h  = world.rects(k,4);
    rectangle('Parent', ax, ...
              'Position', [cx - w/2, cy - h/2, w, h], ...
              'FaceColor', [0.55, 0.55, 0.55], ...
              'EdgeColor', 'k', 'LineWidth', 1);
end

% ---- Static: circle obstacles -------------------------------------------
for k = 1 : size(world.circs, 1)
    cx = world.circs(k,1);  cy = world.circs(k,2);  r = world.circs(k,3);
    th = linspace(0, 2*pi, 60);
    fill(ax, cx + r*cos(th), cy + r*sin(th), ...
         [0.55, 0.55, 0.55], 'EdgeColor', 'k', 'LineWidth', 1);
end

% ---- Static: waypoints -------------------------------------------------
if ~isempty(waypoints)
    plot(ax, waypoints(:,1), waypoints(:,2), 'b--o', ...
         'MarkerSize', 6, 'LineWidth', 1.2, 'DisplayName', 'Waypoints');
end

% ---- Static: goal marker -----------------------------------------------
plot(ax, goal(1), goal(2), 'g*', ...
     'MarkerSize', 14, 'LineWidth', 2, 'DisplayName', 'Goal');

% ---- Dynamic placeholders ----------------------------------------------
handles.h_lidar = plot(ax, NaN, NaN, '-', ...
    'Color', [1, 0.55, 0, 0.35], 'LineWidth', 0.7, 'DisplayName', 'Lidar');
handles.h_trail = plot(ax, NaN, NaN, 'b-', ...
    'LineWidth', 1.2, 'DisplayName', 'Trail');
handles.h_robot = plot(ax, NaN, NaN, 'r-', ...
    'LineWidth', 2,   'DisplayName', 'Robot');
handles.h_arrow = plot(ax, NaN, NaN, 'r-', ...
    'LineWidth', 2.5);

handles.h_title = title(ax, 'Initializing...');

legend(ax, 'show', 'Location', 'northeast');

handles.fig = fig;
handles.ax  = ax;
end
