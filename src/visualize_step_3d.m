function handles = visualize_step_3d(state, world, trail, waypoints, goal, step, handles)
%VISUALIZE_STEP_3D  Update the real-time 3-D robot simulation plot.
%
%   handles = visualize_step_3d(state, world, trail, waypoints, goal, step, handles)
%
%   Renders the wheeled robot in 3-D: a blue rectangular body sitting on
%   two dark wheel discs.  Obstacles are drawn as grey 3-D boxes / cylinders.
%   On the first call this function builds the entire figure including all
%   static geometry.  On subsequent calls only the dynamic graphics objects
%   (body, wheels, trail, heading arrow) are updated for efficiency.
%
%   Inputs
%   ------
%   state     : [x, y, theta] current robot pose
%   world     : world struct from world_build()
%   trail     : Mx2 history of robot positions [m]
%   waypoints : Px2 waypoint list (empty [] if not applicable)
%   goal      : [gx, gy] goal position
%   step      : simulation step counter (used for the title)
%   handles   : handle struct from a previous call (or [] on first call)
%
%   Output
%   ------
%   handles : updated handle struct

% ---- First call: build figure -------------------------------------------
if isempty(handles) || ~isfield(handles, 'fig') || ~ishandle(handles.fig)
    handles = build_figure_3d(world, waypoints, goal);
end

% ---- Update robot body --------------------------------------------------
[bv, bf] = robot_box_verts(state);
set(handles.h_body, 'Vertices', bv, 'Faces', bf);

% ---- Update wheels ------------------------------------------------------
[lv, lf] = wheel_disc_verts(state, +1);   % left  (+y in robot frame)
set(handles.h_lwheel, 'Vertices', lv, 'Faces', lf);

[rv, rf] = wheel_disc_verts(state, -1);   % right (-y in robot frame)
set(handles.h_rwheel, 'Vertices', rv, 'Faces', rf);

% ---- Update trail -------------------------------------------------------
set(handles.h_trail, ...
    'XData', trail(:,1), ...
    'YData', trail(:,2), ...
    'ZData', zeros(size(trail, 1), 1));

% ---- Update heading arrow -----------------------------------------------
arrow_len = 0.45;
arrow_z   = 0.30;   % slightly above top of body
set(handles.h_arrow, ...
    'XData', [state(1), state(1) + arrow_len * cos(state(3))], ...
    'YData', [state(2), state(2) + arrow_len * sin(state(3))], ...
    'ZData', [arrow_z, arrow_z]);

% ---- Update title -------------------------------------------------------
set(handles.h_title, 'String', ...
    sprintf('Step %d   (%.2f, %.2f)   \\theta=%.1f deg', ...
            step, state(1), state(2), rad2deg(state(3))));

drawnow limitrate;
end

% =========================================================================
%  Local helper: build the static figure and dynamic placeholder objects
% =========================================================================
function handles = build_figure_3d(world, waypoints, goal)

fig = figure('Name', '3-D Robot Simulation', 'Color', 'w', 'NumberTitle', 'off');
ax  = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
view(ax, 45, 30);   % isometric-ish 3-D view

b = world.bounds;
axis(ax, [b(1), b(3), b(2), b(4), -0.05, 1.5]);
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
zlabel(ax, 'z [m]');

% Lighting for depth cues
camlight(ax, 'headlight');
lighting(ax, 'gouraud');

% ---- Static: floor ------------------------------------------------------
patch(ax, [b(1), b(3), b(3), b(1)], [b(2), b(2), b(4), b(4)], [0, 0, 0, 0], ...
      'FaceColor', [0.82 0.85 0.82], 'EdgeColor', [0.65 0.65 0.65]);

% ---- Static: rectangle obstacles (grey 3-D boxes) -----------------------
obs_h = 0.50;
for k = 1 : size(world.rects, 1)
    draw_box_3d(ax, world.rects(k,:), obs_h);
end

% ---- Static: circle obstacles (grey cylinders) --------------------------
for k = 1 : size(world.circs, 1)
    draw_cyl_3d(ax, world.circs(k,:), obs_h);
end

% ---- Static: waypoints --------------------------------------------------
if ~isempty(waypoints)
    plot3(ax, waypoints(:,1), waypoints(:,2), zeros(size(waypoints,1), 1), ...
          'b--o', 'MarkerSize', 5, 'LineWidth', 1.2, 'DisplayName', 'Waypoints');
end

% ---- Static: goal marker ------------------------------------------------
plot3(ax, goal(1), goal(2), 0, 'g*', ...
      'MarkerSize', 14, 'LineWidth', 2, 'DisplayName', 'Goal');

% ---- Dynamic: trail -----------------------------------------------------
handles.h_trail = plot3(ax, NaN, NaN, NaN, 'b-', ...
    'LineWidth', 1.5, 'DisplayName', 'Trail');

% ---- Dynamic: robot body (initialised at origin) -------------------------
[bv0, bf0] = robot_box_verts([0, 0, 0]);
handles.h_body = patch(ax, 'Vertices', bv0, 'Faces', bf0, ...
    'FaceColor', [0.20 0.40 0.80], 'EdgeColor', 'k', 'LineWidth', 0.5, ...
    'DisplayName', 'Robot');

% ---- Dynamic: wheels ----------------------------------------------------
[lv0, lf0] = wheel_disc_verts([0, 0, 0], +1);
handles.h_lwheel = patch(ax, 'Vertices', lv0, 'Faces', lf0, ...
    'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'none');

[rv0, rf0] = wheel_disc_verts([0, 0, 0], -1);
handles.h_rwheel = patch(ax, 'Vertices', rv0, 'Faces', rf0, ...
    'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'none');

% ---- Dynamic: heading arrow ---------------------------------------------
handles.h_arrow = plot3(ax, NaN, NaN, NaN, 'r-', 'LineWidth', 2.5);

handles.h_title = title(ax, 'Initializing...');
legend(ax, 'show', 'Location', 'northeast');

handles.fig = fig;
handles.ax  = ax;
end

% =========================================================================
%  Local helper: draw a rectangle obstacle as a 3-D box
% =========================================================================
function draw_box_3d(ax, rect, h)
cx = rect(1);  cy = rect(2);
w  = rect(3);  d  = rect(4);
x1 = cx - w/2;  x2 = cx + w/2;
y1 = cy - d/2;  y2 = cy + d/2;
col = [0.50 0.50 0.50];
ec  = 'k';
% Four walls
patch(ax, [x1 x2 x2 x1], [y1 y1 y1 y1], [0 0 h h], ...
      'FaceColor', col, 'EdgeColor', ec, 'LineWidth', 0.5);
patch(ax, [x2 x2 x2 x2], [y1 y2 y2 y1], [0 0 h h], ...
      'FaceColor', col, 'EdgeColor', ec, 'LineWidth', 0.5);
patch(ax, [x2 x1 x1 x2], [y2 y2 y2 y2], [0 0 h h], ...
      'FaceColor', col, 'EdgeColor', ec, 'LineWidth', 0.5);
patch(ax, [x1 x1 x1 x1], [y2 y1 y1 y2], [0 0 h h], ...
      'FaceColor', col, 'EdgeColor', ec, 'LineWidth', 0.5);
% Top face
patch(ax, [x1 x2 x2 x1], [y1 y1 y2 y2], [h h h h], ...
      'FaceColor', col, 'EdgeColor', ec, 'LineWidth', 0.5);
end

% =========================================================================
%  Local helper: draw a circular obstacle as a 3-D cylinder
% =========================================================================
function draw_cyl_3d(ax, circ, h)
cx = circ(1);  cy = circ(2);  r = circ(3);
n  = 24;
a  = linspace(0, 2*pi, n + 1);  a(end) = [];
xc = cx + r * cos(a);
yc = cy + r * sin(a);
col = [0.50 0.50 0.50];
% Bottom disc
patch(ax, xc, yc, zeros(1, n), 'FaceColor', col, 'EdgeColor', 'none');
% Top disc
patch(ax, xc, yc, repmat(h, 1, n), 'FaceColor', col, 'EdgeColor', 'none');
% Cylindrical side via surf
xs = [xc; xc];
ys = [yc; yc];
zs = [zeros(1, n); repmat(h, 1, n)];
surf(ax, xs, ys, zs, 'FaceColor', col, 'EdgeColor', 'none');
end

% =========================================================================
%  Local helper: robot body box vertices and faces
%
%  Robot local frame: x = forward, y = left, z = up
%  Body rests on top of the wheel axle (z = rw = 0.10 m).
% =========================================================================
function [v, f] = robot_box_verts(state)
x  = state(1);  y  = state(2);  th = state(3);
dx = 0.20;      % fore-aft half-length [m]
dy = 0.175;     % lateral half-width   [m]
z0 = 0.10;      % body bottom (= wheel radius) [m]
z1 = 0.28;      % body top [m]

% Vertices in robot local frame
vl = [-dx, -dy, z0;   % 1: rear-right-bottom
       dx, -dy, z0;   % 2: front-right-bottom
       dx,  dy, z0;   % 3: front-left-bottom
      -dx,  dy, z0;   % 4: rear-left-bottom
      -dx, -dy, z1;   % 5: rear-right-top
       dx, -dy, z1;   % 6: front-right-top
       dx,  dy, z1;   % 7: front-left-top
      -dx,  dy, z1];  % 8: rear-left-top

% Rotate x-y columns by theta, keep z unchanged
c  = cos(th);  s = sin(th);
xy = [c * vl(:,1) - s * vl(:,2), ...
      s * vl(:,1) + c * vl(:,2)];
v  = [xy(:,1) + x, xy(:,2) + y, vl(:,3)];

% Six quad faces (each row = one face, 4 vertex indices)
f = [1 2 3 4;   % bottom
     5 6 7 8;   % top
     2 3 7 6;   % front  (+x side)
     4 1 5 8;   % rear   (-x side)
     1 2 6 5;   % right  (-y side)
     3 4 8 7];  % left   (+y side)
end

% =========================================================================
%  Local helper: wheel disc vertices and face
%
%  Each wheel is a flat n-gon in the plane perpendicular to the robot's
%  lateral (y) axis.  The wheel centre sits at height rw (wheel radius)
%  so the bottom of the wheel touches z = 0.
% =========================================================================
function [v, f] = wheel_disc_verts(state, side)
% side: +1 = left (+y in robot frame), -1 = right (-y in robot frame)
n      = 20;
rw     = 0.10;    % wheel radius [m]
y_trk  = 0.215;   % half-track (distance from robot centre to wheel face) [m]

a  = linspace(0, 2*pi, n + 1);  a(end) = [];
xl = rw * cos(a)';                       % local x
yl = repmat(side * y_trk, n, 1);         % local y (constant for a disc)
zl = rw + rw * sin(a)';                  % local z (centre at height rw)

x0 = state(1);  y0 = state(2);  th = state(3);
c  = cos(th);   s  = sin(th);
xw = c * xl - s * yl + x0;
yw = s * xl + c * yl + y0;

v = [xw, yw, zl];
f = 1:n;   % single polygon face
end
