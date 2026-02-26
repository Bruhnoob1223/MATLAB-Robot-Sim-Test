function [world, waypoints, start_pose, goal] = scenario_circle()
%SCENARIO_CIRCLE  Open arena where the robot drives one full circle.
%
%   An obstacle-free 22 x 22 m space.  24 equally-spaced waypoints lie on a
%   circle of radius 5 m centred at the origin.  The robot starts at the
%   rightmost point (5, 0) heading north and ends back at the same point
%   after one complete loop.
%
%   This scenario is designed for use with Mode C (3-D visualisation).
%
%   Returns
%   -------
%   world      : world struct (see world_build)
%   waypoints  : 24x2 waypoint list for Mode A / C
%   start_pose : [x, y, theta] initial robot pose
%   goal       : [gx, gy] goal position (= start, completing the circle)

bounds = [-12, -12, 12, 12];

% No obstacles
rects = zeros(0, 4);
circs = zeros(0, 3);

world = world_build(rects, circs, bounds);

% 24 waypoints at 15-degree intervals, starting just past 0 degrees so
% that the last waypoint position (at 360 degrees = 0 degrees) coincides
% with the start/goal position (5, 0).  Note: the heading at start_pose
% is pi/2 (north) and is independent of the waypoint angles.
radius = 5.0;
n_pts  = 24;
angles = linspace(2*pi/n_pts, 2*pi, n_pts);   % 15°, 30°, ..., 360°

waypoints = [radius * cos(angles'), radius * sin(angles')];

start_pose = [radius, 0.0, pi/2];   % start at (5, 0), heading north
goal       = [radius, 0.0];          % full circle → back to start
end
