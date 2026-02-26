function [world, waypoints, start_pose, goal] = scenario_open()
%SCENARIO_OPEN  Open arena with a handful of isolated obstacles.
%
%   A 12 x 12 m space with 3 rectangles and 2 circles.
%   The robot navigates diagonally from bottom-left to top-right.
%
%   Returns
%   -------
%   world      : world struct (see world_build)
%   waypoints  : Nx2 waypoint list for Mode A
%   start_pose : [x, y, theta] initial robot pose
%   goal       : [gx, gy] goal position

bounds = [-0.5, -0.5, 11.5, 11.5];

% Rectangles: [cx, cy, width, height]  – axis-aligned
rects = [
    3.0,  2.0,  1.2,  2.0;   % left-side pillar
    7.5,  5.0,  1.8,  1.0;   % mid-field block
    5.0,  8.5,  2.0,  1.0;   % upper block
];

% Circles: [cx, cy, radius]
circs = [
    2.0,  7.5,  0.7;
    9.0,  3.0,  0.8;
];

world = world_build(rects, circs, bounds);

% Waypoints: route around the obstacles
waypoints = [
    1.0,  1.0;
    4.5,  1.5;
    6.0,  3.5;
    8.0,  7.5;
    10.0, 10.0;
];

start_pose = [0.5,  0.5,  0.0];   % [x, y, theta]
goal       = [10.0, 10.0];
end
