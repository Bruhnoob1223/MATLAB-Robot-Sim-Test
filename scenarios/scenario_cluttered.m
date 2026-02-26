function [world, waypoints, start_pose, goal] = scenario_cluttered()
%SCENARIO_CLUTTERED  Dense obstacle field requiring careful routing.
%
%   A 13 x 13 m space with 8 rectangles and 5 circles.
%   The waypoints thread through the tightest gaps.
%
%   Returns
%   -------
%   world      : world struct (see world_build)
%   waypoints  : Nx2 waypoint list for Mode A
%   start_pose : [x, y, theta] initial robot pose
%   goal       : [gx, gy] goal position

bounds = [-0.5, -0.5, 12.5, 12.5];

% Rectangles: [cx, cy, width, height]
rects = [
    2.0,  1.5,  1.2,  1.2;
    4.5,  3.5,  0.8,  2.0;
    7.0,  1.8,  1.0,  1.0;
    1.5,  5.5,  1.0,  1.5;
    8.5,  5.0,  1.2,  1.0;
    3.5,  7.8,  1.5,  0.8;
    7.0,  9.0,  1.0,  1.5;
    10.0, 7.5,  0.8,  2.0;
];

% Circles: [cx, cy, radius]
circs = [
    5.5,  5.5,  0.7;
    2.5,  9.5,  0.5;
    9.0,  2.5,  0.6;
    6.5,  7.5,  0.5;
    4.5,  1.5,  0.4;
];

world = world_build(rects, circs, bounds);

% Waypoints: hand-tuned to navigate through the gaps
waypoints = [
    0.8,  0.5;
    3.5,  0.6;
    6.0,  2.8;
    6.5,  5.5;
    5.5,  8.0;
    8.5,  8.5;
    10.5, 10.5;
    12.0, 12.0;
];

start_pose = [0.5,  0.5,  0.0];
goal       = [12.0, 12.0];
end
