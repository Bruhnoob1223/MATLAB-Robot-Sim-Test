function [world, waypoints, start_pose, goal] = scenario_corridor()
%SCENARIO_CORRIDOR  Long narrow corridor with staggered interior pillars.
%
%   The robot starts at the left end of a 14 m long, 2 m clear corridor
%   (bounded by solid top and bottom walls) and must reach the right end
%   while avoiding three staggered pillars placed in alternating positions.
%
%   Corridor geometry
%   -----------------
%   Bottom wall : y = -0.5  (centre)  height 1 m  →  top edge at y = 0
%   Top    wall : y =  3.0  (centre)  height 1 m  →  bot edge at y = 2.5
%   Clear gap   : y = 0 … 2.5 m   (robot radius 0.2 m → ample clearance)
%
%   Returns
%   -------
%   world      : world struct (see world_build)
%   waypoints  : Nx2 waypoint list for Mode A
%   start_pose : [x, y, theta] initial robot pose
%   goal       : [gx, gy] goal position

bounds = [-1.0, -1.5, 15.0, 4.5];

% Rectangles: [cx, cy, width, height]
rects = [
    % Corridor walls (full length)
     7.0, -0.5, 14.0,  1.0;   % bottom wall  (top edge y = 0)
     7.0,  3.0, 14.0,  1.0;   % top    wall  (bot edge y = 2.5)

    % Staggered interior pillars (each 0.5 m wide, 1.2 m tall)
    %   pillar 1: upper – leaves gap below (robot squeezes y ≈ 0.3 … 1.1)
     3.0,  2.0,  0.5,  1.2;   % centre at (3, 2.0)  → spans y 1.4 … 2.6
    %   pillar 2: lower – leaves gap above (robot squeezes y ≈ 1.3 … 2.2)
     6.5,  0.9,  0.5,  1.2;   % centre at (6.5, 0.9) → spans y 0.3 … 1.5
    %   pillar 3: upper again
    10.0,  2.0,  0.5,  1.2;   % centre at (10, 2.0)  → spans y 1.4 … 2.6
];

circs = zeros(0, 3);

world = world_build(rects, circs, bounds);

% Waypoints snake through the staggered pillars
%   Corridor centre-line is y = 1.25 m
%   Below upper pillar: y ≈ 0.7   Above lower pillar: y ≈ 2.0
waypoints = [
     1.0,  1.25;   % corridor entry
     2.5,  0.70;   % low – clears pillar 1 from below
     4.5,  1.25;   % back to centre
     6.0,  2.00;   % high – clears pillar 2 from above
     8.0,  1.25;   % back to centre
     9.5,  0.70;   % low – clears pillar 3 from below
    11.5,  1.25;   % back to centre
    13.5,  1.25;   % corridor exit
];

start_pose = [0.3,  1.25,  0.0];   % enter from left, facing right
goal       = [13.5, 1.25];
end
