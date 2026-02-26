function [state_new, collided] = ddrive_step(state, v, omega, dt, params, world)
%DDRIVE_STEP  Integrate one timestep of differential-drive kinematics.
%
%   [state_new, collided] = ddrive_step(state, v, omega, dt, params, world)
%
%   Inputs
%   ------
%   state  : [x, y, theta] current robot pose (1x3 row vector)
%   v      : commanded linear  velocity [m/s]
%   omega  : commanded angular velocity [rad/s]
%   dt     : timestep [s]
%   params : robot parameter struct from robot_params()
%   world  : world struct from world_build()
%
%   Outputs
%   -------
%   state_new : [x, y, theta] updated pose
%   collided  : logical – true if the proposed position overlaps an obstacle
%               or exits the map boundary; translation is reverted on collision.

% Clamp commanded velocities to hardware limits
v     = max(-params.max_v,     min(params.max_v,     v));
omega = max(-params.max_omega, min(params.max_omega, omega));

% Euler integration (unicycle / differentialdrive model)
x_new     = state(1) + v * cos(state(3)) * dt;
y_new     = state(2) + v * sin(state(3)) * dt;
theta_new = state(3) + omega * dt;
theta_new = atan2(sin(theta_new), cos(theta_new));  % wrap to (-pi, pi]

state_new = [x_new, y_new, theta_new];

% Collision / boundary check
collided = check_collision([x_new, y_new], params.robot_radius, world);
if collided
    % Revert translation; keep the heading update so the robot can rotate free
    state_new(1) = state(1);
    state_new(2) = state(2);
end
end

% =========================================================================
%  Helper – point-in-obstacle collision check
% =========================================================================
function hit = check_collision(pos, r, world)
%CHECK_COLLISION  Returns true if the circle (pos, r) overlaps any obstacle
%                 or falls outside the world boundary.
hit = false;

% World boundary (robot must stay inside)
b = world.bounds;
if pos(1) - r < b(1) || pos(1) + r > b(3) || ...
   pos(2) - r < b(2) || pos(2) + r > b(4)
    hit = true; return;
end

% Axis-aligned rectangles (inflated by robot radius r)
for k = 1 : size(world.rects, 1)
    cx = world.rects(k,1); cy = world.rects(k,2);
    hw = world.rects(k,3)/2 + r;
    hh = world.rects(k,4)/2 + r;
    if abs(pos(1) - cx) <= hw && abs(pos(2) - cy) <= hh
        hit = true; return;
    end
end

% Circles (inflated by robot radius r)
for k = 1 : size(world.circs, 1)
    cx = world.circs(k,1); cy = world.circs(k,2); rc = world.circs(k,3);
    if norm(pos - [cx, cy]) <= rc + r
        hit = true; return;
    end
end
end
