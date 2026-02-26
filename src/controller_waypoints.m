function [v, omega, wp_idx] = controller_waypoints(state, waypoints, wp_idx, ctrl, rp)
%CONTROLLER_WAYPOINTS  Pure-pursuit waypoint-following controller.
%
%   [v, omega, wp_idx] = controller_waypoints(state, waypoints, wp_idx, ctrl, rp)
%
%   Inputs
%   ------
%   state     : [x, y, theta] current robot pose
%   waypoints : Nx2 matrix of [x, y] waypoints
%   wp_idx    : index of the current target waypoint
%   ctrl      : controller parameter struct with fields:
%                 .lookahead – lookahead radius for waypoint advancement [m]
%                 .k_omega   – proportional heading-error gain
%                 .v_max     – maximum commanded linear speed [m/s]
%   rp        : robot parameter struct from robot_params()
%
%   Outputs
%   -------
%   v      : commanded linear  velocity [m/s]
%   omega  : commanded angular velocity [rad/s]
%   wp_idx : (possibly advanced) waypoint index

n_wp = size(waypoints, 1);

% Advance waypoint index while the robot is within lookahead of the current target
while wp_idx <= n_wp && ...
        norm(state(1:2) - waypoints(wp_idx,:)) < ctrl.lookahead
    wp_idx = wp_idx + 1;
end

% All waypoints consumed – hold position
if wp_idx > n_wp
    v = 0; omega = 0;
    return;
end

target = waypoints(wp_idx, :);
dx = target(1) - state(1);
dy = target(2) - state(2);

% Compute heading error (angle wrapping handled by atan2)
desired_heading = atan2(dy, dx);
heading_error   = atan2(sin(desired_heading - state(3)), ...
                        cos(desired_heading - state(3)));

% Proportional angular velocity
omega = ctrl.k_omega * heading_error;
omega = max(-rp.max_omega, min(rp.max_omega, omega));

% Linear speed: full speed when aligned, slower when turning sharply
v = ctrl.v_max * cos(heading_error)^2;
v = max(0, min(ctrl.v_max, v));

% Slow to a stop at the very last waypoint
if wp_idx == n_wp && norm(state(1:2) - target) < 0.25
    v = 0;
end
end
