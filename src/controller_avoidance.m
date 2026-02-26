function [v, omega] = controller_avoidance(state, goal, ranges, ray_angles, ctrl, rp)
%CONTROLLER_AVOIDANCE  Potential-field obstacle-avoidance controller.
%
%   [v, omega] = controller_avoidance(state, goal, ranges, ray_angles, ctrl, rp)
%
%   Uses an attractive potential toward the goal and repulsive potentials
%   from nearby lidar-detected obstacles to compute (v, omega) commands.
%
%   Inputs
%   ------
%   state      : [x, y, theta] current robot pose
%   goal       : [gx, gy] goal position
%   ranges     : Nx1 lidar range readings [m]
%   ray_angles : Nx1 ray angles RELATIVE to robot heading [rad]
%   ctrl       : controller parameter struct with fields:
%                  .k_att    – attractive potential gain
%                  .k_rep    – repulsive potential gain
%                  .d0       – obstacle influence distance [m]
%                  .v_max    – maximum commanded linear speed [m/s]
%                  .omega_max – maximum commanded angular speed [rad/s]
%   rp         : robot parameter struct from robot_params()
%
%   Outputs
%   -------
%   v     : commanded linear  velocity [m/s]
%   omega : commanded angular velocity [rad/s]

pos   = state(1:2);
theta = state(3);

% ---- Attractive force (unit-vector toward goal) -------------------------
vec_to_goal  = goal - pos;
dist_to_goal = norm(vec_to_goal);
if dist_to_goal > 1e-4
    F_att = ctrl.k_att * vec_to_goal / dist_to_goal;
else
    F_att = [0, 0];
end

% ---- Repulsive forces (from lidar readings inside influence radius) -----
F_rep = [0, 0];
n_rays = numel(ranges);
for i = 1 : n_rays
    d = ranges(i);
    if d < ctrl.d0 && d > 1e-3
        % World-frame direction from which the obstacle is detected
        abs_angle = theta + ray_angles(i);
        obs_dir   = [cos(abs_angle), sin(abs_angle)];  % robot → obstacle
        away_dir  = -obs_dir;                           % push away

        rep_mag = ctrl.k_rep * (1/d - 1/ctrl.d0) / (d^2);
        F_rep   = F_rep + rep_mag * away_dir;
    end
end

% ---- Combined force → desired heading & speed --------------------------
F_total = F_att + F_rep;
F_mag   = norm(F_total);

if F_mag < 1e-6
    v = 0; omega = 0;
    return;
end

desired_heading = atan2(F_total(2), F_total(1));
heading_error   = atan2(sin(desired_heading - theta), cos(desired_heading - theta));

% Scale speed by heading alignment so robot turns before driving forward
omega = 2.0 * heading_error;
omega = max(-ctrl.omega_max, min(ctrl.omega_max, omega));

v = ctrl.v_max * cos(heading_error)^2;
v = max(0, min(ctrl.v_max, v));
end
