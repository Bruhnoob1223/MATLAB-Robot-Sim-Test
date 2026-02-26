function p = robot_params()
%ROBOT_PARAMS  Return a struct containing differential-drive robot parameters.
%
%   Modify the values in this file to retune the physical robot model.
%
%   Fields
%   ------
%   r            : wheel radius [m]
%   L            : wheelbase (center-to-center wheel distance) [m]
%   robot_radius : circular footprint radius used for collision checks [m]
%   max_v        : maximum linear velocity [m/s]
%   max_omega    : maximum angular velocity [rad/s]
%   max_accel    : maximum linear acceleration [m/s^2]  (informational)
%   max_alpha    : maximum angular acceleration [rad/s^2] (informational)

p.r            = 0.05;   % wheel radius [m]
p.L            = 0.30;   % wheelbase [m]
p.robot_radius = 0.20;   % collision footprint radius [m]
p.max_v        = 0.50;   % maximum linear velocity [m/s]
p.max_omega    = 2.00;   % maximum angular velocity [rad/s]
p.max_accel    = 1.50;   % maximum linear acceleration [m/s^2]
p.max_alpha    = 3.00;   % maximum angular acceleration [rad/s^2]
end
