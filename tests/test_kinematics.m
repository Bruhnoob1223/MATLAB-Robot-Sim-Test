%TEST_KINEMATICS  Unit tests for the differential-drive kinematic model.
%
%   Run from the project root:
%       >> run('tests/test_kinematics.m')
%   or change to the tests directory:
%       >> cd tests; test_kinematics
%
%   Tests covered
%   -------------
%   T1 – Pure forward motion                (4 assertions)
%   T2 – Pure rotation (zero linear speed)  (3 assertions)
%   T3 – Heading wrap-around at ±pi         (1 assertion)
%   T4 – Motion at 45-degree heading        (2 assertions)
%   T5 – Speed-limit enforcement            (1 assertion)
%   T6 – Goal reachability via waypoint controller, 25 s budget

fprintf('=== Kinematics & Controller Tests ===\n\n');

% Add source paths relative to this file
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, '..', 'src'));
addpath(fullfile(script_dir, '..', 'scenarios'));

rp    = robot_params();
world = world_build([], [], [-1000, -1000, 1000, 1000]);  % obstacle-free world

n_pass = 0;
n_fail = 0;
tol    = 1e-9;

% =========================================================================
%  T1 – Pure forward motion
% =========================================================================
state = [0, 0, 0];
v = 0.4;  omega = 0;  dt = 0.1;
[s, col] = ddrive_step(state, v, omega, dt, rp, world);

[n_pass, n_fail] = chk('T1a: x advances correctly',      abs(s(1) - v*dt) < tol, n_pass, n_fail);
[n_pass, n_fail] = chk('T1b: y stays zero',              abs(s(2))        < tol, n_pass, n_fail);
[n_pass, n_fail] = chk('T1c: theta stays zero',          abs(s(3))        < tol, n_pass, n_fail);
[n_pass, n_fail] = chk('T1d: no collision in free world', ~col,                  n_pass, n_fail);

% =========================================================================
%  T2 – Pure rotation (v = 0)
% =========================================================================
state = [0, 0, 0];
v = 0;  omega = 1.0;  dt = 0.5;
[s, ~] = ddrive_step(state, v, omega, dt, rp, world);

expected_theta = omega * dt;   % 0.5 rad (within max_omega = 2.0)
[n_pass, n_fail] = chk('T2a: x stays zero for pure rotation', abs(s(1)) < tol,                  n_pass, n_fail);
[n_pass, n_fail] = chk('T2b: y stays zero for pure rotation', abs(s(2)) < tol,                  n_pass, n_fail);
[n_pass, n_fail] = chk('T2c: theta increments correctly',     abs(s(3) - expected_theta) < tol, n_pass, n_fail);

% =========================================================================
%  T3 – Heading wrap-around
% =========================================================================
state = [0, 0, pi - 0.05];      % very close to +pi
v = 0;  omega = 1.0;  dt = 0.2; % pushes heading just past +pi → wraps negative
[s, ~] = ddrive_step(state, v, omega, dt, rp, world);

[n_pass, n_fail] = chk('T3: heading stays in (-pi, pi]', s(3) >= -pi && s(3) <= pi, n_pass, n_fail);

% =========================================================================
%  T4 – Motion at 45-degree heading
% =========================================================================
state = [0, 0, pi/4];
v_cmd = 0.3;  omega = 0;  dt = 0.1;
[s, ~] = ddrive_step(state, v_cmd, omega, dt, rp, world);

[n_pass, n_fail] = chk('T4a: x correct at 45 deg', abs(s(1) - v_cmd*cos(pi/4)*dt) < tol, n_pass, n_fail);
[n_pass, n_fail] = chk('T4b: y correct at 45 deg', abs(s(2) - v_cmd*sin(pi/4)*dt) < tol, n_pass, n_fail);

% =========================================================================
%  T5 – Speed-limit enforcement
% =========================================================================
state = [0, 0, 0];
v_over = rp.max_v * 5;   % 5× the speed limit
[s, ~] = ddrive_step(state, v_over, 0, 0.1, rp, world);

[n_pass, n_fail] = chk('T5: linear speed limit enforced', s(1) <= rp.max_v * 0.1 + tol, n_pass, n_fail);

% =========================================================================
%  T6 – Goal reachability via waypoint controller (≤ 25 s)
% =========================================================================
state     = [0, 0, 0];
waypoints = [5, 0];   % single waypoint directly ahead
goal_T6   = [5, 0];
wp_idx    = 1;
ctrl6.k_omega   = 2.0;
ctrl6.lookahead = 0.8;
ctrl6.v_max     = 0.4;

dt_test  = 0.05;
reached6 = false;
for iter = 1 : 500          % 500 × 0.05 s = 25 s
    [v6, omega6, wp_idx] = controller_waypoints(state, waypoints, wp_idx, ctrl6, rp);
    [state, ~] = ddrive_step(state, v6, omega6, dt_test, rp, world);
    if norm(state(1:2) - goal_T6) < 0.3
        reached6 = true;
        break;
    end
end

[n_pass, n_fail] = chk( ...
    sprintf('T6: goal reached within 25 s (final dist = %.3f m)', norm(state(1:2) - goal_T6)), ...
    reached6, n_pass, n_fail);

% =========================================================================
%  Summary
% =========================================================================
fprintf('\n=== Results: %d passed,  %d failed ===\n', n_pass, n_fail);
if n_fail == 0
    fprintf('[ALL TESTS PASSED]\n\n');
else
    fprintf('[%d TEST(S) FAILED – see messages above]\n\n', n_fail);
    error('test_kinematics: %d test(s) failed.', n_fail);
end

% =========================================================================
%  Local helper – accepts and returns counters to work around script scope
% =========================================================================
function [np, nf] = chk(msg, condition, np, nf)
if condition
    fprintf('[PASS] %s\n', msg);
    np = np + 1;
else
    fprintf('[FAIL] %s\n', msg);
    nf = nf + 1;
end
end
