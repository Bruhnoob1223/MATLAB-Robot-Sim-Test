function run_simulation(scenario_id, mode)
%RUN_SIMULATION  Main entry point for the differential-drive robot simulation.
%
%   Usage
%   -----
%   run_simulation              – Scenario 1 (open),     Mode A (waypoints)
%   run_simulation(2)           – Scenario 2 (cluttered), Mode A
%   run_simulation(3, 'B')      – Scenario 3 (corridor),  Mode B (avoidance)
%   run_simulation(1, 'B')      – Scenario 1,             Mode B
%   run_simulation(4, 'C')      – Scenario 4 (circle),    Mode C (3-D)
%
%   Modes
%   -----
%   'A'  Pure-pursuit waypoint following (2-D view)
%   'B'  Potential-field obstacle avoidance (2-D view)
%   'C'  Pure-pursuit waypoint following with 3-D visualisation
%
%   Scenarios
%   ---------
%   1  Open space  – few scattered obstacles
%   2  Cluttered   – dense obstacle field
%   3  Corridor    – long narrow passage with staggered pillars
%   4  Circle      – obstacle-free arena; robot drives one full circle
%
%   Requirements: MATLAB R2023b+.  Robotics System Toolbox is optional.
%
%   See also: src/robot_params, src/ddrive_step, src/controller_waypoints,
%             src/controller_avoidance, src/sim_lidar, src/world_build,
%             src/visualize_step

% =========================================================================
%  DEFAULT ARGUMENTS
% =========================================================================
if nargin < 1 || isempty(scenario_id), scenario_id = 1; end
if nargin < 2 || isempty(mode),        mode        = 'A'; end

mode = upper(mode);

% =========================================================================
%  TUNABLE PARAMETERS  ← easy to change here
% =========================================================================

dt       = 0.05;    % simulation timestep [s]
T_MAX    = 120;     % maximum simulation time [s]
GOAL_TOL = 0.30;    % goal-reached tolerance [m]
SHOW_LIDAR = true;  % overlay lidar rays on the plot

% Waypoint controller gains (Mode A)
ctrl_wp.lookahead = 0.80;   % pure-pursuit lookahead radius [m]
ctrl_wp.k_omega   = 2.50;   % proportional heading-error gain
ctrl_wp.v_max     = 0.40;   % maximum commanded linear speed [m/s]

% Avoidance controller gains (Mode B)
ctrl_av.k_att    = 0.60;    % attractive potential gain
ctrl_av.k_rep    = 0.90;    % repulsive potential gain
ctrl_av.d0       = 1.50;    % obstacle influence distance [m]
ctrl_av.v_max    = 0.35;    % maximum commanded linear speed [m/s]
ctrl_av.omega_max = 1.80;   % maximum commanded angular speed [rad/s]

% Lidar parameters
lidar.num_rays  = 36;       % number of rays
lidar.fov       = 2 * pi;   % field of view [rad]   (360 degrees)
lidar.max_range = 5.0;      % maximum range [m]
lidar.noise_std = 0.02;     % Gaussian noise standard deviation [m]

% =========================================================================
%  TOOLBOX DETECTION
% =========================================================================
HAS_RST = license('test', 'Robotics_System_Toolbox');
if HAS_RST
    fprintf('[INFO] Robotics System Toolbox detected (pure-MATLAB fallbacks not needed).\n');
else
    fprintf('[INFO] Robotics System Toolbox NOT found – using pure-MATLAB implementations.\n');
end

% =========================================================================
%  PATH SETUP
% =========================================================================
script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;   % called from the project root as a script
end
addpath(fullfile(script_dir, 'src'));
addpath(fullfile(script_dir, 'scenarios'));

% =========================================================================
%  LOAD SCENARIO
% =========================================================================
switch scenario_id
    case 1,  [world, waypoints, start_pose, goal] = scenario_open();
    case 2,  [world, waypoints, start_pose, goal] = scenario_cluttered();
    case 3,  [world, waypoints, start_pose, goal] = scenario_corridor();
    case 4,  [world, waypoints, start_pose, goal] = scenario_circle();
    otherwise
        error('Unknown scenario %d.  Valid choices: 1, 2, 3, 4.', scenario_id);
end

fprintf('[INFO] Scenario %d loaded.  Mode = %s\n', scenario_id, mode);

% =========================================================================
%  ROBOT PARAMETERS
% =========================================================================
rp = robot_params();

% =========================================================================
%  SIMULATION STATE
% =========================================================================
state   = start_pose;       % [x, y, theta]
trail   = state(1:2);       % Mx2  position history
wp_idx  = 1;                % current waypoint index (Mode A)
t       = 0;
step    = 0;
handles = [];
reached = false;

fprintf('[INFO] Simulating.  Press Ctrl-C to abort.\n');
fprintf('       start=(%.2f, %.2f)  goal=(%.2f, %.2f)\n', ...
        start_pose(1), start_pose(2), goal(1), goal(2));

% =========================================================================
%  MAIN SIMULATION LOOP
% =========================================================================
while t < T_MAX && ~reached

    step = step + 1;
    t    = t + dt;

    % --- Sense (lidar) --------------------------------------------------
    if ~strcmp(mode, 'C')
        [ranges, ray_angles] = sim_lidar(state, world, lidar);
    else
        ranges = [];  ray_angles = [];
    end

    % --- Plan / Control -------------------------------------------------
    switch mode
        case {'A', 'C'}
            [v, omega, wp_idx] = controller_waypoints( ...
                state, waypoints, wp_idx, ctrl_wp, rp);
        case 'B'
            [v, omega] = controller_avoidance( ...
                state, goal, ranges, ray_angles, ctrl_av, rp);
        otherwise
            error('Unknown mode ''%s''.  Choose ''A'', ''B'', or ''C''.', mode);
    end

    % --- Act ------------------------------------------------------------
    [state, collided] = ddrive_step(state, v, omega, dt, rp, world);

    if collided
        fprintf('[WARN] Collision at t=%.2f s  (x=%.2f, y=%.2f)\n', ...
                t, state(1), state(2));
    end

    % --- Record trail ---------------------------------------------------
    trail(end+1, :) = state(1:2); %#ok<AGROW>

    % --- Visualise ------------------------------------------------------
    if strcmp(mode, 'C')
        handles = visualize_step_3d(state, world, trail, waypoints, goal, step, handles);
    elseif SHOW_LIDAR
        handles = visualize_step(state, world, ranges, ray_angles, ...
                                 trail, waypoints, goal, step, handles);
    else
        handles = visualize_step(state, world, [], [], ...
                                 trail, waypoints, goal, step, handles);
    end

    % --- Goal check -----------------------------------------------------
    dist_to_goal = norm(state(1:2) - goal);
    if dist_to_goal < GOAL_TOL
        reached = true;
        fprintf('[INFO] Goal reached at t = %.2f s  (final dist = %.3f m)\n', ...
                t, dist_to_goal);
    end

end % simulation loop

if ~reached
    fprintf('[INFO] Simulation ended at t = %.2f s  (goal not reached, dist = %.2f m)\n', ...
            t, norm(state(1:2) - goal));
end

fprintf('[INFO] Done.  Total steps: %d\n', step);
end
