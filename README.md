# MATLAB Differential-Drive Robot Simulator

A complete 2-D ground-robot simulation built in pure MATLAB (no toolboxes
required).  Inspired by the methods in the
[YouTube robotics playlist](https://www.youtube.com/playlist?list=PLROrKak1fmNVBb_pGunToXlgGbsg5XE4n).

---

## Features

| Feature | Detail |
|---|---|
| **Kinematics** | Differential-drive unicycle model (x, y, θ) |
| **Controller A** | Pure-pursuit waypoint following |
| **Controller B** | Potential-field obstacle avoidance |
| **Sensor** | Simulated 360° 2-D lidar with Gaussian noise & ray casting |
| **Obstacles** | Axis-aligned rectangles + circles |
| **Scenarios** | Open space, cluttered field, narrow corridor |
| **Toolbox** | Auto-detects Robotics System Toolbox; falls back to pure MATLAB |

---

## Folder Structure

```
MATLAB-Robot-Sim-Test/
├── run_simulation.m          ← single entry-point script
├── README.md
│
├── src/
│   ├── robot_params.m        ← wheel radius, wheelbase, speed limits
│   ├── ddrive_step.m         ← Euler-integrate kinematics + collision check
│   ├── controller_waypoints.m← pure-pursuit controller (Mode A)
│   ├── controller_avoidance.m← potential-field controller (Mode B)
│   ├── sim_lidar.m           ← ray-casting 2-D lidar with noise
│   ├── world_build.m         ← assemble world struct from obstacle lists
│   └── visualize_step.m      ← real-time 2-D plot (handle-graphics)
│
├── scenarios/
│   ├── scenario_open.m       ← open arena, few obstacles
│   ├── scenario_cluttered.m  ← dense obstacle field
│   └── scenario_corridor.m   ← long narrow passage, staggered pillars
│
├── tests/
│   └── test_kinematics.m     ← 6 unit tests (kinematics + goal reachability)
│
└── docs/                     ← figures / GIFs (optional)
```

---

## Requirements

- MATLAB **R2023b** or newer (R2021a+ will also work in practice).
- No toolboxes are mandatory.  If the **Robotics System Toolbox** is
  present it will be detected and reported, but the simulation uses its
  own pure-MATLAB implementations throughout.

---

## How to Run

### 1 – Open MATLAB and navigate to the project root

```matlab
cd('path/to/MATLAB-Robot-Sim-Test')
```

### 2 – Run a scenario

```matlab
% Scenario 1: open space, waypoint following (default)
run_simulation

% Scenario 1: open space, obstacle avoidance
run_simulation(1, 'B')

% Scenario 2: cluttered field, waypoint following
run_simulation(2, 'A')

% Scenario 2: cluttered field, obstacle avoidance
run_simulation(2, 'B')

% Scenario 3: narrow corridor, waypoint following
run_simulation(3, 'A')

% Scenario 3: narrow corridor, obstacle avoidance
run_simulation(3, 'B')
```

### 3 – Run the unit tests

```matlab
run('tests/test_kinematics.m')
```

Expected output:
```
=== Kinematics & Controller Tests ===

[PASS] T1a: x advances correctly
[PASS] T1b: y stays zero
...
[PASS] T6: reaches goal within 25 s

=== Results: 14 passed,  0 failed ===
[ALL TESTS PASSED]
```

---

## Tunable Parameters

All easy-to-change parameters are at the top of `run_simulation.m`:

| Parameter | Default | Meaning |
|---|---|---|
| `dt` | `0.05` s | Simulation timestep |
| `T_MAX` | `120` s | Maximum runtime |
| `GOAL_TOL` | `0.30` m | Goal-reached distance threshold |
| `SHOW_LIDAR` | `true` | Toggle lidar ray overlay |
| `ctrl_wp.v_max` | `0.40` m/s | Max speed, Mode A |
| `ctrl_wp.k_omega` | `2.50` | Heading-error gain, Mode A |
| `ctrl_av.k_att` | `0.60` | Attractive-potential gain, Mode B |
| `ctrl_av.k_rep` | `0.90` | Repulsive-potential gain, Mode B |
| `ctrl_av.d0` | `1.50` m | Obstacle influence radius, Mode B |
| `lidar.num_rays` | `36` | Lidar ray count |
| `lidar.max_range` | `5.0` m | Lidar maximum range |
| `lidar.noise_std` | `0.02` m | Lidar noise (Gaussian σ) |

Physical robot parameters (wheel radius, wheelbase, speed limits) live in
`src/robot_params.m`.

---

## Scenarios

### Scenario 1 – Open Space
12 × 12 m arena with 3 rectangles and 2 circles.  Wide clearances; tests
basic waypoint navigation and avoidance in a benign environment.

### Scenario 2 – Cluttered
13 × 13 m arena with 8 rectangles and 5 circles.  The waypoints thread
through the tightest gaps; Mode B must find its way dynamically.

### Scenario 3 – Narrow Corridor
14 × 4 m passage bounded by solid top/bottom walls.  Three staggered
interior pillars force the robot to slalom from side to side.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Undefined function 'robot_params'` | Run from the project root, or call `run_simulation` (it adds `src/` to the path automatically). |
| `Undefined function 'scenario_open'` | Same as above – `scenarios/` is added by `run_simulation`. |
| Figure appears blank / crashes on CI | MATLAB needs a display.  On headless systems add `set(0,'DefaultFigureVisible','off')` before calling `run_simulation`. |
| Robot collides repeatedly in Mode B | Increase `ctrl_av.d0` or decrease `ctrl_av.k_rep`; the repulsive field may be too strong in tight spaces. |
| Simulation takes too long | Reduce `T_MAX` or increase `dt` (e.g. `dt = 0.10`). |
| Waypoints not reached in Mode A | Check that waypoints do not lie inside obstacles; also try reducing `ctrl_wp.lookahead`. |
| `license('test',...)` always returns 0 | Normal if Robotics System Toolbox is not installed; the simulation works without it. |