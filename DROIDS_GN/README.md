# DROIDS_GN — Guidance & Navigation

This folder has three parts:

- **Mission planning.** A boustrophedon (lawnmower) search over an area of interest (AOI), smoothed into five flight stages: takeoff, transit, search, return, and landing.
- **A Simulink model** that flies the planned mission.
- **A power analysis** (`power/`) that flies the mission in a closed-loop simulation and turns the motor forces into battery energy.

All coordinates are local NED (North, East, Down). Units are SI: m, s, rad.

## Quick start

| What | How |
|---|---|
| Plan a mission and plot it | Run `run_mission` |
| Also run the Simulink model | Set `run_simulink = true` in `run_mission` |
| Also compute power draw | Set `run_power = true` in `run_mission` |
| Power analysis by itself | `addpath power; main_motorf_robust` |
| Use a random AOI | Set `random_aoi = true` in `run_mission` |

To change the mission, edit `gn/gn_init.m`. It sets the AOI, altitude, speeds, and camera FOV and overlap. To use any polygon as the AOI, set `cfg.aoiPoly`, a 2xN array of [N; E] points.

## How the pieces connect

```
gn_init (cfg)
  └─ gn_build_mission(cfg)
       └─ plan_mission
            ├─ search_boustrophedon    lane waypoints over the AOI
            └─ smooth_path             corner-rounded, speed/accel-limited trajectory per stage
                                    -> mission struct

At run time (MATLAB loop or Simulink):
  mission_sequencer   picks the current stage and time in that stage
  mission_eval        gives reference p, v, a, yaw for that stage   (uses traj_eval)
```

## Folder layout

```
DROIDS_GN/
  run_mission.m          entry point
  gncc_uav_toolbox.slx   Simulink model
  gn/                    mission planning (guidance & navigation)
  power/                 power analysis
```

## Top-level files

| File | Purpose |
|---|---|
| `run_mission.m` | Main script. Plans the mission and prints a summary. Plots the 3D plan, the smoothed path, and speed/acceleration by stage. Can also run Simulink and/or the power analysis. |
| `gncc_uav_toolbox.slx` | Simulink model (UAV Toolbox multirotor) that flies the mission. Run `gn_init` first. |

## Files: mission planning (`gn/`)

| File | Purpose |
|---|---|
| `gn_init.m` | Script. Adds `gn/` and `power/` to the path, then builds `cfg` (AOI, altitude, speeds, camera FOV/overlap → lane spacing, sample times, gains). Calls `gn_build_mission` and defines the Simulink bus objects. Run it before the Simulink model. |
| `gn_build_mission.m` | Turns `cfg` into the mission. Returns the search waypoints, key points (home in the air, search entry, last search point), the `mission` struct, and the raw planner output. |
| `plan_mission.m` | Builds one smoothed trajectory per flight stage. Each stage starts and ends at rest. |
| `search_boustrophedon.m` | Coverage path over a simple polygon. Lane spacing comes from the camera footprint. The lane heading is chosen automatically (or set manually), and transits go around the area when needed. |
| `smooth_path.m` | Converts a waypoint list into a smooth trajectory. Corners become arcs of radius v²/a. Corners that are too tight become "bulb" turns. The trajectory respects the v_max and a_max limits. |
| `traj_eval.m` | Gives p, v, a, and yaw at time t on a `smooth_path` trajectory. |
| `mission_eval.m` | Gives p, v, a, and yaw for a given stage at a time since that stage began. |
| `mission_sequencer.m` | Stage state machine: idle → takeoff → transit → search → return → landing → landed. A stage advances when its trajectory is done and the vehicle is within tolerance of the stage end point. |
| `stage_reference.m` | Simulink-facing version of `mission_eval`: p, v, a, and a done flag for the Mission Manager phase (0–6). |

All `%#codegen` functions can be used directly in MATLAB Function blocks.

## Files: power analysis (`power/`)

This analysis flies the planned mission through a closed-loop quadrotor simulation. The simulation chain is: velocity command → body-tilt cascade controller → motor mixer → motor saturation and lag → nonlinear EOM with wind, gust, and drag. It is run for three cases:

- ideal
- wind + gusts
- wind + gusts + sensor noise

The motor forces from each case are then converted to electrical power.

| File | Purpose |
|---|---|
| `main_motorf_robust.m` | Driver: `results = main_motorf_robust(cfg)`. With no input, it uses `gn_init`. Settings are at the top: vehicle, gains, motor, aero, wind, noise, cases. Prints force and power tables and writes figures plus a CSV to `power/figures/`. |
| `RobustClosedLoopEOM.m` | One state derivative. Covers mission tracking, controller, mixer, motor saturation and lag, and the plant. |
| `RobustQuadrotorEOM.m` | 12-state nonlinear EOM with air-relative drag and a center-of-pressure moment. |
| `ClosedLoopController.m` | Body-tilt cascade: position → velocity PI → attitude → force and moment commands. |
| `VelocityCommand.m` | Outer-loop velocity command for one axis (position or velocity mode). |
| `ComputeMotorForces.m` | Mixer: force and moment commands → f1 through f4. |
| `DrydenGusts.m` | MIL-F-8785C low-altitude Dryden turbulence time series in NED. |
| `MissionReference.m` | Interpolates the sampled mission p and v at time t. |
| `motor_power.m` | Per-motor power from thrust, P = P_hover·(f/f_hover)^n, fit to the weight/power sheet. |
| `PlotMotorfRobust.m` | Figures for motor forces, tracking error, and case comparison. |
| `PlotMissionPower.m` | Figures for total and per-motor power and cumulative energy, shaded by stage. |
| `SaveCurrentFigure.m` | Saves the current figure as a PNG with a light theme. |
| `Weight and Power.xlsx` | Weight and power budget. Sheet3 provides the hover and 1.5×-hover power, avionics power, and battery size for each AUW. |

### Power model

These values come from the 3.2 kg row of Sheet3:

- **Hover:** 800 gf per rotor, 727 W for all four motors.
- **At 1.5× hover thrust:** 1240 W. Together with the hover point, this gives the exponent n = ln(1240/727)/ln(1.5) ≈ 1.32. That curve matches the sheet's motor data to within ±4%.
- **Avionics:** a constant 20 W is added.
- **Usable battery:** 1492 g × 180 Wh/kg × 80% ≈ 215 Wh.

Because the speed limit is only 0.5 m/s, power stays close to hover for the whole flight. Mission energy is therefore roughly 750 W × flight time.

**Known limitation:** the inertias in `main_motorf_robust.m` are from the earlier, lighter (~2.4 kg) vehicle. Only the mass has been updated to 3.2 kg.

## Generated / ignored

`slprj/` and `*.slxc` are Simulink build caches. They are rebuilt automatically and are listed in the repo `.gitignore`. `power/figures/` is regenerated on every power run and is also git-ignored.
