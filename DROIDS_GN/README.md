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
| Also run the Simulink model | Set `cfg.run.simulink = true` in `mission_params` |
| Also compute power draw | Set `cfg.run.power = true` in `mission_params` |
| Power analysis by itself | `addpath power; main_motorf_robust` |
| Use a random AOI | Set `cfg.run.random_aoi = true` in `mission_params` |

All inputs live in `mission_params.m`: run options, location/AOI, drone, flight profile, camera, guidance, timing, controller gains, power, wind, noise, and power-sim cases. To use any polygon as the AOI, set `cfg.aoiPoly`, a 2xN array of [N; E] points. The lanes are planned over the AOI grown by `cfg.aoiBuffer` (default 0.5 m) on every side, so the border and corners are still covered with tracking error. With `cfg.laneExtend` (default on), each lane also runs one turn radius (v²/a) past that boundary, so the turns between lanes happen outside it. `run_mission` prints the resulting coverage of the AOI itself.

## How the pieces connect

```
mission_params -> gn_init (cfg)
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
| `mission_params.m` | Every input, returned as `cfg`. Flat fields feed the planner and Simulink; grouped structs (`cfg.drone`, `cfg.wind`, ...) feed the power analysis. |
| `gncc_uav_toolbox.slx` | Simulink model (UAV Toolbox multirotor) that flies the mission. Run `gn_init` first. |

## Files: mission planning (`gn/`)

| File | Purpose |
|---|---|
| `gn_init.m` | Script. Adds `gn/` and `power/` to the path, then loads `cfg` from `mission_params`. Calls `gn_build_mission` and defines the Simulink bus objects. Run it before the Simulink model. |
| `gn_build_mission.m` | Turns `cfg` into the mission. Returns the search waypoints, key points (home in the air, search entry, last search point), the `mission` struct, and the raw planner output. |
| `plan_mission.m` | Builds one smoothed trajectory per flight stage. Each stage starts and ends at rest. |
| `buffer_aoi.m` | Grows the AOI outward by `cfg.aoiBuffer` with square (mitered) corners, giving the area the lanes are planned over. |
| `coverage_check.m` | Fraction of the AOI within the camera footprint half-width of the search track, plus the worst-case margin: how much tracking error the path tolerates before a gap opens. |
| `search_boustrophedon.m` | Coverage path over a simple polygon. Lane spacing comes from the camera footprint. The lane heading is chosen automatically (or set manually), and transits go around the area when needed. Optional lane overrun past the boundary at both ends (connectors shift out with it). |
| `smooth_path.m` | Converts a waypoint list into a smooth trajectory. Corners become arcs of radius v²/a. Corners that are too tight, or sharper than 127°, become "bulb" turns that loop outside the corner. Above 127° the bulb starts at the corner itself, because an inside arc there would cut more than 2r off the search lane (about 4 m at a hairpin). The trajectory respects the v_max and a_max limits. |
| `traj_eval.m` | Gives p, v, a, and yaw at time t on a `smooth_path` trajectory. |
| `mission_eval.m` | Gives p, v, a, and yaw for a given stage at a time since that stage began. |
| `mission_sequencer.m` | Stage state machine: idle → takeoff → transit → search → return → landing → landed. A stage advances when its trajectory is done and the vehicle is within tolerance of the stage end point. |
| `stage_reference.m` | Simulink-facing version of `mission_eval`: p, v, a, and a done flag for the Mission Manager phase (0–6). |

All `%#codegen` functions can be used directly in MATLAB Function blocks.

## Files: power analysis (`power/`)

This analysis flies the planned mission through a closed-loop quadrotor simulation. The simulation chain is: velocity command → body-tilt cascade controller → motor mixer → motor saturation and lag → nonlinear EOM with wind, gust, and drag. It is run for four cases (set in `cfg.cases`):

- ideal
- wind + gusts
- wind + gusts + ESKF, standalone GPS
- wind + gusts + ESKF, RTK GPS

In the ESKF cases the controller sees a state estimate, not the truth: true state → synthetic IMU, baro, and GPS → 16-state error-state Kalman filter (`power/eskf16/`, from the D&C team's `quadrotor_gnc 2`) → controller. The older AR(1) noise model is still available with `noise = true` on a case.

The motor forces from each case are then converted to electrical power.

| File | Purpose |
|---|---|
| `main_motorf_robust.m` | Driver: `results = main_motorf_robust(cfg)`. With no input, it uses `mission_params`. Prints force and power tables and writes figures plus a CSV to `power/figures/`. |
| `RobustClosedLoopEOM.m` | One state derivative. Covers mission tracking, controller, mixer, motor saturation and lag, and the plant. |
| `RobustQuadrotorEOM.m` | 12-state nonlinear EOM with air-relative drag and a center-of-pressure moment. |
| `ClosedLoopController.m` | Body-tilt cascade: position → velocity PI → attitude → force and moment commands. |
| `VelocityCommand.m` | Outer-loop velocity command for one axis (position or velocity mode). |
| `ComputeMotorForces.m` | Mixer: force and moment commands → f1 through f4. |
| `DrydenGusts.m` | MIL-F-8785C low-altitude Dryden turbulence time series in NED. |
| `MissionReference.m` | Interpolates the sampled mission p, v, and a at time t (a feeds the acceleration feedforward, `cfg.ctrl.acc_ff`). |
| `eskf16/` | 16-state ESKF (position, velocity, attitude, gyro/accel/baro biases): init, predict, GPS and baro updates, sensor specs (`eskf16_specs.m`). |
| `motor_power.m` | Per-motor power from thrust, P = P_hover·(f/f_hover)^n, fit to the weight/power sheet. |
| `PlotMotorfRobust.m` | Figures for motor forces, tracking error, and case comparison. |
| `PlotMissionPower.m` | Figures for total and per-motor power and cumulative energy, shaded by stage. |
| `PlotMissionTW.m` | Figures for total and per-motor thrust-to-weight and thrust headroom on the most-loaded motor, shaded by stage. |
| `PlotNoiseFilter.m` | One figure per ESKF case: raw sensor error (GPS, baro, gyro) next to the error after the ESKF and gyro low-pass, with shared y-limits per row. |
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
