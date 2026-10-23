# ASEN 4018 — DROIDS UAS

Guidance, navigation, and power analysis for the DROIDS quadrotor, a senior design project for ASEN 4018.

## Layout

```
DROIDS_GN/
  run_mission.m          entry point: plan, plot, optional Simulink + power
  gncc_uav_toolbox.slx   Simulink model that flies the mission
  gn/                    guidance & navigation: mission config, search pattern, trajectories
  power/                 closed-loop motor forces -> power draw / battery energy
```

## Run

In MATLAB, open `DROIDS_GN/run_mission.m` and run it. The flags at the top control what runs:

- `random_aoi`: picks a random search area.
- `run_simulink`: also runs the Simulink model.
- `run_power`: also runs the power analysis.

See [DROIDS_GN/README.md](DROIDS_GN/README.md) for what each file does.

## Requirements

MATLAB with Simulink, UAV Toolbox, Stateflow, and Aerospace Blockset.
