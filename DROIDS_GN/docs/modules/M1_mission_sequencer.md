# M1 — Mission Sequencer (ConOps state machine)

Status: **Step M1.0 complete, awaiting approval.** No sequencer logic written yet.

## Purpose
The sequencer decides *which phase* the UAS is in (takeoff, transit, search, …) and tells
guidance which path to generate. It does not compute paths or control the vehicle.

```
 events (wp reached, marker found, ack, override, faults) ──▶ [ SEQUENCER ] ──▶ mode + phase targets ──▶ GUIDANCE
```

## Requirements trace (sources only — no new requirements invented)

| ID | What the sequencer must do | Source |
|---|---|---|
| R1 | Cover ConOps phases: pre-mission/init (0), link established (1), takeoff (2), transit to AOI (3), search (4), marker detection + verify (5), relay + confirm ack (6), return, land, disarm (7) | DR-1 slides 8–9 |
| R2 | Every autonomous phase has a manual override that takes precedence over all autonomous procedures | RFP req. 5; DR-1 UAS-17, UAS-GNCC-11 |
| R3 | Relay marker coordinates (target state + uncertainty, shared frame) to the rover **or base station** | RFP req. 3; DR-1 ConOps step 6 |
| R4 | Wait for confirmation of coordinate transfer before returning | DR-1 slide 9 step 6 |
| R5 | Enforce ≤ 2 m AGL and ≤ 0.5 m/s in every phase (sequencer passes limits; guidance enforces) | RFP req. 4; DR-1 UAS-14/15 |
| R6 | Autonomous search procedure | DR-1 UAS-13, UAS-GNCC-14 |
| R7 | FC must keep flying if the companion computer fails (sequencer runs on Pi 5 ⇒ FC owns failsafe) | DR-1 slides 112–113 (decoupled data flow) |

## Inconsistencies found in this step (not resolved — need your call)
1. **Takeoff mode.** Faculty Lecture 1 mission profile labels takeoff as *manual control*;
   DR-1 slides 8–9 label phases as *Autonomous, Manual Override*. Sequencer design differs
   (autonomous takeoff state vs. "wait for pilot hand-off at hover").
2. **Relay recipient.** RFP allows rover *or base station*; DR-1 says rover only.
3. **M0 not done.** M1 is mostly logic and needs only mode/event definitions, but bus
   definitions (M0) are needed before the Simulink wrapper (M1.3).

## Proposed M1 step plan (each step ends with STOP)
| Step | Deliverable | Verification idea |
|---|---|---|
| M1.0 | Project skeleton, param file, this scope/trace doc | Structure test (`tests/test_M1_0_structure.m`) |
| M1.1 | State + event table, state diagram, transition table (incl. override, abort, RTL) | Every R1–R7 maps to ≥1 state/transition; no orphan states |
| M1.2 | `seq_step.m` — pure codegen MATLAB function `[mode, out] = f(mode, events, GN)` | Unit tests: each transition fires exactly as tabled; override wins from every state |
| M1.3 | Simulink wrapper (MATLAB Function block or Stateflow — to decide) with bus I/O | `codegen`/`slbuild` check passes; harness reproduces M1.2 results |
| M1.4 | Scenario sims: nominal, override in each phase, marker not found, ack lost | Mode-vs-time plots; pass criteria per scenario |

## Open items created in M1.0
- `GN.seq.rate_hz` — TBD (G&N + Avionics)
- `GN.seq.search_timeout_s` — TBD (G&N), depends on M3 + endurance budget
- `GN.seq.relay_ack_timeout_s` — TBD (G&N + rover comms)
- `GN.seq.wp_reached_radius_m` — TBD (G&N + D&C), tied to UAS-GNCC-10
- How the Pi 5 *detects* a pilot override on the Cube Orange+ (flight-mode change via
  MAVLink?) depends on firmware (ArduPilot vs PX4) — TBD (Avionics). Working instr. §7 item 10.
