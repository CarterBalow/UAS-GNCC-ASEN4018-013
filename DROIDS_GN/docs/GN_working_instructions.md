# G&N Working Instructions — Team D.R.O.I.D.S. UAS (ASEN 4018-013)

**Owner role:** Guidance & Navigation (G&N) for the UAS multirotor.
**Close partner:** Dynamics & Control (D&C).
**Purpose of this file:** standing rules for every future G&N work session in this project. Read this first, every session.

---

## 1. How every work session must run

1. **One step at a time, with a check-in gate.** Each task is broken into small steps. After each step, stop, summarize what was done, list what was assumed, and wait for approval before the next step. Do not chain steps without approval.
2. **Do not make anything up.**
   - Every parameter value must come from: the RFP, the DR-1 deck, a team member, a datasheet, or a cited reference. If none exists, write it as `TBD` with an owner, never a guessed number.
   - If an illustrative value is needed to run a simulation, label it `PLACEHOLDER` in code and text, state why that value was picked, and add it to the open-items list (Section 7).
   - If unsure whether something is true, say so explicitly.
3. **Source the math.** Any equation that isn't elementary algebra/calculus gets a citation (author, title, year, and equation/section number when available). If a source cannot be verified in-session, say "unverified" rather than citing from memory.
4. **Describe every step two ways:**
   - **Physically** — what the vehicle/sensor is actually doing and why it matters for the mission.
   - **Mathematically** — the equations, variable definitions, frames, units, and assumptions.
5. **Diagrams when they help:** block diagrams for data flow and interfaces, geometry sketches for frames and search patterns, plots for every simulation result. Every diagram states its frame and units.
6. **Verification on every step.** Each step ends with a check that could fail: a unit test, an analytic comparison, a limit check against requirements (2 m altitude, 0.5 m/s speed, 5 m × 5 m AOI, 30 cm marker accuracy), or a plot inspection with stated pass criteria.

### Standard step template
```
Step N — <title>
Objective:        what this step produces
Physical picture: plain-language description
Math:             equations + definitions + frame + units + source
Assumptions:      numbered list (each one flagged if it's a placeholder)
Implementation:   MATLAB/Simulink files/blocks touched
Verification:     test + pass criteria + result
D&C interface:    what changes for the controller (or "none")
Open items:       new TBDs created
→ STOP for approval
```

---

## 2. Mission context (sourced — do not alter without new source)

Source: ASEN 4018-013/014 AY26-27 RFP v2; DR-1 deck (9/29–10/01/2026).

| Item | Value | Source |
|---|---|---|
| Mission | Take off from Point A, transit to AOI, search for ArUco marker, localize it, relay coordinates to rover/base, return to A, land | RFP UAS-3; DR-1 ConOps slides 8–9 |
| Marker localization accuracy | within 30 cm radius | RFP UAS-3; UAS-10 |
| Max altitude (test/demo) | 2 m AGL | RFP UAS-4; UAS-14 |
| Max speed (test/demo) | 0.5 m/s | RFP UAS-4; UAS-15 |
| Area of interest | ~5 m × 5 m, indoor or outdoor | RFP Definitions 1 |
| Endurance | ≥ 15 min | RFP UAS-2; UAS-7 |
| Manual override | Required for every autonomous phase; takes precedence | RFP UAS-5; UAS-17; UAS-GNCC-11 |
| Visual line of sight | Required | RFP UAS-4; UAS-16 |
| Relay content | Target state + associated uncertainty in shared frame | DR-1 ConOps step 6 |

G&N-relevant derived requirements (from DR-1, several still TBD): UAS-GNCC-3 (altitude accuracy ±TBD), UAS-GNCC-4 (GPS position accuracy ±TBD cm), UAS-GNCC-10 (path execution accuracy ±TBD cm, 3 DOF), UAS-GNCC-12 (compute & execute geometric flight paths), UAS-GNCC-12.4 (path compute time TBD s), UAS-GNCC-14 (search procedure).

---

## 3. Current design baseline (from DR-1 trades — treat as baseline, not final)

| Area | Baseline | Owner |
|---|---|---|
| Waypoint guidance | Spline (PCHIP interpolation was used in the trade sim) | G&N |
| Search pattern | Serpentine | G&N |
| Navigation sensor | GNSS-RTK; also expecting IMU, altimeter, camera | G&N / Avionics |
| State estimation | EKF or UKF (slide 64 says UKF; slide 90 says EKF or UKF) | D&C / G&N |
| Control law | Cascaded PID (outer + inner loops); underdamped tuning | D&C |
| Control input | Direct thrust / RPM | D&C |
| Dynamics model | Nonlinear 6-DOF, linearized about hover; Hoffmann et al. (2007) | D&C |
| Flight controller / companion | Pixhawk 6C + NVIDIA Jetson Nano (anticipated) | Avionics |
| Data flow | Decoupled: FC can fly without companion computer | Avionics |
| Comms | Wi-Fi | Avionics |
| Toolchain | MATLAB/Simulink with code generation (deck lists a tie with embedded C++) | GNC |
| Existing sim | Simulink, 100 Hz, 3-D NED, first-order vehicle lag, GPS noise | G&N |

---

## 4. Interface contract with Dynamics & Control

Goal: anything G&N builds now must drop into D&C's cascaded PID / 6-DOF model later with no rework.

```
 ┌──────────────┐  waypoints   ┌──────────────┐  reference r(t)   ┌──────────────┐
 │  MISSION     │────────────▶ │  GUIDANCE    │─────────────────▶ │  CONTROL     │
 │  SEQUENCER   │              │ (path / traj │  p,v,a,ψ (NED)    │  (D&C:       │
 │ (ConOps FSM) │◀── events ── │  generator)  │                   │  cascaded PID│
 └──────▲───────┘              └──────▲───────┘                   └──────┬───────┘
        │ mode, override               │ x̂ (estimated state)             │ thrust / RPM
        │                              │                                 ▼
 ┌──────┴───────┐   x̂, P       ┌──────┴───────┐   measurements    ┌──────────────┐
 │  MARKER      │◀──────────── │ NAVIGATION   │◀───────────────── │  VEHICLE     │
 │  LOCALIZER   │   camera     │ (GNSS-RTK +  │  GNSS, IMU, alt   │  DYNAMICS    │
 │  (ArUco→NED) │              │  IMU fusion) │                   │ (placeholder │
 └──────────────┘              └──────────────┘                   │  → 6-DOF)    │
                                                                  └──────────────┘
```

**Rules:**
- **Frame:** local NED, origin at Point A (to be confirmed with rover team for the shared frame). Body frame per D&C's convention (to be confirmed — Hoffmann 2007 convention unless D&C says otherwise).
- **Units:** SI only (m, s, rad). Angles in rad internally.
- **Guidance output (reference bus)**, time-stamped, all in NED:
  - `p_ref` [m] (3×1), `v_ref` [m/s] (3×1), `a_ref` [m/s²] (3×1), `psi_ref` [rad], `psi_dot_ref` [rad/s]
  - Rationale: a cascaded position→velocity→attitude loop can use `p_ref`, `v_ref` as setpoints and `a_ref` as feedforward. For quadrotors, position + yaw (and their derivatives) are sufficient to define the full reference state — Mellinger & Kumar (2011), differential flatness. This is why higher-derivative continuity of the path matters (see open item on PCHIP).
- **Navigation output (state bus):** `p_hat`, `v_hat`, attitude estimate, covariance `P`, validity flags.
- Every signal on a bus has: name, frame, units, rate, and source block documented.
- **Swappable blocks:** guidance never calls the dynamics directly; it only reads `x̂`. Placeholder dynamics and placeholder control live in clearly labeled subsystems that D&C replaces.

---

## 5. Placeholder dynamics & control (until D&C's models are integrated)

Use the simplest models that still exercise the guidance, in this order:

1. **Kinematic point (path check only):** ṗ = v_cmd. Verifies geometry, speed limits, coverage.
2. **Point-mass double integrator:** ṗ = v, v̇ = u, with |v| ≤ 0.5 m/s enforced by guidance (not hidden in the plant). Translational dynamics of a hovering multirotor reduce to this when attitude dynamics are assumed fast — to be sourced (e.g., Beard & McLain, *Small Unmanned Aircraft*, 2012) at the step where it's used.
3. **First-order velocity lag:** v̇ = (v_cmd − v)/τ, matching the existing Simulink init. τ = PLACEHOLDER until D&C gives a value.
4. **Placeholder control law:** PD on position error with acceleration feedforward, u = a_ref + K_d (v_ref − v) + K_p (p_ref − p). Gains chosen from a stated ζ, ω_n and marked PLACEHOLDER.

Each placeholder is labeled in Simulink and replaced by D&C's cascaded PID + 6-DOF model at integration.

---

## 6. Task breakdown (to be done slowly, one module at a time)

Order is a proposal; confirm before starting each.

| # | Module | Output | Key dependencies |
|---|---|---|---|
| M0 | Frames, units, conventions | Frame definitions doc + diagram, bus definitions | D&C body-frame convention; rover shared frame |
| M1 | Mission sequencer (state machine) | FSM covering ConOps steps 2–7 incl. override & abort | Avionics override architecture |
| M2 | Transit path (A → AOI entry) | Waypoints → spline reference p,v,a,ψ | M0 |
| M3 | Search pattern (serpentine) | Lane spacing, turn handling, coverage check | Camera FOV/resolution (TBD), altitude |
| M4 | GNSS-RTK integration | Geodetic → ECEF → local NED; RTK accuracy model | Receiver model (TBD) |
| M5 | Navigation filter | EKF/UKF fusing GNSS-RTK, IMU, altimeter | D&C sensor-fusion trade, sensor specs |
| M6 | Marker localization | ArUco detection → camera pose → NED marker position + uncertainty | Camera intrinsics, mount geometry |
| M7 | Return-to-launch & landing | Path back to A, descent profile | M2, M5 |
| M8 | Integration & SIL | Full loop with D&C models; requirement verification | All |

---

## 7. Open items / inconsistencies found in current docs

Flag these; do not resolve them by assumption.

1. **Mass requirement mismatch:** RFP = 25 lb (11.3 kg); UAS-9 = 12.5 lb (5.6 kg); slide 49 UAS-VSYS-6 = 25 lb. Affects D&C hover thrust, not paths directly.
2. **Existing Simulink init uses 5 m altitude and 2 m/s** — exceeds the 2 m / 0.5 m/s limits (noted on slide 95). Must be fixed before any sim is used for verification.
3. **Filter choice:** UKF (slide 64) vs. EKF or UKF (slide 90).
4. **PCHIP smoothness:** PCHIP is a piecewise cubic Hermite interpolant that is C¹ (continuous first derivative), not C² (Fritsch & Carlson, 1980). The acceleration reference would be discontinuous at knots. Check whether D&C needs a continuous `a_ref`; if so, compare against a C² spline or minimum-snap trajectory (Mellinger & Kumar, 2011).
5. **Waypoint trade sim speed:** 0.4 m/s average with 0.5 m/s limit — confirm margin policy with D&C (overshoot of an underdamped controller can exceed 0.5 m/s).
6. **Shared frame with rover:** origin, axes, and whether RTK base gives a common reference — unresolved.
7. **Indoor vs. outdoor testing:** GNSS-RTK will not work indoors; if testing is indoors, a different position source is needed. Unresolved.
8. **TBDs:** UAS-GNCC-3, -4, -10, -12.4 numerical values; camera specs; RTK receiver; D&C time constants and gains.
9. **Software architecture:** Simulink + code gen vs. embedded C++ was a tie; project description says Simulink with embedded translators. Confirm.

---

## 8. Code & model conventions

- MATLAB/Simulink, written to be code-generation compatible (`%#codegen`; fixed-size arrays; no dynamic memory unless confirmed supported by the target toolchain).
- Variable naming includes frame and units where ambiguous: `p_ned_m`, `v_ned_mps`, `psi_rad`.
- All tunable/physical parameters live in one parameter script/data dictionary, each with a source comment or `PLACEHOLDER`/`TBD` tag.
- Each module has its own test script that reproduces its verification plot.

---

## 9. Reference list (verify each before relying on it in a step)

- RFP: ASEN 4018-013/014 AY26-27 RFP v2 (project file).
- DR-1 deck: Team D.R.O.I.D.S., DR-1-UAS-FINAL (project file).
- Hoffmann, Huang, Waslander, Tomlin (2007), *Quadrotor Helicopter Flight Dynamics and Control: Theory and Experiment*, AIAA GNC, AIAA 2007-6461 — D&C's dynamics reference.
- Mellinger & Kumar (2011), *Minimum Snap Trajectory Generation and Control for Quadrotors*, IEEE ICRA — differential flatness, smooth trajectories.
- Fritsch & Carlson (1980), *Monotone Piecewise Cubic Interpolation*, SIAM J. Numer. Anal. — PCHIP.
- Beard & McLain (2012), *Small Unmanned Aircraft: Theory and Practice*, Princeton UP — frames, guidance, path following.
- Groves (2013), *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*, 2nd ed. — GNSS, frame transforms, fusion.
- Garrido-Jurado et al. (2014), *Automatic generation and detection of highly reliable fiducial markers under occlusion*, Pattern Recognition — ArUco.
- Choset (2001), *Coverage for robotics – A survey of recent results*, Annals of Math. and AI — coverage / boustrophedon (serpentine) paths.
