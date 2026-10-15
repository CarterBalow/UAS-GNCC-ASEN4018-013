# Team D.R.O.I.D.S. UAS — Guidance & Navigation (G&N)

ASEN 4018-013, AY26-27. Owner: G&N. Close partner: Dynamics & Control (D&C).
Standing rules: `docs/GN_working_instructions.md` in the class project (read first, every session).

## Quick start (MATLAB)
```matlab
cd DROIDS_GN
gn_startup              % adds paths, loads parameters into workspace struct GN
runtests('tests')       % runs every module's verification tests
```

## Folder layout
| Folder | Contents | Who edits |
|---|---|---|
| `docs/` | Design notes, one file per module (`docs/modules/Mx_*.md`), decision log | G&N |
| `params/` | `gn_params.m` — the ONLY place numbers live. Each value tagged with source, `TBD (owner)` or `PLACEHOLDER (reason)` | G&N |
| `src/common/` | Shared code: bus definitions, frame/unit helpers (filled in M0) | G&N |
| `src/M1_sequencer/` | Mission sequencer (ConOps state machine) | G&N |
| `src/M2_.../` etc. | One folder per module as it starts | G&N |
| `models/` | Simulink models (`.slx`) and test harnesses | G&N |
| `placeholders/dynamics/` | **PLACEHOLDER** plant models — replaced by D&C's 6-DOF model | D&C replaces |
| `placeholders/control/` | **PLACEHOLDER** control laws — replaced by D&C's cascaded PID | D&C replaces |
| `tests/` | `test_<module>_*.m` — each reproduces a step's verification + pass criteria | G&N |
| `results/` | Generated plots/logs (not hand-edited; safe to delete) | auto |

## Conventions (summary — full list in working instructions §4, §8)
- Frame: local NED, origin at Point A (shared rover frame TBD). SI units, angles in rad.
- Names carry frame + units: `p_ned_m`, `v_ned_mps`, `psi_rad`, `t_s`.
- Guidance → D&C reference bus: `p_ref`, `v_ref`, `a_ref` (NED), `psi_ref`, `psi_dot_ref`.
- All `src/` code is codegen-compatible: `%#codegen`, fixed-size arrays, no dynamic memory.
- Guidance never calls dynamics; it reads only the estimated state `x_hat`.

## Module status
| # | Module | Status |
|---|---|---|
| M0 | Frames, units, buses | Not started (deferred — see M1 doc, open item) |
| M1 | Mission sequencer | Step M1.0 (skeleton + scope) — awaiting approval |
| M2–M8 | see working instructions §6 | Not started |
