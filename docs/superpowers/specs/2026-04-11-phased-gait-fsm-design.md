# Phased Gait FSM Design
**Date:** 2026-04-11
**Status:** Approved — pending implementation

---

## Problem Statement

The current `gait_planner.py` uses a single continuous phi-arc with no explicit gait phases. When `--walk` is passed, the robot launches off the ground within ~1 second and the step watchdog fires `EMERGENCY_STOP` at t=3.0s. Root causes:

1. **WBC fires at full IK targets with no stance leg command** — `ik_left_angles` and `ik_right_angles` default to `(0,0,0)`. WBC_KP=200 N·m/rad injects up to 200 × θ_error torque per joint, launching the robot.
2. **No COM shift before toe-off** — gait planner starts swinging immediately without first shifting COM over the stance foot. The robot lifts a leg it is still loading.
3. **No contact confirmation at step completion** — `phi >= 1.0` flips the swing side unconditionally, regardless of whether the foot touched ground.
4. **Stance leg IK never updated during swing** — stance joint targets freeze at last swing command; robot sags as COM shifts.

---

## Solution: 5-State Step Phase FSM

Replace the phi scalar as the sole gait signal with an explicit `StepPhase` enum. The phi arc is retained but confined to the `SWING` phase only.

```
DOUBLE_SUPPORT
    ↓ (both feet confirmed + timer)
COM_SHIFT
    ↓ (CP over stance foot)
LIFT
    ↓ (swing foot unloaded + settled)
SWING   ← phi lives here only
    ↓ (phi ≥ 0.85)
PLACE
    ↓ (contact confirmed + settled)
DOUBLE_SUPPORT (legs swap)
```

---

## Architecture

### Module Ownership

| Concern | Owner |
|---|---|
| Phase FSM — advance transitions | `gait_planner.py` exclusively |
| Phase visibility | `shared_state.step_phase` (all modules read) |
| GRF gating | `grf.py` reads `step_phase` |
| Step count | `mission.py` reads `step_count` (write site moves to PLACE→DS transition) |
| Recovery | `recovery.py` step watchdog resets per phase transition |

### Execution Order (unchanged)
```
ActiveBalance → GRF → GaitPlanner(FSM) → Mission → WBC → Emergency → Recovery → apply_control
```

Phase transitions happen inside GaitPlanner so all downstream modules see the new phase within the same cycle.

---

## New `shared_state.py` Additions

### Enum
```python
class StepPhase(Enum):
    DOUBLE_SUPPORT = auto()
    COM_SHIFT      = auto()
    LIFT           = auto()
    SWING          = auto()
    PLACE          = auto()
```

### New fields on `Siclo1State`

| Field | Type | Default | Purpose |
|---|---|---|---|
| `step_phase` | `StepPhase` | `DOUBLE_SUPPORT` | Current FSM phase — readable by all modules |
| `step_phase_timer` | `float` | `0.0` | Seconds spent in current phase; reset on every transition |
| `stance_side` | `str` | `"right"` | Foot currently bearing weight (complement of `active_swing_side`) |
| `stance_foot_world_pos` | `np.ndarray` | `zeros(3)` | Stance foot position locked in world frame at stance entry; IK anchor for all phases |

### New error code
```python
ERR_PHASE_TIMEOUT = 6   # phase timeout fired (hot-path safe, no string allocation)
```

---

## Phase Definitions

### DOUBLE_SUPPORT

| Attribute | Value |
|---|---|
| **Active behavior** | Run full `active_balance` loop on both legs. Both legs IK computed toward nominal stance every cycle (not frozen). Lock `stance_foot_world_pos` for the incoming stance foot. |
| **Exit condition** | Both feet `CONTACT_CONFIRMED` AND `step_phase_timer ≥ DS_MIN_TIME` (0.10 s) |
| **Timeout** | 2.0 s → `freeze_robot = True` |
| **Notes** | Entry point at spawn and after every PLACE→DS transition. `stance_foot_world_pos` written once on entry — not updated again until next DS entry. |

### COM_SHIFT

| Attribute | Value |
|---|---|
| **Active behavior** | Active balance runs normally. Both legs hold nominal IK anchored to `stance_foot_world_pos`. Stance foot must not drift. |
| **Exit condition** | `abs(capture_point_x − stance_foot_x) < COM_SHIFT_THRESHOLD` (0.03 m) AND `stability_status != UNSTABLE` |
| **Timeout (conditional)** | 1.0 s → if `swing_foot_force > UNLOAD_FORCE_THRESHOLD`: abort → DOUBLE_SUPPORT; else proceed → LIFT |
| **Notes** | `capture_point_x` only evaluated when `stability_status != UNSTABLE`; if UNSTABLE, exit is blocked until stability recovers or timeout. |

### LIFT

| Attribute | Value |
|---|---|
| **Active behavior** | Snapshot `swing_foot_x_stance`. GRF active on stance leg only. Stance IK anchored to `stance_foot_world_pos`. |
| **Exit condition** | `swing_foot_force < UNLOAD_FORCE_THRESHOLD` (5.0 N) AND `abs(swing_foot_velocity_z) < SETTLE_VEL_THRESHOLD` (0.05 m/s) |
| **Timeout (conditional)** | 0.15 s → if `swing_foot_force > UNLOAD_FORCE_THRESHOLD`: abort → DOUBLE_SUPPORT; else proceed → SWING |
| **Notes** | `swing_foot_x_stance` snapshot occurs exactly once on LIFT entry, not each cycle. |

### SWING

| Attribute | Value |
|---|---|
| **Active behavior** | phi advances each cycle. IK computed for swing leg. Stance IK anchored to `stance_foot_world_pos`. GRF on stance only. |
| **Exit condition** | `phi ≥ PLACE_ENTRY_PHI` (0.85) |
| **Timeout** | `SWING_DURATION × 1.5` = 0.6 s → force transition to PLACE; log `ERR_PHASE_TIMEOUT` |
| **Notes** | phi is the only thing that changes here. No phase can reset phi mid-swing. |

### PLACE

| Attribute | Value |
|---|---|
| **Active behavior** | phi continues to 1.0 then clamps. Swing IK drives to final foot target. Stance leg anchored. GRF on stance only. |
| **Exit condition** | `swing_foot_contact == CONTACT_CONFIRMED` AND `abs(swing_foot_velocity_z) < SETTLE_VEL_THRESHOLD` (0.05 m/s) |
| **Timeout (conditional)** | 0.5 s → if `swing_foot_force > UNLOAD_FORCE_THRESHOLD`: enter DOUBLE_SUPPORT (contact happened, sensor was slow); else `freeze_robot = True` (foot missed ground) |
| **On clean exit** | Flip `active_swing_side` / `stance_side`. Reset `phi = 0.0`. Increment `step_count`. Lock new `stance_foot_world_pos`. Enter DOUBLE_SUPPORT. |

---

## Stance Leg IK Anchor Mechanism

Runs every cycle, all phases, for the stance leg:

```python
stance_foot_rel = shared_state.stance_foot_world_pos - stance_hip_pos
ik_stance_angles = kinematics.solve_ik(stance_foot_rel, stance_side)
# written to shared_state.ik_left_angles or ik_right_angles
```

During DOUBLE_SUPPORT and COM_SHIFT, both legs use this mechanism (each foot is its own anchor until swing side is selected).

`stance_foot_world_pos` is written **once** at DOUBLE_SUPPORT entry — never updated mid-phase.

---

## Abort Chain Rule

Any conditional timeout that aborts re-enters DOUBLE_SUPPORT with:
- `swing_phase = 0.0`
- `step_phase_timer = 0.0`
- `active_swing_side` and `stance_side` **unchanged** (same leg retries as swing)
- `stance_foot_world_pos` **unchanged** (anchor survives abort)

The robot can retry the same step after re-stabilizing.

---

## Data Flow Per Phase

| Phase | ActiveBalance | GRF | GaitPlanner | WBC |
|---|---|---|---|---|
| DOUBLE_SUPPORT | Full run, writes `capture_point` | Both legs | Lock anchor, IK both legs | Drives both |
| COM_SHIFT | Full run | Both legs | Hold IK anchored | Drives both |
| LIFT | Full run | Stance only | Snapshot x_stance, IK anchored | Drives both |
| SWING | Full run | Stance only | Advance phi, swing IK | Drives both |
| PLACE | Full run | Stance only | Clamp phi, swing IK to target | Drives both |

---

## Hard Safety Constraints

1. `freeze_robot = True` blocks all phase advances — no exceptions.
2. `stability_status == UNSTABLE` blocks COM_SHIFT exit condition (timer still runs toward timeout).
3. SWING → PLACE is one-way: phi is never reset mid-swing.
4. Both legs always get IK computed every cycle — stance leg is never frozen.
5. `stance_foot_world_pos` written exactly once per stance entry.
6. `swing_foot_velocity_z` gate applies to both LIFT exit and PLACE exit.
7. `step_count` incremented only at confirmed PLACE → DOUBLE_SUPPORT transition.

---

## `step_count` Ownership Change

| | Current | New |
|---|---|---|
| Write site | `phi >= 1.0` in gait_planner | `PLACE → DOUBLE_SUPPORT` transition |
| Condition | Unconditional | Only on clean exit (contact confirmed + settled) |
| `mission.py` | Unchanged — reads `step_count` | Unchanged |

---

## Timeout Constants

| Constant | Value | Units |
|---|---|---|
| `DS_MIN_TIME` | 0.10 | s |
| `DS_TIMEOUT` | 2.0 | s |
| `COM_SHIFT_TIMEOUT` | 1.0 | s |
| `LIFT_TIMEOUT` | 0.15 | s |
| `SWING_TIMEOUT_FACTOR` | 1.5 | × SWING_DURATION |
| `PLACE_TIMEOUT` | 0.5 | s |
| `COM_SHIFT_THRESHOLD` | 0.03 | m |
| `UNLOAD_FORCE_THRESHOLD` | 5.0 | N |
| `SETTLE_VEL_THRESHOLD` | 0.05 | m/s |
| `PLACE_ENTRY_PHI` | 0.85 | dimensionless |

---

## Testing Plan

| File | Coverage |
|---|---|
| `test_step_phase_transitions.py` | All 5→5 nominal transitions; `step_phase_timer` resets on every transition |
| `test_step_phase_guards.py` | Each hard safety constraint blocks the correct transition; `freeze_robot` blocks all |
| `test_step_phase_timeouts.py` | Each timeout fires at correct time; conditional routing by `swing_foot_force` |
| `test_stance_anchor.py` | `stance_foot_world_pos` locked once per phase; IK recomputed every cycle; no mid-phase drift |
| `test_velocity_gate.py` | LIFT and PLACE exits blocked when `swing_foot_velocity_z > SETTLE_VEL_THRESHOLD` |
| `test_gait_planner_fsm.py` | Full step cycle integration; `step_count` increments once per cycle; `active_swing_side` flips once |

---

## Key Points

**KEY POINT:** The 5-phase FSM converts implicit timing assumptions into explicit physical gates — no step can advance until the robot has physically satisfied the condition for that phase.

**KEY LINE:** `stance_foot_world_pos` written once at DOUBLE_SUPPORT entry — all stance IK anchors to this frozen world position throughout the step.
