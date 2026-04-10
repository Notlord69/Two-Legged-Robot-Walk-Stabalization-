# Design: IDLE Step Timer Reset in `recovery.py`

**Date:** 2026-04-10
**Status:** Approved
**Scope:** Task 4 addition to `2026-04-10-capture-point-stability-step-timer` plan

---

## Problem

`RecoveryController.evaluate()` computes `step_duration = sim_time - current_step_start_time`. Both fields initialise to `0.0` at reset. In IDLE, `sim_time` advances every 100 Hz cycle but `current_step_start_time` is never updated (touchdown never fires). After 302 cycles (~3.02 s), `step_duration > 3.0 s` threshold is crossed. If `is_unstable` is also True (no confirmed contacts → UNSTABLE from stability.py), Priority 1 fires EMERGENCY_STOP.

Priority 3 already carries an `and mission_state != MissionState.IDLE` guard. Priority 1 does not.

---

## Goal

The robot must be able to stand in IDLE indefinitely without the step watchdog firing. The 3.0 s watchdog must remain fully active once a walk sequence begins.

---

## Architecture

Single change to `RecoveryController.evaluate()` in `recovery.py`. No new modules. No changes to `shared_state.py`, `gait_planner.py`, `HeartBeat.py`, or `recovery.py`'s public API.

---

## The Change

At the top of `RecoveryController.evaluate()`, before `get_step_duration()` is called:

```python
# Keep timer fresh in IDLE so timeout checks never fire while standing still.
# step_duration stays ~dt each cycle; Priorities 1, 3, 5 cannot threshold-trip.
# Slip (P4) and contact-loss (P2) checks still evaluate normally.
if shared_state.mission_state == MissionState.IDLE:
    shared_state.current_step_start_time = shared_state.sim_time
```

### Why this placement

`get_step_duration()` is called immediately after. Resetting the timer here means `step_duration` is always ~`last_dt` (~10 ms) during IDLE — below any threshold. Priorities 1, 3, and 5 (all timeout-based) cannot fire. Priorities 2 and 4 (contact-loss and slip — not timer-gated) continue to evaluate normally.

### IDLE → WALK transition

The last IDLE cycle sets `current_step_start_time = sim_time`. When `mission_state` flips to WALK, the first WALK cycle skips the reset and calls `get_step_duration()` → returns ~`last_dt`. The 3.0 s watchdog begins from zero. Clean start guaranteed.

---

## Files Changed

| File | Action |
|---|---|
| `recovery.py` | Add 2-line IDLE reset at top of `RecoveryController.evaluate()` |
| `test_recovery_idle_reset.py` | Create — two new pytest cases |

---

## Tests

**`test_no_emergency_stop_after_long_idle`**
- `shared_state.reset()`, set `mission_state = IDLE`, `is_unstable = True`
- Advance `sim_time` to 10.0 s (>> 3.0 s threshold), keep `current_step_start_time = 0.0`
- Call `update_recovery()`
- Assert `shared_state.recovery_action != RecoveryAction.EMERGENCY_STOP`
- Assert `shared_state.current_step_start_time ≈ 10.0` (timer was reset)

**`test_step_duration_near_zero_on_first_walk_cycle`**
- Long IDLE: `sim_time = 10.0`, `current_step_start_time = 0.0`, call `update_recovery()` (resets timer to 10.0)
- Advance `sim_time` by one dt (to 10.01), flip `mission_state = WALK`
- Call `update_recovery()` (no IDLE reset this cycle)
- Assert `shared_state.get_step_duration() < 0.1` (well under 3.0 s threshold)

---

## Success Criteria

1. `test_no_emergency_stop_after_long_idle` passes
2. `test_step_duration_near_zero_on_first_walk_cycle` passes
3. All pre-existing recovery tests pass (no regressions)
4. Robot can stand in IDLE for any duration without EMERGENCY_STOP
