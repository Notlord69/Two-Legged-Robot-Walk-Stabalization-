# Gait Stability Fixes — Design Spec

**Date:** 2026-04-12  
**Scope:** `gait_planner.py`, `test_step_phase_transitions.py`, `test_step_phase_guards.py`  
**Delivery:** Single atomic commit — all 4 fixes + test updates together

---

## Problem Summary

Three observed failures when walking is commanded:

| ID | Symptom | Root Cause (diagnosed) |
|----|---------|----------------------|
| A | Robot leans before walking | COM_SHIFT exit checks sagittal X only; condition trivially true at startup |
| B | LIFT starts before robot is ready | Same wrong-axis exit + gait planner runs at partial torque during RAMP |
| C | Stance leg loses ground contact | `UNLOAD_FORCE_THRESHOLD = 5 N` used for both swing (unload) and stance (load) — both near-meaningless |

One pre-walk readiness gap was also identified: no force balance check before DS→COM_SHIFT transition.

---

## Fix 1 — Dual-Threshold Weight Transfer Guard

### Problem
`UNLOAD_FORCE_THRESHOLD = 5 N` is used for both the swing foot ("is it empty?") and the stance foot ("is it loaded?"). For an 8 kg robot (~78 N total weight), 5 N is effectively zero on both sides. The LIFT guard cannot enforce that weight transfer actually completed.

### Design
Delete `UNLOAD_FORCE_THRESHOLD`. Replace with two named constants:

```python
SWING_UNLOAD_THRESHOLD: float = 5.0   # N, swing foot considered empty below this
STANCE_LOAD_THRESHOLD:  float = 60.0  # N, ~77% of 78 N body weight; stance must carry
                                       #    this before LIFT is permitted
```

**LIFT primary exit** — both conditions must hold simultaneously:
```python
if (swing_force  < SWING_UNLOAD_THRESHOLD and
        swing_vel_z < SETTLE_VEL_THRESHOLD and
        stance_force >= STANCE_LOAD_THRESHOLD):
    # weight transfer complete — proceed to SWING
```

**LIFT timeout abort** — abort if either side still failing:
```python
if timer >= LIFT_TIMEOUT:
    if (swing_force > SWING_UNLOAD_THRESHOLD or
            stance_force < STANCE_LOAD_THRESHOLD):
        self._abort_to_double_support()
    else:
        self._transition_to(StepPhase.SWING)
```

All other uses of the old `UNLOAD_FORCE_THRESHOLD` (COM_SHIFT timeout path, PLACE timeout path) are renamed to `SWING_UNLOAD_THRESHOLD` — those checks ask "does this foot have any force?" which matches the swing-side semantic.

### Tuning Risk
`LIFT_TIMEOUT = 0.15 s` (15 cycles). If the physics simulation cannot transfer ~21 N of additional load to the stance foot in 15 cycles, every LIFT will abort. First tuning target if abort loops appear: lower `STANCE_LOAD_THRESHOLD` to 50 N before touching `LIFT_TIMEOUT`.

---

## Fix 2 — COM_SHIFT Exit: 2D Euclidean Distance

### Problem
`_handle_com_shift` exits when:
```python
abs(cp_x - stance_x) < COM_SHIFT_THRESHOLD   # 1D sagittal (X only)
```
Both feet start at the same sagittal X. With the robot standing still, `cp_x ≈ 0` and `stance_x ≈ 0` → condition is trivially true on the first cycle. COM never shifts laterally before LIFT.

### Design
Replace with 2D Euclidean distance across both X and Y:

```python
cp_close = np.linalg.norm(
    shared_state.capture_point - shared_state.stance_foot_world_pos[:2]
) < COM_SHIFT_THRESHOLD
```

- `capture_point` is shape `(2,)` — [X, Y] world frame  
- `stance_foot_world_pos[:2]` slices the 3D anchor to [X, Y]  
- `COM_SHIFT_THRESHOLD = 0.03 m` — value unchanged; now means "CP within 3 cm of stance foot in any horizontal direction"

### Tuning Risk
`COM_SHIFT_THRESHOLD = 0.03 m` was sized for a 1D sagittal-only comparison. In 2D, if the actual lateral foot separation is large (>0.1 m), the COM must travel a significant distance before CP lands within 3 cm of the stance foot. If the robot consistently hits `COM_SHIFT_TIMEOUT = 1.0 s` and falls through to the timeout path, increase `COM_SHIFT_THRESHOLD` to 0.06–0.08 m.

---

## Fix 3 — Force Balance Gate in Double Support

### Problem
`_handle_double_support` advances to COM_SHIFT on `both_confirmed and timer >= DS_MIN_TIME` only. No check that forces are roughly equal. A robot standing with one foot carrying 3× the weight of the other will proceed to swing, guaranteeing collapse.

### Design
Add force balance check as the final gate before DS→COM_SHIFT:

```python
FORCE_BALANCE_RATIO: float = 2.0   # dimensionless, max allowed ratio max(F)/min(F)
FORCE_BALANCE_FLOOR: float = 10.0  # N, minimum per-foot force before ratio applies
```

Inline in `_handle_double_support` after `both_confirmed and timer >= DS_MIN_TIME`:

```python
lf = shared_state.left_foot_force
rf = shared_state.right_foot_force
if (lf >= FORCE_BALANCE_FLOOR and rf >= FORCE_BALANCE_FLOOR and
        max(lf, rf) / min(lf, rf) <= FORCE_BALANCE_RATIO):
    self._transition_to(StepPhase.COM_SHIFT)
```

`FORCE_BALANCE_FLOOR` prevents a 2 N / 2 N scenario (ratio = 1.0 passes, but the robot is nearly airborne) from satisfying the gate. Both feet must carry at least 10 N before the ratio is trusted.

No helper function is introduced — the check is used in exactly one place.

---

## Fix 4 — Soft Ramp Gate in Double Support

### Problem
The gait planner runs in RAMP state (partial torque). `DS_MIN_TIME = 0.10 s` elapses during the 0.5 s RAMP window. The robot can enter COM_SHIFT → LIFT → SWING while `ramp_gain < 1.0`, meaning joint torques are insufficient to maintain control during the step.

### Design
Soft gate: DS timer still counts during RAMP and the stance foot still locks. Only the DS→COM_SHIFT transition is blocked until full torque.

```python
# Soft ramp gate: timer counts but phase does not advance until full torque
if shared_state.ramp_gain < 1.0:
    return
```

Inserted in `_handle_double_support` **after** the `DS_TIMEOUT` check and **before** `both_confirmed`. This ensures:
1. The freeze-on-timeout path still fires if the robot is stuck too long in DS.
2. No phase advance happens at partial gain.
3. When RAMP completes (0.5 s), `timer` is already 0.5 s > `DS_MIN_TIME = 0.10 s`, so the robot proceeds immediately once force balance is also satisfied.

**Ordering within `_handle_double_support`:**
```
lock stance foot (once)
→ compute stance IK
→ check DS_TIMEOUT (freeze)
→ check ramp_gain < 1.0 (soft gate, return early)
→ check both_confirmed and timer >= DS_MIN_TIME
→ check force balance
→ _transition_to(COM_SHIFT)
```

---

## Test Changes

### File: `test_step_phase_transitions.py`

**1 existing test updated:**

| Test | Change |
|------|--------|
| `test_lift_advances_to_swing_when_unloaded_and_settled` | `right_foot_force`: 30.0 → 65.0 N (must exceed new `STANCE_LOAD_THRESHOLD = 60 N`) |

**4 new tests added:**

| Test | What it covers |
|------|---------------|
| `test_ds_blocked_when_ramp_gain_below_one` | Fix 4: DS timer past DS_MIN_TIME but ramp_gain=0.7 → stays in DS |
| `test_ds_blocked_when_force_imbalanced` | Fix 3: lf=5 N, rf=50 N → ratio=10× > 2× → stays in DS |
| `test_ds_blocked_when_force_below_floor` | Fix 3: lf=2 N, rf=2 N → ratio=1.0 passes but floor fails → stays in DS |
| `test_com_shift_blocked_when_cp_offset_laterally` | Fix 2: cp=[0.10, 0.10], stance=[0.10, 0.0, 0.0] → 2D dist=0.10 > 0.03 → stays in COM_SHIFT |

Existing COM_SHIFT tests (`test_com_shift_advances_to_lift_when_cp_close_and_stable`, `test_com_shift_blocked_when_cp_far`) use purely sagittal X offsets with Y=0 on both sides — their 2D distances match the old 1D results, so they pass without changes.

---

## What Is Not Changing

- `grf.py` — no changes
- `stability.py` — no changes
- `mission.py` — no changes (force balance gate lives in gait_planner DS handler, not mission IDLE handler)
- `LIFT_TIMEOUT`, `DS_MIN_TIME`, `DS_TIMEOUT`, `COM_SHIFT_TIMEOUT` — all unchanged; tuning targets post-simulation only
- `SWING_HEIGHT`, `SWING_DURATION`, `PLACE_ENTRY_PHI` — untouched

---

## Acceptance Criteria

1. All existing tests pass (1 updated, none deleted).
2. All 4 new tests pass.
3. `UNLOAD_FORCE_THRESHOLD` does not appear anywhere in `gait_planner.py`.
4. `_handle_com_shift` uses `np.linalg.norm(...)` not `abs(... - ...)`.
5. `_handle_double_support` gates on `ramp_gain >= 1.0` and force balance before advancing.
6. Every new constant has a unit and physical meaning in its comment (CLAUDE.md requirement).
