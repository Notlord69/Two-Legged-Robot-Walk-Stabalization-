# Capture Point Stability + Step Timer Reset — Design Spec

**Date:** 2026-04-10
**Status:** Approved
**Session trigger:** Diagnostic trace of session `2026-04-10_11-31-26` identified two root causes behind the 302-cycle EMERGENCY_STOP.

---

## Problem Statement

Two independent bugs caused the robot to terminate at cycle 302 via Recovery Priority 1 ("Unstable timeout — critical failure"):

### Bug 1 — Step timer never resets (Logic gap)

`gait_planner.py:158` increments `step_count` at touchdown (`φ ≥ 1.0`) but never calls `recovery.reset_step()`. `current_step_start_time` stays `0.0` for the entire run. `step_duration = sim_time − 0.0` grows monotonically and crosses the 3.0 s `timeout_threshold` at cycle 302 (recovery sees `sim_time = 3.01 s` before the step-15 increment).

### Bug 2 — Stability permanently UNSTABLE (Incorrect computation)

`stability.py` checks whether the **static COM** is inside the support polygon. During walking the robot is frequently in single-support or transitional contact, making the COM leave the shrunk polygon continuously. `stability_status = UNSTABLE` in 301/302 cycles (including cycles where contact was confirmed), keeping `is_unstable = True` and arming Priority 1 permanently.

**Combined effect:** `is_unstable = True` + `step_duration > 3.0 s` at cycle 302 → `EMERGENCY_STOP` → `freeze_robot = True` → next `step()` call returns `False` at the freeze gate → run halts.

**Goal:** The robot must be able to stand still in IDLE indefinitely without triggering any recovery flags, while still responding immediately to a genuine fall (feet leave the ground → UNSTABLE → Priority 1 fires).

---

## Approach Selected

**Approach A — Surgical: Fix Stability + Wire Step Timer.**

Fix the underlying computation (stability) and the missing wiring (step timer). No guards added to `recovery.py`. Recovery stays sensitive to actual falls because a genuine fall causes `is_unstable = True`, which arms Priority 1 correctly.

---

## Architecture

Three files change. No new modules. No changes to `recovery.py`, `HeartBeat.py`, or `mission.py`.

| File | Change summary |
|---|---|
| `shared_state.py` | Add `capture_point: np.ndarray` field (2D X-Y, world frame) |
| `stability.py` | Replace static COM check with Capture Point (LIPM Option A); write `shared_state.capture_point` every cycle |
| `gait_planner.py` | Import `recovery`; call `recovery.reset_step()` at step touchdown (φ ≥ 1.0) |

### Data flow after fix

```
stability.check_stability()  [100 Hz]
    compute com, com_vel
    ω_n = √(g / z_com)                            # LIPM natural frequency
    capture_point = com_xy + com_vel_xy / ω_n      # 2D extrapolated COM
    shared_state.capture_point = capture_point      # written every cycle
    polygon.contains(capture_point)?               # replaces static com_2d check
    → STABLE / MARGINAL / UNSTABLE

gait_planner.update()  [100 Hz, φ ≥ 1.0 path only]
    step_count += 1
    recovery.reset_step()                          # new: current_step_start_time = sim_time
    swing_phase = 0.0
    active_swing_side flips
```

---

## Section 1 — `shared_state.py`

### New field in `__init__` (KINEMATICS STATE block)

```python
self.capture_point: np.ndarray = np.zeros(2)
# m, X-Y world-frame Capture Point (LIPM extrapolated COM).
# Written by stability.py every cycle; read by gait_planner.py for foot placement.
```

### Addition to `reset()`

```python
self.capture_point = np.zeros(2)
```

### Addition to `get_diagnostics()`

```python
'capture_point': self.capture_point.tolist(),
```

**Rationale:** `gait_planner.py:125` already reads this field via `getattr(..., np.zeros(2))`. Formalising the field converts a silent fallback (invisible wrong value) into explicit initialised state that appears in diagnostics.

---

## Section 2 — `stability.py`

### New constants (module level)

```python
G: float = 9.81          # m/s², standard gravitational acceleration
Z_COM_MIN: float = 0.05  # m, floor guard for ω_n — prevents sqrt(g/0) domain error
```

### `check_stability()` — replace static check with Capture Point

**Current logic (lines 291–304):**
```python
com_2d = Point(com[0], com[1])
if polygon.contains(com_2d):
    ...
```

**New logic:**
```python
# Capture Point — LIPM extrapolated COM (Option A)
z_com   = max(com[2], Z_COM_MIN)         # guard: robot at or below floor
omega_n = math.sqrt(G / z_com)           # rad/s, LIPM natural frequency
cp_xy   = com[:2] + com_vel[:2] / omega_n
shared_state.capture_point = cp_xy       # write for gait_planner and diagnostics

cp_point = Point(cp_xy[0], cp_xy[1])
if polygon.contains(cp_point):
    margin_distance = polygon.exterior.distance(cp_point)
    if margin_distance > safety_margin * 0.5:
        shared_state.set_stability_status(StabilityStatus.STABLE, margin=margin_distance)
        return StabilityStatus.STABLE
    else:
        shared_state.set_stability_status(StabilityStatus.MARGINAL, margin=margin_distance)
        return StabilityStatus.MARGINAL
else:
    margin_distance = -polygon.exterior.distance(cp_point)
    shared_state.set_stability_status(StabilityStatus.UNSTABLE, margin=margin_distance)
    return StabilityStatus.UNSTABLE
```

**Import addition:** `import math` at top of file.

### Behaviour at standing-still steady state

When `v_com_xy ≈ 0`, `cp_xy ≈ com_xy`. The check degrades gracefully to the static COM test. Both feet confirmed + static COM inside polygon → `STABLE` → `is_unstable = False` → Priority 1 disarmed.

### Behaviour on genuine fall

Feet lose contact → `get_confirmed_contact_points()` returns empty → polygon is `None` → `UNSTABLE` (line 288, unchanged). `is_unstable = True` → Priority 1 fires after `step_duration > 3.0 s`. Since `step_duration` is already large during a long standing session, Priority 1 fires on the very next cycle after instability is detected.

---

## Section 3 — `gait_planner.py`

### Import addition

```python
import recovery   # for reset_step() at touchdown
```

### Touchdown block (φ ≥ 1.0) — add one call

```python
# Step completion: φ ≥ 1.0
if phi >= 1.0:
    shared_state.step_count    += 1
    shared_state.swing_phase    = 0.0
    shared_state.active_swing_side = (
        "right" if side == "left" else "left"
    )
    recovery.reset_step()   # reset step timer: current_step_start_time = sim_time
```

**Why this call site:** `reset_step()` sets `current_step_start_time = sim_time` and clears `current_step_attempts`. Calling it at each touchdown gives Priority 1 a fresh 3 s window per step during walking. Any step exceeding 3 s (swing foot never lands) correctly triggers Priority 3 reposition.

**No circular import:** `recovery.py` imports only `shared_state`. `gait_planner.py` currently imports `kinematics` and `shared_state`. The new dependency is one-directional.

---

## Section 4 — Error Handling

| Scenario | Handling |
|---|---|
| `z_com < 0.05 m` (robot below floor or inverted) | `z_com = max(com[2], Z_COM_MIN)` before `sqrt` — prevents domain error; UNSTABLE is physically correct anyway |
| `com_vel = zeros` on cycle 1 (`prev_com` is `None`) | Existing `compute_com_velocity()` returns `np.zeros(3)` → CP = COM → correct |
| Velocity noise during standing | CP ≈ COM at v≈0; minor noise shifts CP by `noise / ω_n` (small, ω_n ≈ 3.3 rad/s at 0.88 m) |
| `recovery.reset_step()` called before first step | Idempotent — writes `sim_time` to `current_step_start_time`, safe at any point |

---

## Section 5 — Testing

### `test_gait_shared_state.py`

- Assert `capture_point` field exists and initialises to `np.zeros(2)`
- Assert `reset()` restores `capture_point` to `np.zeros(2)`

### `test_gait_planner.py`

- Add touchdown test: advance `swing_phase` past 1.0, mock `recovery.reset_step`, assert it was called exactly once
- Assert `shared_state.current_step_start_time == shared_state.sim_time` post-touchdown

### `test_stability_capture_point.py` (new file)

Three cases:

1. **Standing still — both feet CONFIRMED, zero velocity:**
   `v_com = [0,0,0]` → CP = COM → COM inside polygon → `STABLE`

2. **High lateral velocity — CP exits polygon:**
   `v_com_y = 2.0 m/s`, CP shifts ~0.6 m lateral → outside polygon → `UNSTABLE`

3. **No confirmed contacts — no polygon:**
   Both feet `NO_CONTACT` → polygon `None` → `UNSTABLE` (existing path, regression test)

### Existing tests — no breakage expected

Margin classification thresholds are unchanged. Existing tests that assert `STABLE` with `v_com = 0` still pass because CP = COM at zero velocity.

---

## Success Criteria

1. Robot stands in IDLE for >300 cycles: `stability_status = STABLE`, `emergency_stop_triggered = False`
2. Walking session: `current_step_start_time` resets at each touchdown; `step_duration` never exceeds `SWING_DURATION + margin` between touchdowns
3. Simulated fall (feet forced to `NO_CONTACT`): `UNSTABLE` within 1 cycle, Priority 1 fires within the next `timeout_threshold` window
4. All existing tests pass
5. 3 new capture-point tests pass
