# Gait Stability Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix three in-simulation robot failures (premature lean, early LIFT, stance foot loss) by applying four targeted changes to `gait_planner.py` as one atomic commit.

**Architecture:** All logic lives in `gait_planner.py`. Tests live in `test_step_phase_transitions.py`. No new files. No other modules touched. TDD: write failing tests first, implement, verify, commit.

**Tech Stack:** Python 3.10, pytest, numpy — no new dependencies.

---

## Task 1: Write 4 failing tests

**Files:**
- Modify: `test_step_phase_transitions.py`

The four new tests cover the four new behaviours. Each must FAIL before implementation (old code has no ramp gate, no force-balance gate, and uses 1-D X-only COM_SHIFT check).

- [ ] **Step 1: Add the four new test functions** — append to end of `test_step_phase_transitions.py`:

```python
# ── Fix 4: soft ramp gate ─────────────────────────────────────────────────────

def test_ds_blocked_when_ramp_gain_below_one():
    """DS must not advance to COM_SHIFT while torque is still ramping up."""
    _reset()
    shared_state.step_phase_timer = 0.5    # s, well past DS_MIN_TIME=0.10
    shared_state.ramp_gain        = 0.7    # dimensionless — still in RAMP
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


# ── Fix 3: force balance gate ─────────────────────────────────────────────────

def test_ds_blocked_when_force_imbalanced():
    """DS must not advance when one foot carries > 2× the other's force."""
    _reset()
    shared_state.step_phase_timer = 0.5
    shared_state.left_foot_force  = 15.0   # N — ratio = 60/15 = 4× > 2×
    shared_state.right_foot_force = 60.0   # N
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


def test_ds_blocked_when_force_below_floor():
    """DS must not advance when both feet are nearly unloaded (ratio passes but floor fails)."""
    _reset()
    shared_state.step_phase_timer = 0.5
    shared_state.left_foot_force  = 2.0    # N — below FORCE_BALANCE_FLOOR=10 N
    shared_state.right_foot_force = 2.0    # N
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.DOUBLE_SUPPORT


# ── Fix 2: 2-D Euclidean COM_SHIFT exit ──────────────────────────────────────

def test_com_shift_blocked_when_cp_offset_laterally():
    """COM_SHIFT must not exit when CP is close in X but far in Y."""
    _reset()
    shared_state.step_phase            = StepPhase.COM_SHIFT
    shared_state.stability_status      = StabilityStatus.STABLE
    shared_state.stance_foot_world_pos = np.array([0.10, 0.0, 0.0])
    # CP at same X as stance foot but 0.10 m off in Y → 2D dist=0.10 > threshold=0.03
    shared_state.capture_point         = np.array([0.10, 0.10])
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.COM_SHIFT
```

- [ ] **Step 2: Run the 4 new tests and confirm they all FAIL**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_step_phase_transitions.py::test_ds_blocked_when_ramp_gain_below_one \
       test_step_phase_transitions.py::test_ds_blocked_when_force_imbalanced \
       test_step_phase_transitions.py::test_ds_blocked_when_force_below_floor \
       test_step_phase_transitions.py::test_com_shift_blocked_when_cp_offset_laterally \
       -v
```

Expected: **4 FAILED**. If any pass, the test is wrong — stop and fix the test logic before continuing.

---

## Task 2: Implement Fix 1 — dual-threshold constants + rename all uses

**Files:**
- Modify: `gait_planner.py`

`UNLOAD_FORCE_THRESHOLD` is used in 6 places. This task replaces the constant definition and every use in one pass. The LIFT test (`test_lift_advances_to_swing_when_unloaded_and_settled`) will break after this task because `_reset()` sets `right_foot_force = 30.0` which no longer satisfies the new `STANCE_LOAD_THRESHOLD = 60 N`. That is expected — it is fixed in Task 3.

- [ ] **Step 1: Replace the constant block in the `# Physical thresholds` section**

Find (lines ~69-72):
```python
# Physical thresholds
UNLOAD_FORCE_THRESHOLD: float = 5.0    # N, foot considered unloaded below this
SETTLE_VEL_THRESHOLD:   float = 0.05   # m/s, foot considered settled below this
```

Replace with:
```python
# Physical thresholds
SWING_UNLOAD_THRESHOLD: float = 5.0    # N, swing foot considered empty below this
STANCE_LOAD_THRESHOLD:  float = 60.0   # N, ~77% of 78 N body weight; stance must carry
                                        #    this before LIFT is permitted
FORCE_BALANCE_RATIO:    float = 2.0    # dimensionless, max allowed ratio max(F)/min(F) at DS→COM_SHIFT
FORCE_BALANCE_FLOOR:    float = 10.0   # N, minimum per-foot force before ratio check applies
SETTLE_VEL_THRESHOLD:   float = 0.05   # m/s, foot considered settled below this
```

- [ ] **Step 2: Update `_handle_com_shift` timeout path (1 use)**

Find inside `_handle_com_shift`:
```python
        if swing_force > UNLOAD_FORCE_THRESHOLD:
```

Replace with:
```python
        if swing_force > SWING_UNLOAD_THRESHOLD:
```

- [ ] **Step 3: Update `_handle_lift` primary exit (2 uses)**

Find inside `_handle_lift`:
```python
        if (swing_force  < UNLOAD_FORCE_THRESHOLD and
                swing_vel_z < SETTLE_VEL_THRESHOLD and
                stance_force >= UNLOAD_FORCE_THRESHOLD):
```

Replace with:
```python
        if (swing_force  < SWING_UNLOAD_THRESHOLD and
                swing_vel_z < SETTLE_VEL_THRESHOLD and
                stance_force >= STANCE_LOAD_THRESHOLD):
```

- [ ] **Step 4: Update `_handle_lift` timeout abort (2 uses)**

Find inside `_handle_lift`:
```python
        if timer >= LIFT_TIMEOUT:
            # Abort if: swing foot still loaded, OR stance foot not bearing weight.
            if (swing_force  > UNLOAD_FORCE_THRESHOLD or
                    stance_force < UNLOAD_FORCE_THRESHOLD):
```

Replace with:
```python
        if timer >= LIFT_TIMEOUT:
            # Abort if: swing foot still loaded, OR stance foot not bearing weight.
            if (swing_force  > SWING_UNLOAD_THRESHOLD or
                    stance_force < STANCE_LOAD_THRESHOLD):
```

- [ ] **Step 5: Update `_handle_place` timeout path (1 use)**

Find inside `_handle_place`:
```python
            if self._swing_foot_force() > UNLOAD_FORCE_THRESHOLD:
```

Replace with:
```python
            if self._swing_foot_force() > SWING_UNLOAD_THRESHOLD:
```

- [ ] **Step 6: Confirm `UNLOAD_FORCE_THRESHOLD` is fully gone**

```bash
grep -n "UNLOAD_FORCE_THRESHOLD" /home/notlord/ros2_ws/Siclo1_V1/gait_planner.py
```

Expected: **no output**. If any line appears, fix it before continuing.

- [ ] **Step 7: Run the LIFT-related tests to confirm the expected breakage**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_step_phase_transitions.py -k "lift" -v
```

Expected: `test_lift_advances_to_swing_when_unloaded_and_settled` **FAILS** (right_foot_force=30.0 < STANCE_LOAD_THRESHOLD=60.0). All other LIFT tests should still pass.

---

## Task 3: Update broken LIFT test + implement Fix 2 (2-D COM_SHIFT exit)

**Files:**
- Modify: `test_step_phase_transitions.py`
- Modify: `gait_planner.py`

- [ ] **Step 1: Update both broken LIFT tests**

Two tests rely on `_reset()`'s `right_foot_force = 30.0` passing the old `UNLOAD_FORCE_THRESHOLD = 5 N`. Both break because `30.0 < STANCE_LOAD_THRESHOLD = 60 N`.

**Update 1** — find in `test_step_phase_transitions.py`:
```python
def test_lift_advances_to_swing_when_unloaded_and_settled():
    _reset()
    shared_state.step_phase              = StepPhase.LIFT
    shared_state.left_foot_force         = 2.0    # < UNLOAD_FORCE_THRESHOLD=5.0
    shared_state.left_foot_velocity      = np.array([0.0, 0.0, 0.01])  # < 0.05 m/s
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING
```

Replace with:
```python
def test_lift_advances_to_swing_when_unloaded_and_settled():
    _reset()
    shared_state.step_phase          = StepPhase.LIFT
    shared_state.left_foot_force     = 2.0    # N, < SWING_UNLOAD_THRESHOLD=5 N
    shared_state.right_foot_force    = 65.0   # N, > STANCE_LOAD_THRESHOLD=60 N
    shared_state.left_foot_velocity  = np.array([0.0, 0.0, 0.01])  # < 0.05 m/s
    gait_planner.update_gait_planner()
    assert shared_state.step_phase == StepPhase.SWING
```

**Update 2** — find in `test_step_phase_transitions.py`:
```python
def test_lift_snapshots_swing_foot_x_stance_on_transition():
    _reset()
    shared_state.step_phase          = StepPhase.LIFT
    shared_state.left_foot_position  = np.array([0.05, 0.0, 0.0])
    shared_state.left_foot_force     = 2.0
    shared_state.left_foot_velocity  = np.array([0.0, 0.0, 0.01])
    gait_planner.update_gait_planner()
    assert shared_state.swing_phase == 0.0
    # x_stance snapped from left_foot_position[0] at transition
    assert abs(shared_state.swing_foot_x_stance - 0.05) < 1e-9
```

Replace with:
```python
def test_lift_snapshots_swing_foot_x_stance_on_transition():
    _reset()
    shared_state.step_phase          = StepPhase.LIFT
    shared_state.left_foot_position  = np.array([0.05, 0.0, 0.0])
    shared_state.left_foot_force     = 2.0    # N, < SWING_UNLOAD_THRESHOLD=5 N
    shared_state.right_foot_force    = 65.0   # N, > STANCE_LOAD_THRESHOLD=60 N
    shared_state.left_foot_velocity  = np.array([0.0, 0.0, 0.01])
    gait_planner.update_gait_planner()
    assert shared_state.swing_phase == 0.0
    # x_stance snapped from left_foot_position[0] at transition
    assert abs(shared_state.swing_foot_x_stance - 0.05) < 1e-9
```

- [ ] **Step 2: Run LIFT tests — all should now pass**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_step_phase_transitions.py -k "lift" -v
```

Expected: **all LIFT tests PASS**.

- [ ] **Step 3: Implement Fix 2 — replace 1-D with 2-D in `_handle_com_shift`**

Find the full variable block at the top of `_handle_com_shift`:
```python
        timer     = shared_state.step_phase_timer
        cp_x      = float(shared_state.capture_point[0])
        stance_x  = float(shared_state.stance_foot_world_pos[0])
        stable    = (shared_state.stability_status != StabilityStatus.UNSTABLE)
        cp_close  = abs(cp_x - stance_x) < COM_SHIFT_THRESHOLD
```

Replace with:
```python
        timer    = shared_state.step_phase_timer
        stable   = (shared_state.stability_status != StabilityStatus.UNSTABLE)
        cp_close = (np.linalg.norm(
            shared_state.capture_point - shared_state.stance_foot_world_pos[:2]
        ) < COM_SHIFT_THRESHOLD)
```

- [ ] **Step 4: Run COM_SHIFT tests**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_step_phase_transitions.py -k "com_shift" -v
```

Expected: **all COM_SHIFT tests PASS**, including the new `test_com_shift_blocked_when_cp_offset_laterally`.

---

## Task 4: Implement Fixes 3 + 4 — rewrite `_handle_double_support` gate

**Files:**
- Modify: `gait_planner.py`

Fixes 3 and 4 both touch `_handle_double_support`. Replace the entire gate block (everything after `_compute_stance_ik()`) in one edit to avoid a broken intermediate state.

- [ ] **Step 1: Replace the gate block in `_handle_double_support`**

Find:
```python
        timer = shared_state.step_phase_timer
        if timer >= DS_TIMEOUT:
            shared_state.freeze_robot = True
            return

        both_confirmed = shared_state.both_feet_in_contact()
        if both_confirmed and timer >= DS_MIN_TIME:
            self._transition_to(StepPhase.COM_SHIFT)
```

Replace with:
```python
        timer = shared_state.step_phase_timer
        if timer >= DS_TIMEOUT:
            shared_state.freeze_robot = True
            return

        # Fix 4: soft ramp gate — DS timer counts during RAMP but no advance until full torque
        if shared_state.ramp_gain < 1.0:
            return

        both_confirmed = shared_state.both_feet_in_contact()
        if both_confirmed and timer >= DS_MIN_TIME:
            # Fix 3: force balance gate — neither foot carrying > FORCE_BALANCE_RATIO× the other
            lf = shared_state.left_foot_force
            rf = shared_state.right_foot_force
            if (lf >= FORCE_BALANCE_FLOOR and rf >= FORCE_BALANCE_FLOOR and
                    max(lf, rf) / min(lf, rf) <= FORCE_BALANCE_RATIO):
                self._transition_to(StepPhase.COM_SHIFT)
```

- [ ] **Step 2: Run all DS tests**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_step_phase_transitions.py -k "ds" -v
```

Expected: **all DS tests PASS**, including the 3 new ones (`ramp_gain`, `force_imbalanced`, `force_below_floor`).

---

## Task 5: Full suite verification and commit

**Files:**
- None changed in this task

- [ ] **Step 1: Run the complete test suite for the gait FSM**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
pytest test_step_phase_transitions.py \
       test_step_phase_guards.py \
       test_step_phase_timeouts.py \
       test_stance_anchor.py \
       test_velocity_gate.py \
       -v
```

Expected: **all tests PASS, 0 failures**. If any fail, do not commit — diagnose and fix first.

- [ ] **Step 2: Confirm no stray reference to the deleted constant**

```bash
grep -rn "UNLOAD_FORCE_THRESHOLD" /home/notlord/ros2_ws/Siclo1_V1/
```

Expected: **no output**.

- [ ] **Step 3: Commit**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
git add gait_planner.py test_step_phase_transitions.py
git commit -m "$(cat <<'EOF'
fix: 4-fix gait stability atomic commit

Fix 1 — dual-threshold weight transfer: replace UNLOAD_FORCE_THRESHOLD
with SWING_UNLOAD_THRESHOLD=5 N (swing empty) and STANCE_LOAD_THRESHOLD=60 N
(stance loaded) to enforce actual weight transfer before LIFT.

Fix 2 — COM_SHIFT 2D exit: replace 1D sagittal abs() check with
np.linalg.norm() across X+Y so lateral COM shift is actually verified.

Fix 3 — DS force balance gate: block DS→COM_SHIFT if either foot is below
FORCE_BALANCE_FLOOR=10 N or ratio max/min > FORCE_BALANCE_RATIO=2.0.

Fix 4 — soft ramp gate: DS timer counts during RAMP but phase does not
advance until ramp_gain >= 1.0 (full torque).

Tests: 4 new, 1 updated, 0 deleted.

Co-Authored-By: Claude Sonnet 4.6 <noreply@anthropic.com>
EOF
)"
```

---

## Tuning Checklist (post-simulation, not part of this commit)

After running a full simulation with `python3 main.py --walk 2.0`:

| Symptom | First tuning target |
|---------|-------------------|
| Every LIFT aborts back to DS | Lower `STANCE_LOAD_THRESHOLD` from 60 N to 50 N |
| COM_SHIFT always hits 1.0 s timeout | Raise `COM_SHIFT_THRESHOLD` from 0.03 m to 0.06 m |
| Robot freezes in DS at startup | Check `FORCE_BALANCE_FLOOR` — may need lowering if real sensor noise keeps forces below 10 N |
