# IDLE Step Timer Reset — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent EMERGENCY_STOP during IDLE by resetting `current_step_start_time` to `sim_time` at the top of `RecoveryController.evaluate()` whenever `mission_state == IDLE`.

**Architecture:** One line added to `recovery.py:evaluate()` before `get_step_duration()` is called. No new modules. Slip (Priority 4) and contact-loss (Priority 2) checks still evaluate in IDLE — only timer-based priorities (1, 3, 5) are defused. When IDLE→WALK transitions, the timer starts from 0 because the last IDLE cycle reset it to `sim_time`.

**Tech Stack:** Python 3.10+, pytest, `shared_state.MissionState`, `shared_state.RecoveryAction`

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `recovery.py` | Modify | Add 2-line IDLE reset at top of `RecoveryController.evaluate()` (before line 115) |
| `test_recovery_idle_reset.py` | Create | Two pytest cases covering the two success criteria |

---

## Task 4 — IDLE timer reset in `recovery.py`

**Files:**
- Modify: `recovery.py` (lines 113-115, inside `RecoveryController.evaluate()`)
- Create: `test_recovery_idle_reset.py`

---

- [ ] **Step 1: Write the two failing tests**

Create `/home/notlord/ros2_ws/Siclo1_V1/test_recovery_idle_reset.py`:

```python
"""Tests for IDLE step-timer reset in recovery.py.

No PyBullet required. All shared_state fields set manually.

Verifies:
  1. EMERGENCY_STOP never fires during a long IDLE (timer kept fresh).
  2. step_duration is near-zero on the first WALK cycle after a long IDLE
     (timer was reset on the last IDLE cycle, so WALK starts clean).
"""
import numpy as np
import pytest
from shared_state import (
    shared_state,
    ContactState,
    MissionState,
    StabilityStatus,
    RecoveryAction,
    RecoveryConfig,
)


def _reset_for_idle():
    """Shared_state for a robot standing in IDLE with both feet confirmed."""
    shared_state.reset()
    shared_state.mission_state = MissionState.IDLE
    shared_state.sim_time = 0.0
    shared_state.current_step_start_time = 0.0

    # Worst case: stability reports UNSTABLE (no contacts confirmed yet)
    shared_state.set_stability_status(StabilityStatus.UNSTABLE, margin=-0.05)
    # Both feet confirmed (normal standing)
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    # No slipping
    shared_state.set_slip_detection('left',  False)
    shared_state.set_slip_detection('right', False)


def test_no_emergency_stop_after_long_idle():
    """EMERGENCY_STOP must NOT fire after sim_time >> 3.0 s in IDLE.

    Scenario: robot stands in IDLE for 10 s. Timer has drifted:
        step_duration = 10.0 - 0.0 = 10.0 s >> 3.0 s threshold.
    Without the fix, Priority 1 (is_unstable + timeout) fires immediately.
    With the fix, current_step_start_time is reset each cycle → duration ≈ 0.
    """
    from recovery import update_recovery

    _reset_for_idle()
    # Simulate 10 s of sim_time passing with no timer reset
    shared_state.sim_time = 10.0
    shared_state.current_step_start_time = 0.0   # stale — the bug scenario

    update_recovery()

    assert shared_state.recovery_action != RecoveryAction.EMERGENCY_STOP, (
        f"EMERGENCY_STOP fired during IDLE after 10 s. "
        f"action={shared_state.recovery_action.name}, "
        f"reason='{shared_state.recovery_reason}'"
    )
    # Timer must have been reset to sim_time
    assert abs(shared_state.current_step_start_time - 10.0) < 1e-6, (
        f"Timer not reset: current_step_start_time={shared_state.current_step_start_time}"
    )


def test_step_duration_near_zero_on_first_walk_cycle():
    """step_duration must be near-zero on the first WALK cycle after long IDLE.

    Scenario:
      1. IDLE for 10 s — last IDLE cycle resets timer to sim_time=10.0.
      2. sim_time advances by one dt (10.01 s), mission_state flips to WALK.
      3. First WALK cycle: step_duration = 10.01 - 10.0 = 0.01 s << 3.0 s.
    Without the fix, step_duration = 10.01 - 0.0 = 10.01 s → immediate EMERGENCY_STOP.
    """
    from recovery import update_recovery

    _reset_for_idle()
    shared_state.sim_time = 10.0
    shared_state.current_step_start_time = 0.0   # stale

    # Simulate the last IDLE cycle (resets the timer)
    update_recovery()
    assert abs(shared_state.current_step_start_time - 10.0) < 1e-6

    # Advance one dt and flip to WALK
    shared_state.sim_time = 10.01
    shared_state.mission_state = MissionState.WALK
    # Both feet confirmed — no contact-loss trigger
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)

    update_recovery()

    duration = shared_state.get_step_duration()
    assert duration < 0.1, (
        f"step_duration on first WALK cycle too large: {duration:.3f} s. "
        f"Watchdog will fire immediately."
    )
    assert shared_state.recovery_action != RecoveryAction.EMERGENCY_STOP, (
        f"EMERGENCY_STOP fired on first WALK cycle. action={shared_state.recovery_action.name}"
    )
```

---

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python3 -m pytest test_recovery_idle_reset.py -v
```

Expected:
```
FAILED test_recovery_idle_reset.py::test_no_emergency_stop_after_long_idle
FAILED test_recovery_idle_reset.py::test_step_duration_near_zero_on_first_walk_cycle
```

`test_no_emergency_stop_after_long_idle` fails because Priority 1 fires EMERGENCY_STOP (is_unstable=True, step_duration=10.0 > 3.0).
`test_step_duration_near_zero_on_first_walk_cycle` fails for the same reason on the first call.

---

- [ ] **Step 3: Add the IDLE reset to `recovery.py`**

In `recovery.py`, find the block at lines 113-115 inside `RecoveryController.evaluate()`:

```python
        """
        # Get current state
        step_duration = shared_state.get_step_duration()
```

Replace with:

```python
        """
        # Reset step timer every cycle in IDLE so timeout checks (P1, P3, P5) never
        # fire while standing still. Slip (P4) and contact-loss (P2) still evaluate.
        # Side effect: when IDLE→WALK, timer starts from 0 — watchdog begins clean.
        if shared_state.mission_state == MissionState.IDLE:
            shared_state.current_step_start_time = shared_state.sim_time

        # Get current state
        step_duration = shared_state.get_step_duration()
```

---

- [ ] **Step 4: Run the two new tests to confirm they pass**

```bash
python3 -m pytest test_recovery_idle_reset.py -v
```

Expected:
```
PASSED test_recovery_idle_reset.py::test_no_emergency_stop_after_long_idle
PASSED test_recovery_idle_reset.py::test_step_duration_near_zero_on_first_walk_cycle
```

---

- [ ] **Step 5: Run the full test suite (regression check)**

```bash
python3 -m pytest --ignore=test_physics.py --ignore=test_physics_detailed.py -v 2>&1 | tail -30
```

Expected: no new `FAILED` lines. Note the total passing count — it should be all prior tests plus the 2 new ones.

---

- [ ] **Step 6: Commit**

```bash
git add recovery.py test_recovery_idle_reset.py
git commit -m "feat: reset step timer in IDLE to prevent watchdog EMERGENCY_STOP"
```

---

## Verification

End-to-end sanity check across all four tasks:

```bash
python3 -m pytest \
  test_stability_capture_point.py::test_stable_when_standing_still \
  test_gait_shared_state.py::test_capture_point_field_exists_with_default \
  test_gait_planner.py::test_reset_step_called_at_touchdown \
  test_recovery_idle_reset.py::test_no_emergency_stop_after_long_idle \
  -v
```

Expected: all 4 `PASSED`.

**Success criteria:**
1. Robot stands in IDLE indefinitely — no EMERGENCY_STOP regardless of `sim_time`
2. First WALK cycle has `step_duration < 0.1 s` — watchdog starts clean
3. All pre-existing tests continue to pass
