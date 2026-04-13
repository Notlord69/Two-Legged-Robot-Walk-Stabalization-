# Robust Contact Confirmation Gate — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the contact confirmation gate so PyBullet's single-contact-point degenerate case (stable flat box on a flat plane) no longer blocks `CONTACT_CONFIRMED`, unblocking the IDLE→RAMP transition and ending the 302-cycle timeout death.

**Architecture:** Three surgical changes in the sensor layer only. A pure helper function `_compute_foot_flat(pts_x, foot_pitch)` replaces the inline flat gate in `read_sensors()`. Foot link pitch (already fetched via `getLinkState`) is extracted and stored per-cycle. The recovery module gains a mission-state guard that prevents the 3-second timeout from firing while the robot is in IDLE. No changes to `perception.py`, `mission.py`, or any control module.

**Tech Stack:** Python 3.10, PyBullet (quaternion from `getLinkState`), pytest.

---

## File Map

| File | Action | What changes |
|------|--------|--------------|
| `shared_state.py` | Modify | Add `left_foot_pitch`, `right_foot_pitch` fields to `__init__` and `reset()` |
| `HeartBeat.py` | Modify | Add `import math`; add `FLAT_PITCH_THRESHOLD` constant; add `_compute_foot_flat()` helper; update per-foot blocks in `read_sensors()` |
| `recovery.py` | Modify | Add `MissionState` to imports; add `mission_state != IDLE` guard in `evaluate()` |
| `test_transitions.py` | Modify | Add pytest functions for `_compute_foot_flat` and perception integration |
| `test_mission.py` | Modify | Add recovery IDLE-guard tests |

---

## Task 1 — Add foot pitch fields to `shared_state.py`

**Files:**
- Modify: `shared_state.py:207` (after `right_foot_flat`)
- Modify: `shared_state.py:483` (after `right_foot_flat` in `reset()`)
- Test: `test_transitions.py`

- [ ] **Step 1: Write the failing tests**

Add to the bottom of `test_transitions.py`:

```python
import math
import pytest


# ── Task 1 tests ──────────────────────────────────────────────────────────── #

def test_foot_pitch_fields_exist_after_init():
    """shared_state has left_foot_pitch and right_foot_pitch initialised to 0."""
    from shared_state import Siclo1State
    s = Siclo1State()
    assert hasattr(s, 'left_foot_pitch')
    assert hasattr(s, 'right_foot_pitch')
    assert s.left_foot_pitch == 0.0
    assert s.right_foot_pitch == 0.0


def test_foot_pitch_fields_reset_to_zero():
    """reset() brings left_foot_pitch and right_foot_pitch back to 0."""
    shared_state.left_foot_pitch  = 0.5
    shared_state.right_foot_pitch = -0.3
    shared_state.reset()
    assert shared_state.left_foot_pitch  == 0.0
    assert shared_state.right_foot_pitch == 0.0
```

- [ ] **Step 2: Run tests and confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python3 -m pytest test_transitions.py::test_foot_pitch_fields_exist_after_init \
                  test_transitions.py::test_foot_pitch_fields_reset_to_zero -v
```

Expected: `AttributeError: 'Siclo1State' object has no attribute 'left_foot_pitch'`

- [ ] **Step 3: Add the fields to `shared_state.py`**

In `shared_state.py`, locate the block starting at line 203:

```python
        # --- 3-Tick Validation & Flat Foot ---
        self.left_contact_ticks: int = 0
        self.right_contact_ticks: int = 0
        self.left_foot_flat: bool = False
        self.right_foot_flat: bool = False
```

Replace with:

```python
        # --- 3-Tick Validation & Flat Foot ---
        self.left_contact_ticks: int = 0
        self.right_contact_ticks: int = 0
        self.left_foot_flat: bool = False
        self.right_foot_flat: bool = False
        self.left_foot_pitch:  float = 0.0   # rad — world-Y pitch of foot link; updated by read_sensors()
        self.right_foot_pitch: float = 0.0   # rad — world-Y pitch of foot link; updated by read_sensors()
```

In `shared_state.py`, locate the block in `reset()` starting at line 480:

```python
            self.left_contact_ticks = 0
            self.right_contact_ticks = 0
            self.left_foot_flat = False
            self.right_foot_flat = False
```

Replace with:

```python
            self.left_contact_ticks  = 0
            self.right_contact_ticks = 0
            self.left_foot_flat      = False
            self.right_foot_flat     = False
            self.left_foot_pitch     = 0.0
            self.right_foot_pitch    = 0.0
```

- [ ] **Step 4: Run tests and confirm they pass**

```bash
python3 -m pytest test_transitions.py::test_foot_pitch_fields_exist_after_init \
                  test_transitions.py::test_foot_pitch_fields_reset_to_zero -v
```

Expected: `2 passed`

- [ ] **Step 5: Commit**

```bash
git add shared_state.py test_transitions.py
git commit -m "feat: add left/right_foot_pitch fields to shared_state"
```

---

## Task 2 — Add `_compute_foot_flat()` and `FLAT_PITCH_THRESHOLD` to `HeartBeat.py`

**Files:**
- Modify: `HeartBeat.py` (add `import math`, constant, helper function)
- Test: `test_transitions.py`

- [ ] **Step 1: Write the failing tests**

Append to `test_transitions.py` (after the Task 1 tests):

```python
# ── Task 2 tests ──────────────────────────────────────────────────────────── #

class TestComputeFootFlat:
    """Unit tests for the _compute_foot_flat pure helper in HeartBeat.py."""

    def test_single_contact_low_pitch_is_flat(self):
        """Single contact point + pitch 4° (< 7°) → flat confirmed."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0], math.radians(4.0)) is True

    def test_single_contact_high_pitch_not_flat(self):
        """Single contact point + pitch 15° (> 7°) → tiptoe, not confirmed."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0], math.radians(15.0)) is False

    def test_single_contact_exactly_threshold_not_flat(self):
        """7° exactly is rejected — gate is strictly less than."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0], math.radians(7.0)) is False

    def test_single_contact_negative_pitch_accepted(self):
        """Negative pitch (heel slightly raised on opposite side) within 7° → flat."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0], math.radians(-4.0)) is True

    def test_multi_point_wide_spread_is_flat(self):
        """Multiple contact points with 3 cm spread → flat regardless of pitch."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0, 0.03], math.radians(45.0)) is True

    def test_multi_point_narrow_spread_not_flat(self):
        """Multiple contact points with 0.5 cm spread → NOT flat (below 1 cm gate)."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0, 0.005], math.radians(0.0)) is False

    def test_no_contact_points_not_flat(self):
        """Empty contact list → not flat."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([], math.radians(0.0)) is False

    def test_tick_gate_still_required(self):
        """Foot flat=True but ticks=2 → state stays CONTACT_TENTATIVE (ticks gate untouched)."""
        import perception
        from shared_state import shared_state, ContactState
        shared_state.reset()
        perception.reset_perception()
        shared_state.left_foot_position = np.array([0.0, 0.0, 0.01])
        shared_state.left_contact_ticks = 2
        shared_state.left_foot_flat = True
        state, _ = perception.update_perception()
        assert state == ContactState.CONTACT_TENTATIVE
```

- [ ] **Step 2: Run tests and confirm they fail**

```bash
python3 -m pytest test_transitions.py::TestComputeFootFlat -v
```

Expected: `ImportError: cannot import name '_compute_foot_flat' from 'HeartBeat'`

- [ ] **Step 3: Add `import math`, the constant, and the helper to `HeartBeat.py`**

**3a.** In `HeartBeat.py`, find the existing imports at the top of the file (around line 26). Add `import math` on the line after `import threading`:

```python
import threading
import math
```

**3b.** In `HeartBeat.py`, locate the module-level constants block (around line 64–90, after `_WBC_RIGHT_JOINTS`). The `_clip_effort` function lives at line 192. Add the constant and helper immediately before `_clip_effort`:

```python
# Foot-flat gate — single contact point pitch tolerance
FLAT_PITCH_THRESHOLD: float = math.radians(7.0)
# rad — foot-flat gate for single-contact confirmation.
# At q=0 the URDF foot sole is parallel to the floor (zero plantar-flexion offset).
# 7° accepts post-bounce settling (0–6°); rejects tiptoe (>20°) and
# transient heel-strike (8–15°, never accumulates 3 ticks anyway).


def _compute_foot_flat(pts_x: list, foot_pitch: float) -> bool:
    """Pure function: is the foot confirmed flat from contact geometry and pitch?

    pts_x       -- list of contact point X coordinates (world frame, metres).
    foot_pitch  -- foot link pitch in radians (rotation about world Y, from getLinkState).

    Multi-point path: requires spread > 1 cm (original behaviour).
    Single-point path: pitch fallback for the PyBullet degenerate case where a
        stable rectangular box on a flat plane returns only one contact point.
    """
    if len(pts_x) > 1:
        return (max(pts_x) - min(pts_x)) > 0.01
    if len(pts_x) == 1:
        return abs(foot_pitch) < FLAT_PITCH_THRESHOLD
    return False
```

- [ ] **Step 4: Run tests and confirm they pass**

```bash
python3 -m pytest test_transitions.py::TestComputeFootFlat -v
```

Expected: `8 passed`

- [ ] **Step 5: Commit**

```bash
git add HeartBeat.py test_transitions.py
git commit -m "feat: add _compute_foot_flat helper and FLAT_PITCH_THRESHOLD constant"
```

---

## Task 3 — Wire pitch extraction into `read_sensors()`

**Files:**
- Modify: `HeartBeat.py:321–350` (per-foot blocks inside `read_sensors()`)

No new tests are needed — `_compute_foot_flat` is already tested in Task 2. This task wires the tested helper into the sensor read loop.

- [ ] **Step 1: Update the left-foot block in `read_sensors()`**

Locate the left-foot block in `PyBulletInterface.read_sensors()` (around line 321). The current code is:

```python
                if foot == 'left':
                    ss.left_foot_position = pos
                    ss.left_foot_velocity = vel
                    ss.left_foot_force = force
                    if force > threshold:
                        ss.left_contact_ticks += 1
                        # Flat Foot: X-spread of contact points > 1cm (more realistic for Siclo1)
                        pts_x = [c[5][0] for c in contacts]
                        ss.left_foot_flat = (max(pts_x) - min(pts_x)) > 0.01 if len(pts_x) > 1 else False
                        # Store all contact point positions
                        ss.left_contact_points = [np.array(c[5]) for c in contacts]
                    else:
                        ss.left_contact_ticks = 0
                        ss.left_foot_flat = False
                        ss.left_contact_points = []
```

Replace with:

```python
                if foot == 'left':
                    ss.left_foot_position = pos
                    ss.left_foot_velocity = vel
                    ss.left_foot_force = force
                    # Pitch is kinematic — computed unconditionally, not gated on force
                    qx, qy, qz, qw = link_state[1]
                    foot_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
                    ss.left_foot_pitch = foot_pitch
                    if force > threshold:
                        ss.left_contact_ticks += 1
                        pts_x = [c[5][0] for c in contacts]
                        ss.left_foot_flat = _compute_foot_flat(pts_x, foot_pitch)
                        ss.left_contact_points = [np.array(c[5]) for c in contacts]
                    else:
                        ss.left_contact_ticks = 0
                        ss.left_foot_flat = False
                        ss.left_contact_points = []
```

- [ ] **Step 2: Update the right-foot block in `read_sensors()`**

Locate the right-foot block (around line 336). The current code is:

```python
                else:
                    ss.right_foot_position = pos
                    ss.right_foot_velocity = vel
                    ss.right_foot_force = force
                    if force > threshold:
                        ss.right_contact_ticks += 1
                        pts_x = [c[5][0] for c in contacts]
                        # Reduced threshold to 1cm spread
                        ss.right_foot_flat = (max(pts_x) - min(pts_x)) > 0.01 if len(pts_x) > 1 else False
                        # Store all contact point positions
                        ss.right_contact_points = [np.array(c[5]) for c in contacts]
                    else:
                        ss.right_contact_ticks = 0
                        ss.right_foot_flat = False
                        ss.right_contact_points = []
```

Replace with:

```python
                else:
                    ss.right_foot_position = pos
                    ss.right_foot_velocity = vel
                    ss.right_foot_force = force
                    # Pitch is kinematic — computed unconditionally, not gated on force
                    qx, qy, qz, qw = link_state[1]
                    foot_pitch = math.asin(max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx))))
                    ss.right_foot_pitch = foot_pitch
                    if force > threshold:
                        ss.right_contact_ticks += 1
                        pts_x = [c[5][0] for c in contacts]
                        ss.right_foot_flat = _compute_foot_flat(pts_x, foot_pitch)
                        ss.right_contact_points = [np.array(c[5]) for c in contacts]
                    else:
                        ss.right_contact_ticks = 0
                        ss.right_foot_flat = False
                        ss.right_contact_points = []
```

- [ ] **Step 3: Run the full test suite to confirm no regressions**

```bash
python3 -m pytest test_transitions.py test_mission.py test_gui_sync_thread.py \
                  test_gait_planner.py test_dual_mode_runner.py -v
```

Expected: all previously passing tests still pass; `TestComputeFootFlat` (8 tests) pass.

- [ ] **Step 4: Commit**

```bash
git add HeartBeat.py
git commit -m "feat: wire foot pitch extraction and OR gate into read_sensors()"
```

---

## Task 4 — Add recovery IDLE guard to `recovery.py`

**Files:**
- Modify: `recovery.py:32–38` (imports block)
- Modify: `recovery.py:141` (timeout condition)
- Test: `test_mission.py`

- [ ] **Step 1: Write the failing tests**

**1a.** In `test_mission.py`, update the existing top-of-file import (line 4) to add `RecoveryAction`:

```python
from shared_state import shared_state, Siclo1State, ContactState, MissionState, RecoveryAction
```

**1b.** Append to `test_mission.py` (after the last existing test):

```python
# ── Task 4 tests — recovery IDLE guard ───────────────────────────────────── #

def _reset_for_recovery():
    """Shared setup: step_duration = 4 s (> 3 s threshold), both feet unconfirmed.
    Priorities 1, 4, 5 are suppressed so only Priority 3 (timeout) can fire."""
    shared_state.reset()
    shared_state.current_step_start_time = 0.0
    shared_state.sim_time = 4.0           # step_duration = 4.0 > timeout_threshold
    # Both feet unconfirmed — would trigger Priority 3 if mission is not IDLE
    shared_state.set_contact_state('left',  ContactState.NO_CONTACT)
    shared_state.set_contact_state('right', ContactState.NO_CONTACT)
    # Priority 1 suppressed: is_unstable = False  (done by reset())
    # Priority 4 suppressed: slip flags = False    (done by reset())
    # Priority 5 suppressed: stability_status = STABLE (done by reset())


def test_recovery_timeout_blocked_when_idle():
    """Priority 3 timeout must NOT fire when mission_state == IDLE.
    A standing robot that was never commanded to walk must not self-destruct."""
    from recovery import RecoveryController
    _reset_for_recovery()
    shared_state.mission_state = MissionState.IDLE
    controller = RecoveryController()
    action, _ = controller.evaluate()
    assert action == RecoveryAction.NONE


def test_recovery_timeout_fires_when_walking():
    """Priority 3 timeout still fires during WALK — existing behaviour preserved."""
    from recovery import RecoveryController
    _reset_for_recovery()
    shared_state.mission_state = MissionState.WALK
    controller = RecoveryController()
    action, _ = controller.evaluate()
    assert action == RecoveryAction.REPOSITION
```

- [ ] **Step 2: Run tests and confirm they fail**

```bash
python3 -m pytest test_mission.py::test_recovery_timeout_blocked_when_idle \
                  test_mission.py::test_recovery_timeout_fires_when_walking -v
```

Expected: `test_recovery_timeout_blocked_when_idle` FAILS (currently returns REPOSITION instead of NONE). `test_recovery_timeout_fires_when_walking` PASSES (no change yet — it already works).

- [ ] **Step 3: Add `MissionState` to `recovery.py` imports**

Locate the imports block in `recovery.py` (around line 32):

```python
from shared_state import (
    shared_state,
    ContactState,
    StabilityStatus,
    RecoveryAction,
    RecoveryConfig
)
```

Replace with:

```python
from shared_state import (
    shared_state,
    ContactState,
    MissionState,
    StabilityStatus,
    RecoveryAction,
    RecoveryConfig,
)
```

- [ ] **Step 4: Add the IDLE guard to `RecoveryController.evaluate()`**

Locate the Priority 3 block in `recovery.py` (around line 141):

```python
        if both_not_confirmed and step_duration > self.config.timeout_threshold:
            # Check if we've tried too many times
            if self.current_step_attempts >= self.config.max_recovery_attempts:
                return RecoveryAction.EMERGENCY_STOP, "Max recovery attempts exceeded"
            else:
                return RecoveryAction.REPOSITION, "Timeout without contact - reposition"
```

Replace with:

```python
        if (both_not_confirmed and
                step_duration > self.config.timeout_threshold and
                shared_state.mission_state != MissionState.IDLE):
            # Check if we've tried too many times
            if self.current_step_attempts >= self.config.max_recovery_attempts:
                return RecoveryAction.EMERGENCY_STOP, "Max recovery attempts exceeded"
            else:
                return RecoveryAction.REPOSITION, "Timeout without contact - reposition"
```

- [ ] **Step 5: Run tests and confirm they pass**

```bash
python3 -m pytest test_mission.py::test_recovery_timeout_blocked_when_idle \
                  test_mission.py::test_recovery_timeout_fires_when_walking -v
```

Expected: `2 passed`

- [ ] **Step 6: Run the full test suite for final regression check**

```bash
python3 -m pytest test_transitions.py test_mission.py test_gui_sync_thread.py \
                  test_gait_planner.py test_dual_mode_runner.py \
                  test_heartbeat_gait_wiring.py test_grf.py -v
```

Expected: all tests pass.

- [ ] **Step 7: Commit**

```bash
git add recovery.py test_mission.py
git commit -m "fix: gate recovery timeout on mission_state != IDLE

A robot standing in IDLE (never commanded to walk) must not trigger
the 3-second contact-timeout. The timeout is only meaningful during
an active step (RAMP/WALK/DECEL/STOP)."
```
