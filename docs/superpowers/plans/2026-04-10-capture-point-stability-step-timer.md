# Capture Point Stability + Step Timer Reset — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the 302-cycle EMERGENCY_STOP by replacing the static COM stability check with a LIPM Capture Point check, and wiring `recovery.reset_step()` at step touchdown in the gait planner.

**Architecture:** Three files change — `shared_state.py` gains a `capture_point` field; `stability.py` replaces the static COM containment check with a Capture Point containment check; `gait_planner.py` calls `recovery.reset_step()` at touchdown. No new modules. No changes to `recovery.py` or `HeartBeat.py`.

**Tech Stack:** Python 3.10+, NumPy, Shapely (already used in stability.py), pytest

---

## File Map

| File | Action | Responsibility |
|---|---|---|
| `shared_state.py` | Modify | Add `capture_point: np.ndarray` field to `__init__`, `reset()`, `get_diagnostics()` |
| `stability.py` | Modify | Add `import math`, add `G`/`Z_COM_MIN` constants, replace static COM check with Capture Point in `check_stability()` |
| `gait_planner.py` | Modify | Add `import recovery`, call `recovery.reset_step()` at `φ ≥ 1.0` touchdown |
| `test_gait_shared_state.py` | Modify | Add `capture_point` field existence and reset tests |
| `test_stability_capture_point.py` | Create | Three Capture Point stability tests |
| `test_gait_planner.py` | Modify | Add test asserting `recovery.reset_step()` called at touchdown |

---

## Task 1 — Formalise `capture_point` in `shared_state.py`

**Files:**
- Modify: `shared_state.py` (`__init__`, `reset()`, `get_diagnostics()`)
- Modify: `test_gait_shared_state.py`

- [ ] **Step 1: Write the failing tests**

Open `test_gait_shared_state.py` and append these two tests:

```python
def test_capture_point_field_exists_with_default():
    """capture_point initialises to 2D zero vector."""
    s = Siclo1State()
    assert hasattr(s, 'capture_point')
    assert s.capture_point.shape == (2,)
    assert s.capture_point[0] == 0.0
    assert s.capture_point[1] == 0.0


def test_capture_point_resets_to_zero():
    """reset() restores capture_point to zeros."""
    import numpy as np
    s = Siclo1State()
    s.capture_point = np.array([1.5, -0.3])
    s.reset()
    assert s.capture_point[0] == 0.0
    assert s.capture_point[1] == 0.0
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python -m pytest test_gait_shared_state.py::test_capture_point_field_exists_with_default test_gait_shared_state.py::test_capture_point_resets_to_zero -v
```

Expected: `FAILED` — `AttributeError: 'Siclo1State' object has no attribute 'capture_point'`

- [ ] **Step 3: Add `capture_point` to `shared_state.py`**

In `shared_state.py`, inside `Siclo1State.__init__()`, locate the `# KINEMATICS STATE` block (around line 260). Add after `self.torso_pitch_correction`:

```python
# Capture Point — LIPM extrapolated COM (X-Y world frame, metres).
# Written by stability.py every 100 Hz cycle; read by gait_planner.py
# for foot target placement.
self.capture_point: np.ndarray = np.zeros(2)
```

In `reset()` (around line 473), add:

```python
self.capture_point = np.zeros(2)
```

In `get_diagnostics()` (around line 420), add:

```python
'capture_point':     self.capture_point.tolist(),
```

- [ ] **Step 4: Run tests to confirm they pass**

```bash
python -m pytest test_gait_shared_state.py::test_capture_point_field_exists_with_default test_gait_shared_state.py::test_capture_point_resets_to_zero -v
```

Expected: `PASSED` (both)

- [ ] **Step 5: Run full existing test suite — confirm no regressions**

```bash
python -m pytest test_gait_shared_state.py -v
```

Expected: all existing tests still `PASSED`

- [ ] **Step 6: Commit**

```bash
git add shared_state.py test_gait_shared_state.py
git commit -m "feat: add capture_point field to Siclo1State"
```

---

## Task 2 — Capture Point check in `stability.py`

**Files:**
- Create: `test_stability_capture_point.py`
- Modify: `stability.py`

- [ ] **Step 1: Write the three failing tests**

Create `test_stability_capture_point.py`:

```python
"""Tests for Capture Point stability classification in stability.py.

No PyBullet required. All shared_state fields are set manually.
Tests verify LIPM Capture Point (CP = com_xy + v_com_xy / omega_n) is used
instead of the static COM for polygon containment.
"""
import math
import numpy as np
import pytest
from shared_state import shared_state, ContactState, StabilityStatus


def _reset_for_stability():
    """Set up shared_state for a standing robot with both feet confirmed.

    Contact points are set explicitly (2 per foot) to form a valid rectangle
    polygon with ≥ 3 non-collinear points.  Using only foot_position fallback
    gives 2 collinear points — Shapely cannot build a polygon from those.

    Polygon corners (X, Y):
        (-0.1, +0.05)  (-0.1, -0.05)   ← left foot (front, back)
        (+0.1, +0.05)  (+0.1, -0.05)   ← right foot (front, back)
    COM at (0, 0) is well inside this rectangle.
    """
    shared_state.reset()

    # Robot standing upright: COM at nominal height, no load
    shared_state.base_position = np.array([0.0, 0.0, 0.8806])
    shared_state.link_positions = {}          # empty → COM falls back to base_position
    shared_state.current_load_mass = 0.0

    # Both feet confirmed on the ground
    shared_state.set_contact_state('left',  ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    shared_state.left_foot_position  = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position = np.array([ 0.1, 0.0, 0.0])

    # Explicit contact points — 2 per foot, forming a valid convex rectangle.
    # get_confirmed_contact_points() uses these when the list is non-empty.
    shared_state.left_contact_points = [
        np.array([-0.1,  0.05, 0.0]),   # left foot, forward edge
        np.array([-0.1, -0.05, 0.0]),   # left foot, rear edge
    ]
    shared_state.right_contact_points = [
        np.array([ 0.1,  0.05, 0.0]),   # right foot, forward edge
        np.array([ 0.1, -0.05, 0.0]),   # right foot, rear edge
    ]


def test_stable_when_standing_still():
    """Both feet confirmed, zero velocity → CP = COM → inside polygon → STABLE."""
    from stability import stability_monitor, update_stability

    _reset_for_stability()
    # Zero velocity: prime prev_com so first finite-difference gives v=0
    stability_monitor.prev_com = np.array([0.0, 0.0, 0.8806])

    status = update_stability(dt=0.01)

    assert status == StabilityStatus.STABLE, (
        f"Expected STABLE, got {status.name}. "
        f"CP should equal COM at zero velocity."
    )
    # capture_point should be written and equal to COM xy
    assert shared_state.capture_point.shape == (2,)
    assert abs(shared_state.capture_point[0]) < 0.01   # near zero x
    assert abs(shared_state.capture_point[1]) < 0.01   # near zero y


def test_unstable_when_high_lateral_velocity():
    """High lateral velocity pushes CP outside support polygon → UNSTABLE.

    At z_com = 0.8806 m:
        omega_n = sqrt(9.81 / 0.8806) ≈ 3.338 rad/s
        v_y = 2.0 m/s  →  CP_y = 0 + 2.0 / 3.338 ≈ 0.599 m

    Support polygon spans Y: [-0.05, +0.05] m (foot half-width from contact points).
    CP_y ≈ 0.599 >> 0.05 → CP is well outside → UNSTABLE.
    """
    from stability import stability_monitor, update_stability

    _reset_for_stability()

    # Simulate lateral velocity by setting prev_com offset:
    # com_now = [0, 0, 0.8806], prev_com shifted so v_y = 2.0 m/s over dt=0.01 s
    dt = 0.01
    v_y = 2.0  # m/s, lateral — pushes CP far outside ±0.1 m polygon
    stability_monitor.prev_com = np.array([0.0, -v_y * dt, 0.8806])

    status = update_stability(dt=dt)

    assert status == StabilityStatus.UNSTABLE, (
        f"Expected UNSTABLE, got {status.name}. "
        f"CP_y ≈ {v_y / math.sqrt(9.81 / 0.8806):.3f} m should be outside ±0.1 m polygon."
    )
    # CP should reflect the extrapolation
    omega_n = math.sqrt(9.81 / 0.8806)
    expected_cp_y = 0.0 + v_y / omega_n
    assert abs(shared_state.capture_point[1] - expected_cp_y) < 0.05


def test_unstable_when_no_confirmed_contacts():
    """No confirmed contacts → no support polygon → UNSTABLE (existing behaviour)."""
    from stability import update_stability
    from shared_state import ContactState

    shared_state.reset()
    shared_state.base_position = np.array([0.0, 0.0, 0.8806])
    shared_state.link_positions = {}
    # Both feet explicitly NOT confirmed
    shared_state.set_contact_state('left',  ContactState.NO_CONTACT)
    shared_state.set_contact_state('right', ContactState.NO_CONTACT)

    status = update_stability(dt=0.01)

    assert status == StabilityStatus.UNSTABLE, (
        f"Expected UNSTABLE with no contact polygon, got {status.name}"
    )
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
python -m pytest test_stability_capture_point.py -v
```

Expected: `FAILED` — tests pass or fail with wrong reasons because stability still uses static COM.
`test_stable_when_standing_still` may pass by accident (COM is also inside polygon at v=0).
`test_unstable_when_high_lateral_velocity` will fail because static COM at [0,0] is inside polygon regardless of velocity.
`test_unstable_when_no_confirmed_contacts` should already pass (existing behaviour).

- [ ] **Step 3: Implement Capture Point in `stability.py`**

**3a — Add `import math` at the top of `stability.py`, after `import numpy as np`:**

```python
import math
```

**3b — Add constants at module level, after the imports block:**

```python
# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

G: float = 9.81          # m/s², standard gravitational acceleration
Z_COM_MIN: float = 0.05  # m, floor guard for omega_n — prevents sqrt domain error
                          # (robot at/below floor is already UNSTABLE; guard avoids crash)
```

**3c — Replace the containment check in `check_stability()`.** Find the block starting at line 291 (approximately):

```python
        com_2d = Point(com[0], com[1])

        if polygon.contains(com_2d):
            margin_distance = polygon.exterior.distance(com_2d)
            if margin_distance > safety_margin * 0.5:
                shared_state.set_stability_status(StabilityStatus.STABLE, margin=margin_distance)
                return StabilityStatus.STABLE
            else:
                shared_state.set_stability_status(StabilityStatus.MARGINAL, margin=margin_distance)
                return StabilityStatus.MARGINAL
        else:
            margin_distance = -polygon.exterior.distance(com_2d)
            shared_state.set_stability_status(StabilityStatus.UNSTABLE, margin=margin_distance)
            return StabilityStatus.UNSTABLE
```

Replace it entirely with:

```python
        # Capture Point — LIPM extrapolated COM (Option A).
        # CP = com_xy + v_com_xy / omega_n
        # omega_n = sqrt(g / z_com): natural frequency of the inverted pendulum.
        # At v=0 (standing still), CP = COM → degrades to static check.
        z_com   = max(com[2], Z_COM_MIN)                  # guard: robot at/below floor
        omega_n = math.sqrt(G / z_com)                    # rad/s
        cp_xy   = com[:2] + com_vel[:2] / omega_n         # 2D capture point (X, Y)
        shared_state.capture_point = cp_xy                 # write for gait_planner + diagnostics

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

- [ ] **Step 4: Run the three new tests to confirm they pass**

```bash
python -m pytest test_stability_capture_point.py -v
```

Expected:
```
PASSED test_stability_capture_point.py::test_stable_when_standing_still
PASSED test_stability_capture_point.py::test_unstable_when_high_lateral_velocity
PASSED test_stability_capture_point.py::test_unstable_when_no_confirmed_contacts
```

- [ ] **Step 5: Run all existing tests to confirm no regressions**

```bash
python -m pytest --ignore=test_physics.py --ignore=test_physics_detailed.py -v 2>&1 | tail -20
```

(Physics tests require PyBullet — skip if no display. All other tests must pass.)

Expected: no new `FAILED` lines.

- [ ] **Step 6: Commit**

```bash
git add stability.py test_stability_capture_point.py
git commit -m "feat: replace static COM check with LIPM Capture Point in stability.py"
```

---

## Task 3 — Wire `recovery.reset_step()` at touchdown in `gait_planner.py`

**Files:**
- Modify: `gait_planner.py`
- Modify: `test_gait_planner.py`

- [ ] **Step 1: Write the failing test**

Open `test_gait_planner.py`. Add this test at the end of the file:

```python
def test_reset_step_called_at_touchdown():
    """recovery.reset_step() is called exactly once when swing_phase crosses 1.0."""
    from unittest.mock import patch
    from gait_planner import update_gait_planner, SWING_DURATION

    _reset_for_planner()
    # Set phase so the next update pushes phi past 1.0
    shared_state.swing_phase = 1.0 - (0.01 / SWING_DURATION) + 1e-9

    with patch('recovery.reset_step') as mock_reset:
        update_gait_planner()

    mock_reset.assert_called_once()
    # Also verify step_start_time was updated (reset_step writes sim_time)
    # We verify the mock was called; the actual write is tested in recovery tests.
```

- [ ] **Step 2: Run the test to confirm it fails**

```bash
python -m pytest test_gait_planner.py::test_reset_step_called_at_touchdown -v
```

Expected: `FAILED` — `AssertionError: Expected 'reset_step' to have been called once. Called 0 times.`

- [ ] **Step 3: Add `import recovery` and the `reset_step()` call in `gait_planner.py`**

**3a — Add import.** At the top of `gait_planner.py`, after `import kinematics`:

```python
import recovery   # reset_step() called at touchdown to restart step-duration watchdog
```

**3b — Add the call at touchdown.** Find the step-completion block at line ~156:

```python
        # Step completion: φ ≥ 1.0
        if phi >= 1.0:
            shared_state.step_count    += 1
            shared_state.swing_phase    = 0.0
            # Flip swing side
            shared_state.active_swing_side = (
                "right" if side == "left" else "left"
            )
```

Replace with:

```python
        # Step completion: φ ≥ 1.0
        if phi >= 1.0:
            shared_state.step_count    += 1
            shared_state.swing_phase    = 0.0
            # Flip swing side
            shared_state.active_swing_side = (
                "right" if side == "left" else "left"
            )
            recovery.reset_step()  # restart step-duration watchdog: current_step_start_time = sim_time
```

- [ ] **Step 4: Run the new test to confirm it passes**

```bash
python -m pytest test_gait_planner.py::test_reset_step_called_at_touchdown -v
```

Expected: `PASSED`

- [ ] **Step 5: Run the full gait_planner test suite**

```bash
python -m pytest test_gait_planner.py -v
```

Expected: all tests `PASSED` — including all pre-existing tests.

- [ ] **Step 6: Run the full test suite (final regression check)**

```bash
python -m pytest --ignore=test_physics.py --ignore=test_physics_detailed.py -v 2>&1 | tail -30
```

Expected: no `FAILED` lines. Note the count of passing tests — it should be all prior tests plus the 4 new ones (2 in `test_gait_shared_state.py`, 3 in `test_stability_capture_point.py`, 1 in `test_gait_planner.py`).

- [ ] **Step 7: Commit**

```bash
git add gait_planner.py test_gait_planner.py
git commit -m "feat: call recovery.reset_step() at touchdown to restart step watchdog"
```

---

## Verification

After all three tasks, run this end-to-end sanity check to confirm the system can stand still indefinitely without triggering recovery:

```bash
python -m pytest test_stability_capture_point.py::test_stable_when_standing_still \
                 test_gait_shared_state.py::test_capture_point_field_exists_with_default \
                 test_gait_planner.py::test_reset_step_called_at_touchdown -v
```

Expected: all `PASSED`.

**Success criteria (from spec):**
1. `test_stable_when_standing_still` passes — robot standing with confirmed feet and zero velocity classifies `STABLE` → `is_unstable = False` → Priority 1 disarmed
2. `test_unstable_when_high_lateral_velocity` passes — high lateral momentum correctly classified `UNSTABLE`
3. `test_reset_step_called_at_touchdown` passes — step watchdog resets at each touchdown during walking
4. All pre-existing tests continue to pass
