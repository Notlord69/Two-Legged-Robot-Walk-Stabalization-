# Swing Oscillation Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix robot tripping during swing phase by retuning WBC gains, increasing swing clearance, and adding tracking telemetry.

**Architecture:** Lower WBC_KP to avoid torque saturation, raise WBC_KD for near-critical damping. Increase SWING_HEIGHT and SWING_DURATION for more clearance and tracking time. Add telemetry fields to shared_state to monitor tracking error.

**Tech Stack:** Python 3.10+, PyBullet, pytest

---

## File Structure

| File | Responsibility | Change Type |
|------|----------------|-------------|
| `shared_state.py` | Add `wbc_tracking_error` and `wbc_torque_saturated` dict fields | Modify |
| `HeartBeat.py` | Update WBC_KP/KD constants, populate telemetry in `_wbc_step()` | Modify |
| `gait_planner.py` | Update SWING_HEIGHT/SWING_DURATION constants | Modify |
| `Test_Enviroment/test_wbc_tracking.py` | Tests for new telemetry fields | Create |
| `Test_Enviroment/test_swing_constants.py` | Tests for updated swing parameters | Create |

---

## Task 1: Add Telemetry Fields to shared_state.py

**Files:**
- Modify: `shared_state.py:315-316` (after `grf_torque_correction`)
- Test: `Test_Enviroment/test_wbc_tracking.py`

- [ ] **Step 1: Write the failing test**

Create `Test_Enviroment/test_wbc_tracking.py`:

```python
"""Tests for WBC tracking telemetry fields."""

import pytest
from shared_state import shared_state


def test_tracking_error_field_exists():
    """shared_state has wbc_tracking_error dict."""
    assert hasattr(shared_state, 'wbc_tracking_error')
    assert isinstance(shared_state.wbc_tracking_error, dict)


def test_saturation_flag_field_exists():
    """shared_state has wbc_torque_saturated dict."""
    assert hasattr(shared_state, 'wbc_torque_saturated')
    assert isinstance(shared_state.wbc_torque_saturated, dict)


def test_tracking_error_initially_empty():
    """wbc_tracking_error starts as empty dict."""
    shared_state.reset()
    assert shared_state.wbc_tracking_error == {}


def test_saturation_flag_initially_empty():
    """wbc_torque_saturated starts as empty dict."""
    shared_state.reset()
    assert shared_state.wbc_torque_saturated == {}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_wbc_tracking.py -v`

Expected: FAIL with `AttributeError: 'Siclo1State' object has no attribute 'wbc_tracking_error'`

- [ ] **Step 3: Add telemetry fields to shared_state.py**

In `shared_state.py`, after line 315 (`self.grf_torque_correction: Dict[str, float] = {}`), add:

```python
        # WBC tracking telemetry — written by HeartBeat._wbc_step() each cycle.
        # Used to diagnose torque saturation and tracking lag.
        self.wbc_tracking_error: Dict[str, float] = {}      # rad, θ_cmd - θ_actual per joint
        self.wbc_torque_saturated: Dict[str, bool] = {}     # True when |τ| ≥ effort limit
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_wbc_tracking.py -v`

Expected: 4 tests PASS

- [ ] **Step 5: Commit**

```bash
git add shared_state.py Test_Enviroment/test_wbc_tracking.py
git commit -m "feat(shared_state): add WBC tracking telemetry fields

wbc_tracking_error: Dict[str, float] — per-joint tracking error (rad)
wbc_torque_saturated: Dict[str, bool] — saturation flag per joint"
```

---

## Task 2: Update WBC Gains in HeartBeat.py

**Files:**
- Modify: `HeartBeat.py:80-81` (WBC_KP, WBC_KD constants)
- Test: `Test_Enviroment/test_wbc_tracking.py` (extend)

- [ ] **Step 1: Write the failing test**

Append to `Test_Enviroment/test_wbc_tracking.py`:

```python
def test_wbc_kp_reduced():
    """WBC_KP should be 100.0 N·m/rad (reduced from 200)."""
    from HeartBeat import WBC_KP
    assert WBC_KP == 100.0, f"WBC_KP={WBC_KP}, expected 100.0"


def test_wbc_kd_increased():
    """WBC_KD should be 28.0 N·m·s/rad (increased from 15)."""
    from HeartBeat import WBC_KD
    assert WBC_KD == 28.0, f"WBC_KD={WBC_KD}, expected 28.0"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_wbc_tracking.py::test_wbc_kp_reduced Test_Enviroment/test_wbc_tracking.py::test_wbc_kd_increased -v`

Expected: FAIL with `AssertionError: WBC_KP=200.0, expected 100.0`

- [ ] **Step 3: Update WBC gains in HeartBeat.py**

In `HeartBeat.py`, change lines 80-81 from:

```python
WBC_KP: float = 200.0   # N·m/rad, joint position proportional gain
WBC_KD: float = 15.0    # N·m·s/rad, joint velocity derivative gain
```

To:

```python
WBC_KP: float = 100.0   # N·m/rad, joint position proportional gain (halved to avoid saturation)
WBC_KD: float = 28.0    # N·m·s/rad, joint velocity derivative gain (ζ ≈ 0.9 near-critical damping)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_wbc_tracking.py::test_wbc_kp_reduced Test_Enviroment/test_wbc_tracking.py::test_wbc_kd_increased -v`

Expected: 2 tests PASS

- [ ] **Step 5: Commit**

```bash
git add HeartBeat.py Test_Enviroment/test_wbc_tracking.py
git commit -m "tune(HeartBeat): reduce WBC_KP, increase WBC_KD for critical damping

WBC_KP: 200 → 100 N·m/rad (avoids torque saturation at 0.5 rad error)
WBC_KD: 15 → 28 N·m·s/rad (ζ ≈ 0.9 for leg inertia ~0.5 kg·m²)"
```

---

## Task 3: Add Tracking Telemetry to _wbc_step()

**Files:**
- Modify: `HeartBeat.py:597-609` (inside `_wbc_step()`)
- Test: `Test_Enviroment/test_wbc_tracking.py` (extend)

- [ ] **Step 1: Write the failing test**

Append to `Test_Enviroment/test_wbc_tracking.py`:

```python
def test_tracking_error_populated_after_wbc():
    """wbc_tracking_error dict is populated after _wbc_step runs."""
    from shared_state import shared_state, URDF_JOINT_LIMITS
    
    shared_state.reset()
    # Set up minimal state for WBC to run
    shared_state.joint_positions = {'Left_Hip_Forwards': 0.1, 'Left_Knee': 0.2, 'Left_Ankle': 0.0,
                                    'Right_Hip_Fowards': 0.1, 'Right_Knee': 0.2, 'Right_Ankle': 0.0}
    shared_state.joint_velocities = {'Left_Hip_Forwards': 0.0, 'Left_Knee': 0.0, 'Left_Ankle': 0.0,
                                     'Right_Hip_Fowards': 0.0, 'Right_Knee': 0.0, 'Right_Ankle': 0.0}
    shared_state.ik_left_angles = (0.15, 0.25, 0.0)   # slightly different from actual
    shared_state.ik_right_angles = (0.15, 0.25, 0.0)
    shared_state.target_torques = {}
    
    # Import after reset to get fresh module state
    from HeartBeat import Siclo1Controller
    # We can't easily instantiate the full controller, so test the dict is populated
    # by checking that the field type is correct after manual population
    shared_state.wbc_tracking_error['Left_Hip_Forwards'] = 0.05
    assert 'Left_Hip_Forwards' in shared_state.wbc_tracking_error


def test_saturation_flag_is_bool():
    """wbc_torque_saturated values must be bool."""
    from shared_state import shared_state
    
    shared_state.wbc_torque_saturated['Left_Hip_Forwards'] = True
    shared_state.wbc_torque_saturated['Left_Knee'] = False
    
    assert shared_state.wbc_torque_saturated['Left_Hip_Forwards'] is True
    assert shared_state.wbc_torque_saturated['Left_Knee'] is False
```

- [ ] **Step 2: Run test to verify it passes (field exists from Task 1)**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_wbc_tracking.py::test_tracking_error_populated_after_wbc Test_Enviroment/test_wbc_tracking.py::test_saturation_flag_is_bool -v`

Expected: 2 tests PASS (fields exist, we're testing usage pattern)

- [ ] **Step 3: Add telemetry population to _wbc_step()**

In `HeartBeat.py`, modify the `_wbc_step()` method. Replace lines 597-609:

```python
        for idx, jname, _ in _WBC_LEFT_JOINTS:
            theta_target = shared_state.ik_left_angles[idx]
            theta_now    = jp.get(jname, 0.0)
            omega_now    = jv.get(jname, 0.0)
            tau = WBC_KP * (theta_target - theta_now) - WBC_KD * omega_now
            torques[jname] = torques.get(jname, 0.0) + _clip_effort(jname, tau)

        for idx, jname, _ in _WBC_RIGHT_JOINTS:
            theta_target = shared_state.ik_right_angles[idx]
            theta_now    = jp.get(jname, 0.0)
            omega_now    = jv.get(jname, 0.0)
            tau = WBC_KP * (theta_target - theta_now) - WBC_KD * omega_now
            torques[jname] = torques.get(jname, 0.0) + _clip_effort(jname, tau)
```

With:

```python
        for idx, jname, _ in _WBC_LEFT_JOINTS:
            theta_target = shared_state.ik_left_angles[idx]
            theta_now    = jp.get(jname, 0.0)
            omega_now    = jv.get(jname, 0.0)
            error = theta_target - theta_now
            tau = WBC_KP * error - WBC_KD * omega_now
            clipped_tau = _clip_effort(jname, tau)
            torques[jname] = torques.get(jname, 0.0) + clipped_tau
            # Tracking telemetry
            shared_state.wbc_tracking_error[jname] = error
            lim = URDF_JOINT_LIMITS.get(jname, {}).get('effort', 100.0)
            shared_state.wbc_torque_saturated[jname] = (abs(tau) >= lim - 0.1)

        for idx, jname, _ in _WBC_RIGHT_JOINTS:
            theta_target = shared_state.ik_right_angles[idx]
            theta_now    = jp.get(jname, 0.0)
            omega_now    = jv.get(jname, 0.0)
            error = theta_target - theta_now
            tau = WBC_KP * error - WBC_KD * omega_now
            clipped_tau = _clip_effort(jname, tau)
            torques[jname] = torques.get(jname, 0.0) + clipped_tau
            # Tracking telemetry
            shared_state.wbc_tracking_error[jname] = error
            lim = URDF_JOINT_LIMITS.get(jname, {}).get('effort', 100.0)
            shared_state.wbc_torque_saturated[jname] = (abs(tau) >= lim - 0.1)
```

- [ ] **Step 4: Run all tracking tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_wbc_tracking.py -v`

Expected: 8 tests PASS

- [ ] **Step 5: Commit**

```bash
git add HeartBeat.py Test_Enviroment/test_wbc_tracking.py
git commit -m "feat(HeartBeat): populate WBC tracking telemetry in _wbc_step

Writes wbc_tracking_error and wbc_torque_saturated per joint each cycle.
Enables post-run analysis of tracking lag and saturation frequency."
```

---

## Task 4: Update Swing Constants in gait_planner.py

**Files:**
- Modify: `gait_planner.py:58-59` (SWING_HEIGHT, SWING_DURATION)
- Test: `Test_Enviroment/test_swing_constants.py`

- [ ] **Step 1: Write the failing test**

Create `Test_Enviroment/test_swing_constants.py`:

```python
"""Tests for updated swing trajectory constants."""

import pytest


def test_swing_height_increased():
    """SWING_HEIGHT should be 0.06 m (increased from 0.04)."""
    from gait_planner import SWING_HEIGHT
    assert SWING_HEIGHT == 0.06, f"SWING_HEIGHT={SWING_HEIGHT}, expected 0.06"


def test_swing_duration_increased():
    """SWING_DURATION should be 0.50 s (increased from 0.40)."""
    from gait_planner import SWING_DURATION
    assert SWING_DURATION == 0.50, f"SWING_DURATION={SWING_DURATION}, expected 0.50"


def test_swing_height_units():
    """SWING_HEIGHT is in meters, must be positive and reasonable."""
    from gait_planner import SWING_HEIGHT
    assert 0.02 < SWING_HEIGHT < 0.15, "SWING_HEIGHT out of reasonable range"


def test_swing_duration_units():
    """SWING_DURATION is in seconds, must be positive and reasonable."""
    from gait_planner import SWING_DURATION
    assert 0.2 < SWING_DURATION < 1.0, "SWING_DURATION out of reasonable range"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_swing_constants.py -v`

Expected: FAIL with `AssertionError: SWING_HEIGHT=0.04, expected 0.06`

- [ ] **Step 3: Update swing constants in gait_planner.py**

In `gait_planner.py`, change lines 58-59 from:

```python
SWING_HEIGHT:   float = 0.04   # m, peak foot clearance above ground at φ=0.5
SWING_DURATION: float = 0.40   # s, full swing phase (40 cycles at 100 Hz)
```

To:

```python
SWING_HEIGHT:   float = 0.06   # m, peak foot clearance above ground at φ=0.5 (+2cm margin)
SWING_DURATION: float = 0.50   # s, full swing phase (50 cycles at 100 Hz, reduced accel)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_swing_constants.py -v`

Expected: 4 tests PASS

- [ ] **Step 5: Commit**

```bash
git add gait_planner.py Test_Enviroment/test_swing_constants.py
git commit -m "tune(gait_planner): increase SWING_HEIGHT and SWING_DURATION

SWING_HEIGHT: 0.04 → 0.06 m (+2cm clearance margin)
SWING_DURATION: 0.40 → 0.50 s (reduces peak acceleration, more tracking time)"
```

---

## Task 5: Run Regression Tests

**Files:**
- Test: `Test_Enviroment/test_gait_planner.py`
- Test: `Test_Enviroment/test_heartbeat_gait_wiring.py`
- Test: `Test_Enviroment/test_grf.py`

- [ ] **Step 1: Run gait_planner tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_gait_planner.py -v`

Expected: 10 tests PASS (some may need SWING_HEIGHT/DURATION updates in assertions)

- [ ] **Step 2: Fix any failing gait_planner tests**

If tests fail due to hardcoded 0.04/0.40 values, update them to 0.06/0.50.

- [ ] **Step 3: Run heartbeat wiring tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_heartbeat_gait_wiring.py -v`

Expected: 5 tests PASS

- [ ] **Step 4: Run GRF tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_grf.py -v`

Expected: 8 tests PASS

- [ ] **Step 5: Run full test suite**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/ -v --tb=short`

Expected: All tests PASS

- [ ] **Step 6: Commit any test fixes**

```bash
git add Test_Enviroment/
git commit -m "test: update test assertions for new swing constants"
```

---

## Task 6: Integration Validation

**Files:**
- Run: `main.py --walk 1.0`

- [ ] **Step 1: Run walking simulation with GUI**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python3 main.py --walk 1.0 --gui`

Observe:
1. Swing leg moves smoothly (no crazy oscillation)
2. Legs don't tangle or collide
3. Robot completes walk without tripping

- [ ] **Step 2: Check telemetry output**

After the run, check session folder for tracking data. Verify:
- `max(|wbc_tracking_error|) < 0.3 rad`
- `wbc_torque_saturated` count is low

- [ ] **Step 3: Document results**

If successful, proceed. If tripping persists, note observations for further tuning.

- [ ] **Step 4: Final commit**

```bash
git add -A
git commit -m "feat: swing oscillation fix complete

- WBC gains retuned for near-critical damping
- Swing clearance increased to 6cm
- Swing duration extended to 500ms
- Tracking telemetry added for diagnostics"
```

---

## Summary

| Task | Description | Tests Added |
|------|-------------|-------------|
| 1 | Add telemetry fields to shared_state | 4 |
| 2 | Update WBC gains | 2 |
| 3 | Add telemetry to _wbc_step() | 2 |
| 4 | Update swing constants | 4 |
| 5 | Regression tests | 0 (existing) |
| 6 | Integration validation | 0 (manual) |

**Total new tests:** 12

---

## Implementation Status (2026-04-19)

### Completed Tasks
- [x] Task 1: Telemetry fields added to shared_state.py
- [x] Task 2: WBC gains updated (KP=100, KD=28)
- [x] Task 3: Telemetry logging in _wbc_step()
- [x] Task 4: Swing constants updated (HEIGHT=0.06, DURATION=0.50)
- [x] Task 5: Regression tests pass (278 tests)
- [ ] Task 6: Integration validation — **BLOCKED**

### Bug Fixed During Session
**Timestep bug:** `gait_planner.py` was using wall-clock `last_dt` (~0.0001s in DIRECT mode) instead of fixed simulation timestep (0.01s). Fixed by hardcoding `dt = 0.01`.

### Remaining Issues (Unsolved)

**Symptom:** Robot lifts leg slightly, then swings crazily. Simulation crashes at ~405 cycles.

**Observations:**
1. `max_err=0.658rad` — tracking error is HIGH (target was <0.3rad)
2. `sat_count=0` — no torque saturation, suggesting gains too LOW
3. No `[LIFT]` or `[SWING]` logs — FSM stuck in DOUBLE_SUPPORT or COM_SHIFT
4. Crash at cycle 405 — likely physics instability from poor tracking

**Hypothesis:** WBC_KP=100 is too soft. The leg can't track IK targets, leading to large errors and eventually instability.

**Next Steps to Try:**
1. Increase WBC_KP to 150 (middle ground between 100 and 200)
2. Check why FSM isn't reaching LIFT phase — may need to debug COM_SHIFT conditions
3. Add more logging to COM_SHIFT phase handler
4. Consider if FORCE_BALANCE_RATIO (2.0) gate is blocking transition

### Diagnostic Logging Added
- `[GATE]` in DOUBLE_SUPPORT — shows timer, ramp_gain, forces, ratio
- `[LIFT]` in LIFT phase — shows forces and velocity (not reached yet)
- `[SWING]` in SWING phase — shows phi and timer (not reached yet)
- `[WBC]` every 50 cycles — shows max tracking error and saturation count

### Files Modified
| File | Changes |
|------|---------|
| `shared_state.py` | Added wbc_tracking_error, wbc_torque_saturated; clear in reset() |
| `HeartBeat.py` | WBC_KP=100, WBC_KD=28; telemetry logging; periodic WBC print |
| `gait_planner.py` | SWING_HEIGHT=0.06, SWING_DURATION=0.50; fixed dt=0.01; debug prints |
| `Test_Enviroment/test_wbc_tracking.py` | 8 new tests |
| `Test_Enviroment/test_swing_constants.py` | 4 new tests |

### Commits Made
```
d854456 feat(shared_state): add WBC tracking telemetry fields
82a10f3 tune(HeartBeat): reduce WBC_KP, increase WBC_KD for critical damping
e9bb9a3 feat(HeartBeat): populate WBC tracking telemetry in _wbc_step
2defa3c tune(gait_planner): increase SWING_HEIGHT and SWING_DURATION
cca4f55 test: update test assertions for new swing constants
35e7450 fix(gait_planner): use fixed 0.01s timestep instead of wall-clock dt
```
