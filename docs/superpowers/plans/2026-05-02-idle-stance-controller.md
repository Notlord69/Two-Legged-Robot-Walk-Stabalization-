# Idle Stance Controller + Left Hip Torque Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the robot stand stably when spawned without `--walk`, and fix the dead left hip pitch torque.

**Architecture:** Two changes — (1) disable PyBullet default velocity motors in `_build_joint_map()` so TORQUE_CONTROL works on all joints, (2) add `_handle_idle_stance()` to `GaitPlannerController` so the gait planner computes IK targets during IDLE instead of returning early.

**Tech Stack:** Python 3.10, PyBullet, existing `kinematics.solve_ik()`, pytest

**Geometry note:** The robot has extreme proportions (L_THIGH=60.7mm, L_SHANK=687mm). The reachable annulus is R_MIN=0.6313m to R_MAX=0.7426m (only 111mm range). "80% of full extension" (0.594m) falls below R_MIN and is unreachable. The plan uses 95% of R_MAX (d=0.7055m), which gives hip=-1.22 rad, knee=1.30 rad — well within ±1.571 URDF limits with stability margin.

---

### Task 1: Disable Default PyBullet Motors

**Files:**
- Modify: `HeartBeat.py:325-343` (`PyBulletRobot._build_joint_map()`)

- [ ] **Step 1: Add motor disable loop after joint map build**

In `HeartBeat.py`, at the end of `_build_joint_map()` (after `self._joint_list = list(self.joint_ids.items())` on line 341), add:

```python
        for jname, jid in self._joint_list:
            p.setJointMotorControl2(
                self.robot_id, jid,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0,
                physicsClientId=self.pc,
            )
```

This disables the default velocity motor on every joint. Joints later set to POSITION_CONTROL (hip roll, hip twist in `apply_control()`) will override this. Joints using TORQUE_CONTROL (hip pitch, knee, ankle) need this to function.

- [ ] **Step 2: Verify no import changes needed**

`pybullet` is already imported as `p` at the top of `HeartBeat.py`. No new imports required.

- [ ] **Step 3: Commit**

```bash
git add HeartBeat.py
git commit -m "fix: disable default PyBullet velocity motors after URDF load

Without this, the default velocity motor on each joint fights against
TORQUE_CONTROL commands. Left_Hip_Forwards showed 0 N·m applied
torque for the entire run despite non-zero WBC output."
```

---

### Task 2: Write Failing Tests for Idle Stance

**Files:**
- Create: `Test_Enviroment/test_idle_stance.py`

- [ ] **Step 1: Write test file with three tests**

Create `Test_Enviroment/test_idle_stance.py`:

```python
"""Tests for idle stance — gait planner must produce IK targets during IDLE."""
import math
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import shared_state, MissionState, StepPhase, ContactState


def _reset_for_idle():
    """Set up shared_state for an IDLE-standing robot."""
    shared_state.reset()
    shared_state.mission_state = MissionState.IDLE
    shared_state.ramp_gain = 0.0
    shared_state.freeze_robot = False
    shared_state.step_phase = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer = 0.0
    shared_state.stance_side = "right"
    shared_state.timing_violation_this_cycle = False
    shared_state.com_position = np.array([0.0, 0.0, 0.75])
    shared_state.com_velocity = np.zeros(3)
    shared_state.link_positions = {
        'Left_Upper_Leg_1':  np.array([0.0, 0.1, 0.75]),
        'Right_Upper_Leg_1': np.array([0.0, -0.1, 0.75]),
    }
    shared_state.left_foot_position = np.array([0.0, 0.1, 0.0])
    shared_state.right_foot_position = np.array([0.0, -0.1, 0.0])
    shared_state.set_contact_state('left', ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)


def test_idle_sets_nonzero_ik_targets():
    """IDLE must produce non-zero IK angles — not (0,0,0,0)."""
    from gait_planner import update_gait_planner
    _reset_for_idle()
    update_gait_planner()
    left = shared_state.ik_left_angles
    right = shared_state.ik_right_angles
    assert any(abs(a) > 0.01 for a in left), f"Left IK still zero: {left}"
    assert any(abs(a) > 0.01 for a in right), f"Right IK still zero: {right}"


def test_idle_ik_within_joint_limits():
    """IDLE stance angles must be within URDF limits (±1.571 rad)."""
    from gait_planner import update_gait_planner
    _reset_for_idle()
    update_gait_planner()
    limit = 1.570796  # rad, URDF ±π/2
    for label, angles in [("left", shared_state.ik_left_angles),
                          ("right", shared_state.ik_right_angles)]:
        for i, a in enumerate(angles):
            assert abs(a) <= limit + 0.001, (
                f"{label}[{i}] = {a:.4f} exceeds ±{limit:.4f}")


def test_idle_fallback_on_ik_failure():
    """If IK raises ValueError, fallback angles must be used (non-zero, within limits)."""
    from gait_planner import update_gait_planner
    _reset_for_idle()
    with patch('gait_planner.kinematics.solve_ik', side_effect=ValueError("mocked")):
        update_gait_planner()
    left = shared_state.ik_left_angles
    right = shared_state.ik_right_angles
    limit = 1.570796
    assert any(abs(a) > 0.01 for a in left), f"Left fallback still zero: {left}"
    assert any(abs(a) > 0.01 for a in right), f"Right fallback still zero: {right}"
    for label, angles in [("left", left), ("right", right)]:
        for i, a in enumerate(angles):
            assert abs(a) <= limit + 0.001, (
                f"Fallback {label}[{i}] = {a:.4f} exceeds limits")
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python3 -m pytest Test_Enviroment/test_idle_stance.py -v`

Expected: All 3 tests FAIL — `test_idle_sets_nonzero_ik_targets` fails because IK targets are still `(0,0,0,0)` after calling `update_gait_planner()` with `mission_state == IDLE`.

- [ ] **Step 3: Commit failing tests**

```bash
git add Test_Enviroment/test_idle_stance.py
git commit -m "test: add failing tests for idle stance IK targets"
```

---

### Task 3: Implement Idle Stance in Gait Planner

**Files:**
- Modify: `gait_planner.py:54-59` (add constant), `gait_planner.py:142-146` (update `update()`), add new method `_handle_idle_stance()`

- [ ] **Step 1: Add stance height constant**

In `gait_planner.py`, after the existing constants block (after line 83, the `_RIGHT_HIP_LINK` line), add:

```python
# Idle stance: 95% of maximum leg reach (R_MAX = L_THIGH + L_SHANK - buffer).
# 80% is unreachable (falls below R_MIN) due to extreme L_THIGH/L_SHANK ratio.
# 95% gives hip=-1.22 rad, knee=1.30 rad — within ±1.571 URDF limits.
IDLE_STANCE_RATIO: float = 0.95  # dimensionless, fraction of kinematics.R_MAX
_IDLE_STANCE_D: float = IDLE_STANCE_RATIO * kinematics.R_MAX  # m, target hip-to-foot distance

# Fallback joint angles if IK fails (pre-computed from 95% R_MAX, foot below hip).
# Left: URDF axis = -X → geometric angles negated.
# Right: URDF axis = +X → geometric angles kept.
_IDLE_FALLBACK_LEFT:  tuple = (0.0, 1.2191, -1.3021, 0.0)   # (roll, hip_pitch, knee, ankle)
_IDLE_FALLBACK_RIGHT: tuple = (0.0, -1.2191, 1.3021, 0.0)   # (roll, hip_pitch, knee, ankle)
```

- [ ] **Step 2: Modify `update()` to call idle stance instead of returning**

In `gait_planner.py`, replace lines 142-146:

```python
    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        if (shared_state.freeze_robot or
                shared_state.mission_state == MissionState.IDLE):
            return
```

with:

```python
    def update(self) -> None:
        """Called once per 100 Hz cycle by HeartBeat.py."""
        if shared_state.freeze_robot:
            return

        if shared_state.mission_state == MissionState.IDLE:
            self._handle_idle_stance()
            return
```

- [ ] **Step 3: Add `_handle_idle_stance()` method**

In `gait_planner.py`, add the new method to `GaitPlannerController` after the `_abort_to_double_support()` method (after line 188):

```python
    def _handle_idle_stance(self) -> None:
        """Compute standing IK for both legs — feet directly below hips.

        Uses IK-derived angles at 95% of max leg reach. Falls back to
        pre-computed angles if IK fails.
        """
        for side in ("left", "right"):
            try:
                foot_target = (0.0, 0.0, -_IDLE_STANCE_D)
                ik_angles = kinematics.solve_ik(foot_target, side)
                angles = (0.0, ik_angles[0], ik_angles[1], ik_angles[2])
            except ValueError:
                angles = _IDLE_FALLBACK_LEFT if side == "left" else _IDLE_FALLBACK_RIGHT

            if side == "left":
                shared_state.ik_left_angles = angles
            else:
                shared_state.ik_right_angles = angles
```

- [ ] **Step 4: Run the idle stance tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python3 -m pytest Test_Enviroment/test_idle_stance.py -v`

Expected: All 3 tests PASS.

- [ ] **Step 5: Run existing gait planner tests to check for regressions**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python3 -m pytest Test_Enviroment/test_gait_planner.py Test_Enviroment/test_gait_planner_fsm.py -v`

Expected: `test_no_update_when_idle` will FAIL because it asserts swing_phase and step_count stay at 0, which they still do — but the test name implies "no update" and now we DO update IK targets. Verify the test still passes as-is since `_handle_idle_stance` doesn't touch `swing_phase` or `step_count`. If it fails, fix in next step.

- [ ] **Step 6: Commit**

```bash
git add gait_planner.py
git commit -m "feat: add idle stance controller to gait planner

Computes IK targets at 95% of max leg reach when mission is IDLE,
instead of returning early with (0,0,0,0). Falls back to pre-computed
angles if IK fails. This prevents the robot from collapsing under
gravity when spawned without --walk."
```

---

### Task 4: Run Full Test Suite

**Files:** None (verification only)

- [ ] **Step 1: Run all tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python3 -m pytest Test_Enviroment/ -v --tb=short 2>&1 | tail -60`

Expected: All tests pass. If `test_no_update_when_idle` fails, update it: it should verify that `swing_phase` and `step_count` stay at 0 during IDLE (the gait FSM doesn't advance), but IK targets ARE now written.

- [ ] **Step 2: Fix any regression (if needed)**

If `test_no_update_when_idle` in `Test_Enviroment/test_gait_planner.py` fails, update it from:

```python
def test_no_update_when_idle():
    """IDLE state: swing_phase stays 0, step_count stays 0."""
    from gait_planner import update_gait_planner
    _reset_for_planner(mission_state=MissionState.IDLE)
    update_gait_planner()
    assert shared_state.swing_phase == 0.0
    assert shared_state.step_count == 0
```

to:

```python
def test_no_gait_advance_when_idle():
    """IDLE state: gait FSM doesn't advance (swing_phase, step_count stay 0)."""
    from gait_planner import update_gait_planner
    _reset_for_planner(mission_state=MissionState.IDLE)
    update_gait_planner()
    assert shared_state.swing_phase == 0.0
    assert shared_state.step_count == 0
```

- [ ] **Step 3: Commit test fix (if needed)**

```bash
git add Test_Enviroment/test_gait_planner.py
git commit -m "test: rename test_no_update_when_idle to reflect gait-only scope"
```

---

### Task 5: Integration Verification

**Files:** None (manual verification)

- [ ] **Step 1: Run the simulation**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python3 main.py --gui --duration 1000`

Observe visually: the robot should stand with bent knees and NOT flip over.

- [ ] **Step 2: Check telemetry output**

After the run completes, find the latest session folder and verify:

```bash
# Find latest session
LATEST=$(ls -td sessions/2026-* | head -1)

# Check COM height stays above 0.7m
awk -F',' 'NR>1{if($6+0 < 0.5) print "COM TOO LOW at t="$1" z="$6}' "$LATEST/telemetry.csv" | head -5

# Check left hip torque is non-zero
awk -F',' 'NR>1 && NR<10{printf "t=%.2f tau_L_hip=%.4f\n", $1, $46}' "$LATEST/telemetry.csv"

# Check contact forces above threshold
awk -F',' 'NR>1 && NR<10{printf "t=%.2f L_force=%.1f R_force=%.1f\n", $1, $13, $14}' "$LATEST/telemetry.csv"

# Check IK targets are non-zero
awk -F',' 'NR==2{printf "IK: L_pitch=%.4f L_knee=%.4f R_pitch=%.4f R_knee=%.4f\n", $27, $28, $31, $32}' "$LATEST/telemetry.csv"
```

Expected:
- COM z stays above 0.5m (ideally above 0.7m)
- `tau_L_hip` shows non-zero values (not 0.0000)
- Contact forces > 5.0 N on both feet after settling
- IK targets show non-zero hip pitch and knee values

- [ ] **Step 3: Check regime monitor**

```bash
awk -F',' 'NR>1 && NR<10{print "t="$1" regime="$3" condition="$4}' "$LATEST/regime.csv"
```

Expected: Condition should be above CRITICAL (e.g., DEGRADED or NOMINAL).

- [ ] **Step 4: Commit summary (if all passes)**

No code change — just verify. If the robot still flips or telemetry looks wrong, investigate WBC gain tuning (out of scope for this plan — separate follow-up).
