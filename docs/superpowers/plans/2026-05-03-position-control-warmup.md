# POSITION_CONTROL Warmup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the TORQUE_CONTROL WBC during warmup with PyBullet POSITION_CONTROL so the robot settles stably at stance before the 100 Hz loop starts.

**Architecture:** Two new methods on `PyBulletRobot` (`enter_position_mode`, `restore_torque_mode`) bracket a simplified warmup loop that omits WBC and apply_control. POSITION_CONTROL is active only during warmup; the real simulation uses TORQUE_CONTROL (WBC, KP=30/KD=10) unchanged.

**Tech Stack:** PyBullet (`p.POSITION_CONTROL`, `p.VELOCITY_CONTROL`), Python 3.10, pytest

---

## Files

| Action | Path | Change |
|--------|------|--------|
| Modify | `HeartBeat.py` | Add `enter_position_mode()`, `restore_torque_mode()` to `PyBulletRobot`; rewrite `_warmup()` in `Siclo1Controller` |
| Create | `Test_Enviroment/test_position_warmup.py` | 8 new tests for the two methods and warmup structure |

---

### Task 1: `enter_position_mode()` on `PyBulletRobot`

**Files:**
- Modify: `HeartBeat.py` (STANCE INITIALISATION section, after `set_stance_pose()`)
- Test: `Test_Enviroment/test_position_warmup.py` (create new)

- [ ] **Step 1: Write failing tests**

Create `Test_Enviroment/test_position_warmup.py`:

```python
"""Tests for POSITION_CONTROL warmup — enter/restore mode methods and warmup structure."""
import inspect
import pytest


def test_enter_position_mode_exists():
    """PyBulletRobot must have enter_position_mode method."""
    from HeartBeat import PyBulletRobot
    assert hasattr(PyBulletRobot, 'enter_position_mode'), \
        "PyBulletRobot missing enter_position_mode"


def test_enter_position_mode_uses_position_control():
    """enter_position_mode must call POSITION_CONTROL with maxVelocity."""
    from HeartBeat import PyBulletRobot
    src = inspect.getsource(PyBulletRobot.enter_position_mode)
    assert 'POSITION_CONTROL' in src, "must use POSITION_CONTROL"
    assert 'maxVelocity' in src, "must set maxVelocity for gentle settling"
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
python3 -m pytest Test_Enviroment/test_position_warmup.py::test_enter_position_mode_exists \
                  Test_Enviroment/test_position_warmup.py::test_enter_position_mode_uses_position_control \
                  -v
```

Expected: 2 FAILED with `AttributeError` or `ImportError`.

- [ ] **Step 3: Add `enter_position_mode()` to `PyBulletRobot` in `HeartBeat.py`**

Insert after the closing `p.resetJointState(...)` block of `set_stance_pose()` (around line 390), before `# SENSOR READING`:

```python
    def enter_position_mode(self, target_angles: dict) -> None:
        """Set all controlled joints to POSITION_CONTROL for warmup settling.

        PyBullet's internal PD is unconditionally stable under contact impulses,
        unlike TORQUE_CONTROL which saturates at velocities > effort_limit/KD.
        maxVelocity=1.0 rad/s gives gentle settling without overshoot.
        force=50.0 N·m holds stance against gravity (hip gravity torque < 10 N·m).
        """
        for jname, angle in target_angles.items():
            jid = self.joint_ids.get(jname)
            if jid is not None:
                p.setJointMotorControl2(
                    self.robot_id, jid,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=angle,
                    targetVelocity=0.0,
                    maxVelocity=1.0,   # rad/s — gentle settling
                    force=50.0,        # N·m — supports stance against gravity
                    physicsClientId=self.pc,
                )
```

- [ ] **Step 4: Run tests to confirm they pass**

```bash
python3 -m pytest Test_Enviroment/test_position_warmup.py::test_enter_position_mode_exists \
                  Test_Enviroment/test_position_warmup.py::test_enter_position_mode_uses_position_control \
                  -v
```

Expected: 2 PASSED.

- [ ] **Step 5: Commit**

```bash
git add HeartBeat.py Test_Enviroment/test_position_warmup.py
git commit -m "feat: add enter_position_mode to PyBulletRobot"
```

---

### Task 2: `restore_torque_mode()` on `PyBulletRobot`

**Files:**
- Modify: `HeartBeat.py` (after `enter_position_mode()`)
- Test: `Test_Enviroment/test_position_warmup.py` (append)

- [ ] **Step 1: Write failing tests**

Append to `Test_Enviroment/test_position_warmup.py`:

```python
def test_restore_torque_mode_exists():
    """PyBulletRobot must have restore_torque_mode method."""
    from HeartBeat import PyBulletRobot
    assert hasattr(PyBulletRobot, 'restore_torque_mode'), \
        "PyBulletRobot missing restore_torque_mode"


def test_restore_torque_mode_uses_velocity_control_force_zero():
    """restore_torque_mode must re-disable motors with VELOCITY_CONTROL force=0."""
    from HeartBeat import PyBulletRobot
    src = inspect.getsource(PyBulletRobot.restore_torque_mode)
    assert 'VELOCITY_CONTROL' in src, "must use VELOCITY_CONTROL to hand joints to WBC"
    assert 'force=0' in src, "must set force=0 to disable motor constraint"
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
python3 -m pytest Test_Enviroment/test_position_warmup.py::test_restore_torque_mode_exists \
                  Test_Enviroment/test_position_warmup.py::test_restore_torque_mode_uses_velocity_control_force_zero \
                  -v
```

Expected: 2 FAILED with `AttributeError`.

- [ ] **Step 3: Add `restore_torque_mode()` to `PyBulletRobot` in `HeartBeat.py`**

Insert immediately after the closing brace of `enter_position_mode()`:

```python
    def restore_torque_mode(self) -> None:
        """Re-disable all joint motors after POSITION_CONTROL warmup.

        Restores VELOCITY_CONTROL force=0 on every joint — identical to
        _build_joint_map(). Hands joints back to the WBC TORQUE_CONTROL path.
        """
        for jname, jid in self._joint_list:
            p.setJointMotorControl2(
                self.robot_id, jid,
                controlMode=p.VELOCITY_CONTROL,
                force=0.0,
                physicsClientId=self.pc,
            )
```

- [ ] **Step 4: Run tests to confirm they pass**

```bash
python3 -m pytest Test_Enviroment/test_position_warmup.py::test_restore_torque_mode_exists \
                  Test_Enviroment/test_position_warmup.py::test_restore_torque_mode_uses_velocity_control_force_zero \
                  -v
```

Expected: 2 PASSED.

- [ ] **Step 5: Commit**

```bash
git add HeartBeat.py Test_Enviroment/test_position_warmup.py
git commit -m "feat: add restore_torque_mode to PyBulletRobot"
```

---

### Task 3: Rewrite `_warmup()` in `Siclo1Controller`

**Files:**
- Modify: `HeartBeat.py` (`_warmup()` method, currently lines 752–781)
- Test: `Test_Enviroment/test_position_warmup.py` (append)

- [ ] **Step 1: Write failing tests**

Append to `Test_Enviroment/test_position_warmup.py`:

```python
def test_warmup_calls_enter_position_mode():
    """_warmup must call enter_position_mode before the loop."""
    from HeartBeat import Siclo1Controller
    src = inspect.getsource(Siclo1Controller._warmup)
    assert 'enter_position_mode' in src, \
        "_warmup must call enter_position_mode"


def test_warmup_calls_restore_torque_mode():
    """_warmup must call restore_torque_mode after the loop."""
    from HeartBeat import Siclo1Controller
    src = inspect.getsource(Siclo1Controller._warmup)
    assert 'restore_torque_mode' in src, \
        "_warmup must call restore_torque_mode"


def test_warmup_does_not_call_wbc_step():
    """_warmup must not call _wbc_step — POSITION_CONTROL handles joints during warmup."""
    from HeartBeat import Siclo1Controller
    src = inspect.getsource(Siclo1Controller._warmup)
    assert '_wbc_step' not in src, \
        "_warmup must not call _wbc_step during POSITION_CONTROL warmup"


def test_warmup_does_not_call_apply_control():
    """_warmup must not call apply_control — POSITION_CONTROL is set once, not per-cycle."""
    from HeartBeat import Siclo1Controller
    src = inspect.getsource(Siclo1Controller._warmup)
    assert 'apply_control' not in src, \
        "_warmup must not call apply_control during POSITION_CONTROL warmup"
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
python3 -m pytest Test_Enviroment/test_position_warmup.py::test_warmup_calls_enter_position_mode \
                  Test_Enviroment/test_position_warmup.py::test_warmup_calls_restore_torque_mode \
                  Test_Enviroment/test_position_warmup.py::test_warmup_does_not_call_wbc_step \
                  Test_Enviroment/test_position_warmup.py::test_warmup_does_not_call_apply_control \
                  -v
```

Expected: first 2 FAILED (`enter/restore_torque_mode` not in source), last 2 PASSED (WBC currently present → will flip after implementation).

- [ ] **Step 3: Replace `_warmup()` in `HeartBeat.py`**

Find the full `_warmup()` method (currently lines ~752–781) and replace it entirely with:

```python
    def _warmup(self, cycles: int) -> None:
        """Settle robot at idle stance using POSITION_CONTROL before 100 Hz loop.

        PyBullet POSITION_CONTROL handles first-contact impulses stably.
        WBC (TORQUE_CONTROL) is NOT active during warmup — joints are owned by
        PyBullet's internal PD until restore_torque_mode() is called.
        The 10 ms timing guard does NOT apply here.
        """
        self.pybullet.enter_position_mode(gait_planner.get_idle_stance_angles())
        for _ in range(cycles):
            self.pybullet.read_sensors()
            self.pybullet.update_link_positions()
            perception.update_perception()
            stability.update_stability(dt=TARGET_DT)
            balance_controller.update_balance()
            sim.interface.step_simulation(self.physics_client)

        self.pybullet.restore_torque_mode()
        gait_planner.reset_gait_planner()
        balance_controller.reset_balance()
```

- [ ] **Step 4: Run all 8 new tests to confirm they pass**

```bash
python3 -m pytest Test_Enviroment/test_position_warmup.py -v
```

Expected: 8 PASSED.

- [ ] **Step 5: Run full test suite to confirm no regressions**

```bash
python3 -m pytest Test_Enviroment/ -x -q
```

Expected: all tests pass (previously 383). Count may be higher with the 8 new tests.

- [ ] **Step 6: Commit**

```bash
git add HeartBeat.py Test_Enviroment/test_position_warmup.py
git commit -m "feat: use POSITION_CONTROL during warmup to avoid WBC saturation at first contact"
```

---

### Task 4: Smoke Test Verification

**Files:** none (read-only verification)

- [ ] **Step 1: Run 1000-cycle simulation**

```bash
cd /home/notlord/ros2_ws/Siclo1_V1
timeout 60 python3 main.py --duration 1000 2>/dev/null
```

- [ ] **Step 2: COM altitude check — must stay below 2m**

```bash
LATEST=$(ls -td sessions/2026-* | head -1)
awk -F',' 'NR>1{z=$6+0; if(z>2.0) print "FAIL: COM_z="z" at t="$1}' \
    "$LATEST/telemetry.csv" | head -5
```

Expected: no output (no FAILs).

- [ ] **Step 3: Torque oscillation check — must not alternate ±100 N·m every cycle**

```bash
LATEST=$(ls -td sessions/2026-* | head -1)
awk -F',' 'NR>1 && NR<12{printf "t=%.2f tau_L_hip=%.1f tau_R_hip=%.1f\n", \
    $1, $46, $49}' "$LATEST/telemetry.csv"
```

Expected: torques do NOT alternate +100/-100 every cycle. Values may vary but should not be locked at ±100.

- [ ] **Step 4: Contact forces after settling**

```bash
LATEST=$(ls -td sessions/2026-* | head -1)
awk -F',' 'NR>1 && $1+0>1.0 && $1+0<1.5{printf "t=%.2f L=%.1fN R=%.1fN COM_z=%.3f\n", \
    $1, $13, $14, $6}' "$LATEST/telemetry.csv" | head -5
```

Expected: `L > 5 N` and `R > 5 N` (both feet bearing weight), `COM_z` between 0.65–0.90.

- [ ] **Step 5: If all checks pass, commit smoke test results as a note in the spec**

Update `docs/superpowers/specs/2026-05-03-position-control-warmup-design.md` — add a one-line "Verified:" entry under the Verification section with the date and that all checks passed.

```bash
git add docs/superpowers/specs/2026-05-03-position-control-warmup-design.md
git commit -m "docs: mark position-control warmup spec as verified"
```

- [ ] **Step 6: If any check fails, stop and report the failing check with the telemetry data**

Do not attempt fixes without a new design session.
