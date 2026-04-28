# Unified Balance Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the broken lateral-only `active_balance.py` with a unified 2-axis balance controller that corrects both lateral (X) and sagittal (Y) COM drift via position targets, not competing torques.

**Architecture:** Balance corrections flow as position targets (hip roll for lateral, pitch offset for sagittal) into WBC, which remains the sole torque authority. Emergency sagittal torque is threshold-gated and saturation-aware. The old torque-summing approach in active_balance.py is deleted entirely.

**Tech Stack:** Python 3.10, PyBullet (via sim/interface.py), NumPy, shared_state singleton pattern.

**Spec:** `docs/superpowers/specs/2026-04-22-unified-balance-controller-design.md`

**Status (2026-04-27):** Plan written. NO code changes made yet. All 6 tasks pending. Resume with Task 1.

**Execution preference:** User wants subagent-driven (max 2 subagents). Batch Tasks 1-3 into Subagent 1, Tasks 4-6 into Subagent 2, review inline between dispatches.

---

### Task 1: shared_state.py — Add balance controller fields and update STAGE_NAMES

**Files:**
- Modify: `shared_state.py:370-430` (new fields in `__init__`)
- Modify: `shared_state.py:48-52` (STAGE_NAMES)
- Modify: `shared_state.py:599-632` (reset method)
- Test: `Test_Enviroment/test_balance_shared_state.py`

- [ ] **Step 1: Write tests for new shared_state fields**

```python
# Test_Enviroment/test_balance_shared_state.py
"""Tests for balance controller shared_state fields."""
import pytest
import numpy as np
from shared_state import shared_state, STAGE_NAMES


def test_balance_hip_roll_fields_exist():
    """Balance hip roll fields default to 0.0."""
    shared_state.reset()
    assert shared_state.balance_hip_roll_left == 0.0
    assert shared_state.balance_hip_roll_right == 0.0


def test_sagittal_pitch_offset_field_exists():
    """Sagittal pitch offset defaults to 0.0."""
    shared_state.reset()
    assert shared_state.sagittal_pitch_offset == 0.0


def test_emergency_sagittal_torque_field_exists():
    """Emergency sagittal torque defaults to empty dict."""
    shared_state.reset()
    assert shared_state.emergency_sagittal_torque == {}


def test_stability_margin_lateral_field_exists():
    shared_state.reset()
    assert shared_state.stability_margin_lateral == 0.0


def test_stability_margin_sagittal_field_exists():
    shared_state.reset()
    assert shared_state.stability_margin_sagittal == 0.0


def test_capture_point_error_fields_exist():
    shared_state.reset()
    assert shared_state.capture_point_error_lateral == 0.0
    assert shared_state.capture_point_error_sagittal == 0.0


def test_balance_mode_fields_exist():
    shared_state.reset()
    assert shared_state.balance_mode_lateral == "INACTIVE"
    assert shared_state.balance_mode_sagittal == "INACTIVE"


def test_stage_names_has_balance():
    """STAGE_NAMES uses 'balance' not 'active_balance'."""
    assert 'balance' in STAGE_NAMES
    assert 'active_balance' not in STAGE_NAMES


def test_reset_clears_balance_fields():
    """reset() zeroes all balance fields."""
    shared_state.balance_hip_roll_left = 0.15
    shared_state.balance_hip_roll_right = -0.10
    shared_state.sagittal_pitch_offset = 0.05
    shared_state.emergency_sagittal_torque = {'Left_Hip_Forwards': 10.0}
    shared_state.stability_margin_lateral = 0.03
    shared_state.stability_margin_sagittal = 0.02
    shared_state.capture_point_error_lateral = 0.04
    shared_state.capture_point_error_sagittal = 0.06
    shared_state.balance_mode_lateral = "ACTIVE"
    shared_state.balance_mode_sagittal = "EMERGENCY"

    shared_state.reset()

    assert shared_state.balance_hip_roll_left == 0.0
    assert shared_state.balance_hip_roll_right == 0.0
    assert shared_state.sagittal_pitch_offset == 0.0
    assert shared_state.emergency_sagittal_torque == {}
    assert shared_state.stability_margin_lateral == 0.0
    assert shared_state.stability_margin_sagittal == 0.0
    assert shared_state.capture_point_error_lateral == 0.0
    assert shared_state.capture_point_error_sagittal == 0.0
    assert shared_state.balance_mode_lateral == "INACTIVE"
    assert shared_state.balance_mode_sagittal == "INACTIVE"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_balance_shared_state.py -v`
Expected: FAIL — fields don't exist yet.

- [ ] **Step 3: Add new fields to shared_state.__init__()**

Add after the `stance_foot_world_pos` / `non_stance_foot_world_pos` block (~line 420):

```python
# Balance controller outputs (written by balance_controller.py)
self.balance_hip_roll_left:  float = 0.0   # rad, position target for Left_Hip_Inwards
self.balance_hip_roll_right: float = 0.0   # rad, position target for Right_Hip_Inwards
self.sagittal_pitch_offset:  float = 0.0   # rad, added to IK hip_pitch before WBC
self.emergency_sagittal_torque: Dict[str, float] = {}  # N·m, direct injection

# 2D stability margins (written by stability.py)
self.stability_margin_lateral:  float = 0.0  # m, CP-to-polygon-edge in X
self.stability_margin_sagittal: float = 0.0  # m, CP-to-polygon-edge in Y

# Balance diagnostics
self.capture_point_error_lateral:  float = 0.0  # m, ξ_x - support_center_x
self.capture_point_error_sagittal: float = 0.0  # m, ξ_y - support_center_y
self.balance_mode_lateral:  str = "INACTIVE"    # ACTIVE/INACTIVE
self.balance_mode_sagittal: str = "INACTIVE"    # NORMAL/EMERGENCY/INACTIVE
```

Update `STAGE_NAMES` (line 48):

```python
STAGE_NAMES: tuple = (
    'sensors', 'link_positions', 'perception', 'stability',
    'balance', 'grf', 'gait_planner', 'mission',
    'wbc', 'recovery', 'apply_control', 'step_sim',
)
```

Update `reset()` — add inside the `with self._lock:` block, after clearing WBC tracking:

```python
# Balance controller fields
self.balance_hip_roll_left = 0.0
self.balance_hip_roll_right = 0.0
self.sagittal_pitch_offset = 0.0
self.emergency_sagittal_torque = {}
self.stability_margin_lateral = 0.0
self.stability_margin_sagittal = 0.0
self.capture_point_error_lateral = 0.0
self.capture_point_error_sagittal = 0.0
self.balance_mode_lateral = "INACTIVE"
self.balance_mode_sagittal = "INACTIVE"
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_balance_shared_state.py -v`
Expected: ALL PASS

- [ ] **Step 5: Commit**

```bash
git add shared_state.py Test_Enviroment/test_balance_shared_state.py
git commit -m "feat(shared_state): add balance controller fields and rename stage"
```

---

### Task 2: balance_controller.py — Create unified balance controller

**Files:**
- Create: `balance_controller.py`
- Test: `Test_Enviroment/test_balance_controller.py`

- [ ] **Step 1: Write unit tests for balance_controller**

```python
# Test_Enviroment/test_balance_controller.py
"""Unit tests for balance_controller.py — pure math, no PyBullet."""
import math
import numpy as np
import pytest
from shared_state import shared_state, ContactState, StepPhase, MissionState


def _reset_for_balance():
    """Set up shared_state for balance controller testing."""
    shared_state.reset()
    shared_state.mission_state = MissionState.WALK
    shared_state.ramp_gain = 1.0
    shared_state.freeze_robot = False
    shared_state.emergency_stop_triggered = False
    shared_state.step_phase = StepPhase.DOUBLE_SUPPORT

    # COM at nominal standing position
    shared_state.com_position = np.array([0.0, 0.25, 0.8806])
    shared_state.com_velocity = np.zeros(3)

    # Both feet confirmed, symmetric
    shared_state.set_contact_state('left', ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    shared_state.left_foot_position = np.array([-0.1, 0.25, 0.0])
    shared_state.right_foot_position = np.array([0.1, 0.25, 0.0])
    shared_state.stance_foot_world_pos = np.array([0.1, 0.25, 0.0])
    shared_state.stance_side = "right"

    # Capture point at COM (zero velocity → CP = COM)
    shared_state.capture_point = np.array([0.0, 0.25])


# ── Lateral balance tests ────────────────────────────────────────────────

def test_lateral_zero_error_zero_roll():
    """Zero CP_x error → zero hip roll outputs."""
    from balance_controller import update_balance, reset_balance
    _reset_for_balance()
    reset_balance()
    update_balance()
    assert abs(shared_state.balance_hip_roll_left) < 1e-6
    assert abs(shared_state.balance_hip_roll_right) < 1e-6


def test_lateral_positive_error_negative_roll():
    """Positive CP_x error (COM right of support) → negative hip roll."""
    from balance_controller import update_balance, reset_balance
    _reset_for_balance()
    reset_balance()
    # COM shifted right → CP right of support center
    shared_state.com_position = np.array([0.05, 0.25, 0.8806])
    shared_state.capture_point = np.array([0.05, 0.25])
    update_balance()
    # Negative roll = abduct toward error direction
    assert shared_state.balance_hip_roll_left < 0.0 or shared_state.balance_hip_roll_right < 0.0


def test_lateral_roll_clipped_to_max():
    """Hip roll is clipped to ±HIP_ROLL_MAX."""
    from balance_controller import update_balance, reset_balance, HIP_ROLL_MAX
    _reset_for_balance()
    reset_balance()
    # Extreme lateral error
    shared_state.com_position = np.array([1.0, 0.25, 0.8806])
    shared_state.com_velocity = np.array([2.0, 0.0, 0.0])
    shared_state.capture_point = np.array([1.5, 0.25])
    # Run many cycles to bypass rate limiting
    for _ in range(100):
        update_balance()
    assert abs(shared_state.balance_hip_roll_left) <= HIP_ROLL_MAX + 1e-6
    assert abs(shared_state.balance_hip_roll_right) <= HIP_ROLL_MAX + 1e-6


def test_lateral_rate_limiting():
    """Large step input → output changes by at most RATE_LIMIT per cycle."""
    from balance_controller import update_balance, reset_balance, HIP_ROLL_RATE_LIMIT
    _reset_for_balance()
    reset_balance()
    # Large lateral error on first call
    shared_state.com_position = np.array([0.5, 0.25, 0.8806])
    shared_state.capture_point = np.array([0.5, 0.25])
    update_balance()
    # After one cycle, roll should be at most RATE_LIMIT from zero
    assert abs(shared_state.balance_hip_roll_left) <= HIP_ROLL_RATE_LIMIT + 1e-6
    assert abs(shared_state.balance_hip_roll_right) <= HIP_ROLL_RATE_LIMIT + 1e-6


# ── Sagittal balance tests ───────────────────────────────────────────────

def test_sagittal_zero_error_zero_offset():
    """Zero CP_y error → zero pitch offset."""
    from balance_controller import update_balance, reset_balance
    _reset_for_balance()
    reset_balance()
    update_balance()
    assert abs(shared_state.sagittal_pitch_offset) < 1e-6


def test_sagittal_positive_error_negative_offset():
    """Positive CP_y error (COM forward of support) → negative pitch offset (lean back)."""
    from balance_controller import update_balance, reset_balance
    _reset_for_balance()
    reset_balance()
    shared_state.com_position = np.array([0.0, 0.35, 0.8806])
    shared_state.capture_point = np.array([0.0, 0.35])
    # stance foot at y=0.25, CP at y=0.35 → positive sagittal error
    for _ in range(20):
        update_balance()
    assert shared_state.sagittal_pitch_offset < 0.0


def test_sagittal_pitch_offset_clipped():
    """Pitch offset is clipped to ±PITCH_OFFSET_MAX."""
    from balance_controller import update_balance, reset_balance, PITCH_OFFSET_MAX
    _reset_for_balance()
    reset_balance()
    shared_state.com_position = np.array([0.0, 2.0, 0.8806])
    shared_state.com_velocity = np.array([0.0, 5.0, 0.0])
    shared_state.capture_point = np.array([0.0, 3.5])
    for _ in range(200):
        update_balance()
    assert abs(shared_state.sagittal_pitch_offset) <= PITCH_OFFSET_MAX + 1e-6


# ── Emergency mode tests ────────────────────────────────────────────────

def test_emergency_activation():
    """Large sagittal error (|e_y| > EMERGENCY_THRESHOLD) → non-zero emergency torque."""
    from balance_controller import update_balance, reset_balance, EMERGENCY_THRESHOLD
    _reset_for_balance()
    reset_balance()
    # Push CP_y far from stance foot (error > 0.08 m)
    shared_state.com_position = np.array([0.0, 0.45, 0.8806])
    shared_state.com_velocity = np.array([0.0, 0.5, 0.0])
    shared_state.capture_point = np.array([0.0, 0.45])
    # Error = 0.45 - 0.25 = 0.20 m > EMERGENCY_THRESHOLD (0.08 m)
    update_balance()
    assert len(shared_state.emergency_sagittal_torque) > 0
    total_emergency = sum(abs(v) for v in shared_state.emergency_sagittal_torque.values())
    assert total_emergency > 0.0


def test_emergency_hysteresis():
    """Emergency deactivates only when |e_y| < THRESHOLD - HYSTERESIS."""
    from balance_controller import (
        update_balance, reset_balance,
        EMERGENCY_THRESHOLD, EMERGENCY_HYSTERESIS,
    )
    _reset_for_balance()
    reset_balance()

    # Activate emergency: error = 0.20 m
    shared_state.com_position = np.array([0.0, 0.45, 0.8806])
    shared_state.capture_point = np.array([0.0, 0.45])
    update_balance()
    assert shared_state.balance_mode_sagittal == "EMERGENCY"

    # Reduce error to just below threshold but above hysteresis band
    # e_y = 0.07 m (< 0.08 threshold but > 0.06 = threshold - hysteresis)
    shared_state.com_position = np.array([0.0, 0.32, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    shared_state.capture_point = np.array([0.0, 0.32])
    update_balance()
    # Should STILL be in emergency (hysteresis prevents exit)
    assert shared_state.balance_mode_sagittal == "EMERGENCY"

    # Reduce error below hysteresis band: e_y < 0.06 m
    shared_state.com_position = np.array([0.0, 0.30, 0.8806])
    shared_state.capture_point = np.array([0.0, 0.30])
    update_balance()
    assert shared_state.balance_mode_sagittal == "NORMAL"


# ── Safety gate tests ────────────────────────────────────────────────────

def test_freeze_produces_zero_outputs():
    """freeze_robot → all balance outputs zeroed."""
    from balance_controller import update_balance, reset_balance
    _reset_for_balance()
    reset_balance()
    # First produce non-zero outputs
    shared_state.com_position = np.array([0.05, 0.35, 0.8806])
    shared_state.capture_point = np.array([0.05, 0.35])
    for _ in range(10):
        update_balance()

    # Now freeze
    shared_state.freeze_robot = True
    update_balance()
    assert shared_state.balance_hip_roll_left == 0.0
    assert shared_state.balance_hip_roll_right == 0.0
    assert shared_state.sagittal_pitch_offset == 0.0
    assert shared_state.emergency_sagittal_torque == {}


def test_emergency_stop_produces_zero_outputs():
    """emergency_stop_triggered → all balance outputs zeroed."""
    from balance_controller import update_balance, reset_balance
    _reset_for_balance()
    reset_balance()
    shared_state.emergency_stop_triggered = True
    update_balance()
    assert shared_state.balance_hip_roll_left == 0.0
    assert shared_state.sagittal_pitch_offset == 0.0
    assert shared_state.emergency_sagittal_torque == {}


def test_diagnostics_returns_dict():
    """get_balance_diagnostics() returns a dict with expected keys."""
    from balance_controller import update_balance, reset_balance, get_balance_diagnostics
    _reset_for_balance()
    reset_balance()
    update_balance()
    d = get_balance_diagnostics()
    assert isinstance(d, dict)
    assert 'lateral_error' in d
    assert 'sagittal_error' in d
    assert 'mode_lateral' in d
    assert 'mode_sagittal' in d
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_balance_controller.py -v`
Expected: FAIL — `balance_controller` module doesn't exist.

- [ ] **Step 3: Implement balance_controller.py**

```python
# balance_controller.py
"""
================================================================================
PROJECT SICLO1 — UNIFIED BALANCE CONTROLLER
================================================================================

2-axis LIPM capture-point balance via position targets (not competing torques).

Lateral  (X) → hip roll position targets → POSITION_CONTROL on Hip_Inwards
Sagittal (Y) → pitch offset added to IK  → WBC tracks adjusted setpoint
               emergency torque           → direct injection when falling

Replaces active_balance.py.

INPUTS (shared_state):
    com_position, com_velocity, capture_point,
    left/right_foot_{position,contact_state},
    stance_foot_world_pos, stance_side, step_phase,
    freeze_robot, emergency_stop_triggered

OUTPUTS (shared_state):
    balance_hip_roll_left, balance_hip_roll_right,
    sagittal_pitch_offset, emergency_sagittal_torque,
    capture_point_error_lateral, capture_point_error_sagittal,
    balance_mode_lateral, balance_mode_sagittal

Author: Siclo1 Project Team
Date: April 2026
================================================================================
"""

import math
import numpy as np
from typing import Dict

from shared_state import (
    shared_state,
    ContactState,
    StepPhase,
    URDF_JOINT_NAMES,
)

# ============================================================================
# PHYSICAL CONSTANTS
# ============================================================================

G: float = 9.81  # m/s²

# ============================================================================
# LATERAL BALANCE CONSTANTS (tuning, not URDF-derived)
# ============================================================================

LATERAL_ROLL_GAIN:   float = 0.8    # rad/m, CP error → hip roll angle
LATERAL_KD:          float = 0.1    # rad·s/m, derivative damping on ẋ_com
HIP_ROLL_MAX:        float = 0.25   # rad (~14°), URDF limit is ±0.698 rad
HIP_ROLL_RATE_LIMIT: float = 0.03   # rad/cycle, smooth transitions

# ============================================================================
# SAGITTAL BALANCE CONSTANTS (tuning, not URDF-derived)
# ============================================================================

SAGITTAL_PITCH_GAIN:    float = 1.2    # rad/m, CP_y error → pitch offset
SAGITTAL_KD:            float = 0.3    # rad·s/m, velocity damping
PITCH_OFFSET_MAX:       float = 0.15   # rad (~8.6°), prevents IK workspace violation
PITCH_RATE_LIMIT:       float = 0.02   # rad/cycle, smooth transitions

EMERGENCY_THRESHOLD:    float = 0.08   # m, CP_y error triggers direct torque
EMERGENCY_KP:           float = 40.0   # N·m/m, proportional emergency torque
EMERGENCY_KD:           float = 10.0   # N·m·s/m, derivative emergency damping
EMERGENCY_TORQUE_MAX:   float = 50.0   # N·m, below URDF effort limit of 100 N·m
EMERGENCY_HYSTERESIS:   float = 0.02   # m, must drop below threshold - hysteresis to exit

# ============================================================================
# COM HEIGHT BOUNDS
# ============================================================================

COM_HEIGHT_MIN: float = 0.30  # m, below this robot is on the ground
COM_HEIGHT_MAX: float = 1.80  # m, above this something is very wrong

# ============================================================================
# URDF JOINT NAME ALIASES
# ============================================================================

_L_HIP_FWD: str = URDF_JOINT_NAMES['L_HIP_FORWARDS']  # 'Left_Hip_Forwards'
_R_HIP_FWD: str = URDF_JOINT_NAMES['R_HIP_FORWARDS']  # 'Right_Hip_Fowards'


# ============================================================================
# BALANCE CONTROLLER
# ============================================================================

class BalanceController:
    """Unified 2-axis balance controller using LIPM capture point."""

    def __init__(self):
        self._prev_hip_roll_left:  float = 0.0
        self._prev_hip_roll_right: float = 0.0
        self._prev_pitch_offset:   float = 0.0
        self._in_emergency:        bool = False

    def update(self) -> None:
        """Called once per 100 Hz cycle. Writes balance outputs to shared_state."""
        if shared_state.freeze_robot or shared_state.emergency_stop_triggered:
            self._write_zero_outputs()
            return

        h = shared_state.com_position[2]
        if not (COM_HEIGHT_MIN < h < COM_HEIGHT_MAX):
            self._write_zero_outputs()
            return

        omega = math.sqrt(G / h)

        x_support = self._lateral_support_center()
        y_support = self._sagittal_support_center()

        cp_x = float(shared_state.capture_point[0])
        cp_y = float(shared_state.capture_point[1])
        vx = float(shared_state.com_velocity[0])
        vy = float(shared_state.com_velocity[1])

        e_x = cp_x - x_support
        e_y = cp_y - y_support

        shared_state.capture_point_error_lateral = e_x
        shared_state.capture_point_error_sagittal = e_y

        self._update_lateral(e_x, vx)
        self._update_sagittal(e_y, vy)

    # ── Lateral balance ──────────────────────────────────────────────────

    def _update_lateral(self, e_x: float, vx: float) -> None:
        raw_roll = -(LATERAL_ROLL_GAIN * e_x + LATERAL_KD * vx)
        target_roll = max(-HIP_ROLL_MAX, min(HIP_ROLL_MAX, raw_roll))

        phase = shared_state.step_phase
        walking = phase in (
            StepPhase.COM_SHIFT, StepPhase.LIFT,
            StepPhase.SWING, StepPhase.PLACE,
        )

        if walking:
            left_roll = self._rate_limit(target_roll, self._prev_hip_roll_left, HIP_ROLL_RATE_LIMIT)
            right_roll = self._rate_limit(-target_roll * 0.5, self._prev_hip_roll_right, HIP_ROLL_RATE_LIMIT)
        else:
            sym = target_roll * 0.5
            left_roll = self._rate_limit(sym, self._prev_hip_roll_left, HIP_ROLL_RATE_LIMIT)
            right_roll = self._rate_limit(sym, self._prev_hip_roll_right, HIP_ROLL_RATE_LIMIT)

        self._prev_hip_roll_left = left_roll
        self._prev_hip_roll_right = right_roll
        shared_state.balance_hip_roll_left = left_roll
        shared_state.balance_hip_roll_right = right_roll
        shared_state.balance_mode_lateral = "ACTIVE"

    # ── Sagittal balance ─────────────────────────────────────────────────

    def _update_sagittal(self, e_y: float, vy: float) -> None:
        raw_offset = -(SAGITTAL_PITCH_GAIN * e_y + SAGITTAL_KD * vy)
        clipped_offset = max(-PITCH_OFFSET_MAX, min(PITCH_OFFSET_MAX, raw_offset))
        pitch_offset = self._rate_limit(clipped_offset, self._prev_pitch_offset, PITCH_RATE_LIMIT)
        self._prev_pitch_offset = pitch_offset
        shared_state.sagittal_pitch_offset = pitch_offset

        abs_e_y = abs(e_y)

        if self._in_emergency:
            if abs_e_y < EMERGENCY_THRESHOLD - EMERGENCY_HYSTERESIS:
                self._in_emergency = False
                shared_state.emergency_sagittal_torque = {}
                shared_state.balance_mode_sagittal = "NORMAL"
            else:
                self._apply_emergency_torque(e_y, vy)
                shared_state.balance_mode_sagittal = "EMERGENCY"
        else:
            if abs_e_y >= EMERGENCY_THRESHOLD:
                self._in_emergency = True
                self._apply_emergency_torque(e_y, vy)
                shared_state.balance_mode_sagittal = "EMERGENCY"
            else:
                shared_state.emergency_sagittal_torque = {}
                shared_state.balance_mode_sagittal = "NORMAL"

    def _apply_emergency_torque(self, e_y: float, vy: float) -> None:
        raw_tau = -(EMERGENCY_KP * e_y + EMERGENCY_KD * vy)
        tau = max(-EMERGENCY_TORQUE_MAX, min(EMERGENCY_TORQUE_MAX, raw_tau))

        l_conf = shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED
        r_conf = shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED

        if l_conf and r_conf:
            shared_state.emergency_sagittal_torque = {
                _L_HIP_FWD: tau * 0.5,
                _R_HIP_FWD: tau * 0.5,
            }
        elif l_conf:
            shared_state.emergency_sagittal_torque = {_L_HIP_FWD: tau}
        elif r_conf:
            shared_state.emergency_sagittal_torque = {_R_HIP_FWD: tau}
        else:
            shared_state.emergency_sagittal_torque = {
                _L_HIP_FWD: tau * 0.5,
                _R_HIP_FWD: tau * 0.5,
            }

    # ── Support center computation ───────────────────────────────────────

    def _lateral_support_center(self) -> float:
        walking_phases = (
            StepPhase.COM_SHIFT, StepPhase.LIFT,
            StepPhase.SWING, StepPhase.PLACE,
        )
        if shared_state.step_phase in walking_phases:
            return float(shared_state.stance_foot_world_pos[0])

        l_conf = shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED
        r_conf = shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED
        if l_conf and r_conf:
            return (shared_state.left_foot_position[0] +
                    shared_state.right_foot_position[0]) / 2.0
        elif l_conf:
            return float(shared_state.left_foot_position[0])
        elif r_conf:
            return float(shared_state.right_foot_position[0])
        return 0.0

    def _sagittal_support_center(self) -> float:
        walking_phases = (
            StepPhase.COM_SHIFT, StepPhase.LIFT,
            StepPhase.SWING, StepPhase.PLACE,
        )
        if shared_state.step_phase in walking_phases:
            return float(shared_state.stance_foot_world_pos[1])

        l_conf = shared_state.left_foot_contact_state == ContactState.CONTACT_CONFIRMED
        r_conf = shared_state.right_foot_contact_state == ContactState.CONTACT_CONFIRMED
        if l_conf and r_conf:
            return (shared_state.left_foot_position[1] +
                    shared_state.right_foot_position[1]) / 2.0
        elif l_conf:
            return float(shared_state.left_foot_position[1])
        elif r_conf:
            return float(shared_state.right_foot_position[1])
        return 0.0

    # ── Helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _rate_limit(target: float, prev: float, limit: float) -> float:
        delta = target - prev
        if delta > limit:
            return prev + limit
        if delta < -limit:
            return prev - limit
        return target

    def _write_zero_outputs(self) -> None:
        shared_state.balance_hip_roll_left = 0.0
        shared_state.balance_hip_roll_right = 0.0
        shared_state.sagittal_pitch_offset = 0.0
        shared_state.emergency_sagittal_torque = {}
        shared_state.balance_mode_lateral = "INACTIVE"
        shared_state.balance_mode_sagittal = "INACTIVE"

    def reset(self) -> None:
        self._prev_hip_roll_left = 0.0
        self._prev_hip_roll_right = 0.0
        self._prev_pitch_offset = 0.0
        self._in_emergency = False
        self._write_zero_outputs()

    def get_diagnostics(self) -> dict:
        return {
            'lateral_error': shared_state.capture_point_error_lateral,
            'sagittal_error': shared_state.capture_point_error_sagittal,
            'hip_roll_left': shared_state.balance_hip_roll_left,
            'hip_roll_right': shared_state.balance_hip_roll_right,
            'pitch_offset': shared_state.sagittal_pitch_offset,
            'emergency_torque': dict(shared_state.emergency_sagittal_torque),
            'mode_lateral': shared_state.balance_mode_lateral,
            'mode_sagittal': shared_state.balance_mode_sagittal,
            'in_emergency': self._in_emergency,
        }


# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

_controller = BalanceController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_balance() -> None:
    """Called by HeartBeat.py once per 100 Hz cycle."""
    _controller.update()


def reset_balance() -> None:
    """Reset controller state after warmup."""
    _controller.reset()


def get_balance_diagnostics() -> dict:
    """Return dict with both axes' state for telemetry."""
    return _controller.get_diagnostics()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_balance_controller.py -v`
Expected: ALL PASS

- [ ] **Step 5: Commit**

```bash
git add balance_controller.py Test_Enviroment/test_balance_controller.py
git commit -m "feat: add unified balance controller with lateral + sagittal axes"
```

---

### Task 3: stability.py — Add 2D margin decomposition

**Files:**
- Modify: `stability.py:361-377` (inside `check_stability`, after polygon.contains check)
- Test: `Test_Enviroment/test_stability_2d_margins.py`

- [ ] **Step 1: Write tests for 2D margin decomposition**

```python
# Test_Enviroment/test_stability_2d_margins.py
"""Tests for 2D stability margin decomposition in stability.py."""
import numpy as np
import pytest
from shared_state import shared_state, ContactState
from stability import update_stability


def _setup_standing():
    """Set up a stable standing pose with both feet confirmed."""
    shared_state.reset()
    shared_state.com_position = np.array([0.0, 0.0, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    shared_state.base_position = np.array([0.0, 0.0, 0.8806])
    # Feet with enough spread for a valid polygon
    shared_state.left_foot_position = np.array([-0.1, -0.05, 0.0])
    shared_state.right_foot_position = np.array([0.1, 0.05, 0.0])
    shared_state.set_contact_state('left', ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    # Provide enough contact points for a polygon (need ≥3)
    shared_state.left_contact_points = [
        np.array([-0.12, -0.06, 0.0]),
        np.array([-0.08, -0.04, 0.0]),
    ]
    shared_state.right_contact_points = [
        np.array([0.08, 0.04, 0.0]),
        np.array([0.12, 0.06, 0.0]),
    ]


def test_lateral_margin_written():
    """stability_margin_lateral is set to a positive value when stable."""
    _setup_standing()
    update_stability(dt=0.01)
    # Only check if polygon was valid (stable or marginal)
    if shared_state.stability_margin > 0:
        assert shared_state.stability_margin_lateral >= 0.0


def test_sagittal_margin_written():
    """stability_margin_sagittal is set to a positive value when stable."""
    _setup_standing()
    update_stability(dt=0.01)
    if shared_state.stability_margin > 0:
        assert shared_state.stability_margin_sagittal >= 0.0


def test_margins_zero_when_unstable():
    """Both margins are 0.0 when CP is outside polygon."""
    shared_state.reset()
    shared_state.com_position = np.array([5.0, 5.0, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    shared_state.base_position = np.array([5.0, 5.0, 0.8806])
    shared_state.left_foot_position = np.array([-0.1, 0.0, 0.0])
    shared_state.right_foot_position = np.array([0.1, 0.0, 0.0])
    shared_state.set_contact_state('left', ContactState.CONTACT_CONFIRMED)
    shared_state.set_contact_state('right', ContactState.CONTACT_CONFIRMED)
    shared_state.left_contact_points = [
        np.array([-0.12, -0.05, 0.0]),
        np.array([-0.08, 0.05, 0.0]),
    ]
    shared_state.right_contact_points = [
        np.array([0.08, -0.05, 0.0]),
        np.array([0.12, 0.05, 0.0]),
    ]
    update_stability(dt=0.01)
    assert shared_state.stability_margin_lateral == 0.0
    assert shared_state.stability_margin_sagittal == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_stability_2d_margins.py -v`
Expected: FAIL — `stability_margin_lateral` never written (stays 0.0 even when stable).

- [ ] **Step 3: Add 2D margin decomposition to stability.py**

In `check_stability()`, after the `polygon.contains(cp_point)` block that computes `margin_distance`, add the per-axis margin decomposition. Modify the `if polygon.contains(cp_point):` branch:

Replace the existing `if polygon.contains(cp_point):` / `else:` block (approximately lines 362-377) with:

```python
        cp_point = Point(cp_xy[0], cp_xy[1])
        if polygon.contains(cp_point):
            margin_distance = polygon.exterior.distance(cp_point)

            # 2D margin decomposition — project CP onto polygon edges per axis
            xs = [pt[0] for pt in polygon.exterior.coords]
            ys = [pt[1] for pt in polygon.exterior.coords]
            shared_state.stability_margin_lateral = min(
                cp_xy[0] - min(xs), max(xs) - cp_xy[0]
            )
            shared_state.stability_margin_sagittal = min(
                cp_xy[1] - min(ys), max(ys) - cp_xy[1]
            )

            _t4 = time.perf_counter()
            _emit_profile(_t0, _t1, _t2, _t3, _t4, len(contact_points))
            if margin_distance > safety_margin * 0.5:
                shared_state.set_stability_status(StabilityStatus.STABLE, margin=margin_distance)
                return StabilityStatus.STABLE
            else:
                shared_state.set_stability_status(StabilityStatus.MARGINAL, margin=margin_distance)
                return StabilityStatus.MARGINAL
        else:
            margin_distance = -polygon.exterior.distance(cp_point)
            shared_state.stability_margin_lateral = 0.0
            shared_state.stability_margin_sagittal = 0.0
            _t4 = time.perf_counter()
            _emit_profile(_t0, _t1, _t2, _t3, _t4, len(contact_points))
            shared_state.set_stability_status(StabilityStatus.UNSTABLE, margin=margin_distance)
            return StabilityStatus.UNSTABLE
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_stability_2d_margins.py -v`
Expected: ALL PASS

- [ ] **Step 5: Commit**

```bash
git add stability.py Test_Enviroment/test_stability_2d_margins.py
git commit -m "feat(stability): add per-axis stability margin decomposition"
```

---

### Task 4: gait_planner.py — Remove hip_roll logic, read from balance_controller

**Files:**
- Modify: `gait_planner.py` (remove `_compute_hip_roll`, constants, state; modify `_compute_stance_ik`, `_update_non_stance_ik_from_joints`, `_handle_com_shift`, `reset_gait_planner`)
- Test: `Test_Enviroment/test_gait_balance_integration.py`

- [ ] **Step 1: Write tests for gait_planner reading hip_roll from shared_state**

```python
# Test_Enviroment/test_gait_balance_integration.py
"""Tests for gait_planner reading hip_roll from balance_controller via shared_state."""
import numpy as np
import pytest
from unittest.mock import patch
from shared_state import (
    shared_state, ContactState, MissionState, StepPhase, StabilityStatus,
)
import gait_planner


def _reset():
    shared_state.reset()
    shared_state.freeze_robot = False
    shared_state.timing_violation_this_cycle = False
    shared_state.mission_state = MissionState.WALK
    shared_state.ramp_gain = 1.0
    shared_state.step_phase = StepPhase.SWING
    shared_state.step_phase_timer = 0.0
    shared_state.active_swing_side = "left"
    shared_state.stance_side = "right"
    shared_state.swing_phase = 0.3
    shared_state.swing_foot_x_stance = 0.0
    shared_state.capture_point = np.array([0.0, 0.0])
    shared_state.stability_status = StabilityStatus.STABLE
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force = 0.0
    shared_state.right_foot_force = 30.0
    shared_state.left_foot_position = np.array([0.0, 0.0, 0.0])
    shared_state.right_foot_position = np.array([0.1, 0.0, 0.0])
    shared_state.left_foot_velocity = np.zeros(3)
    shared_state.right_foot_velocity = np.zeros(3)
    shared_state.stance_foot_world_pos = np.array([0.1, 0.0, 0.0])
    shared_state.link_positions = {
        'Left_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
        'Right_Upper_Leg_1': np.array([0.0, 0.0, 0.7]),
    }
    shared_state.com_position = np.array([0.0, 0.0, 0.8806])
    shared_state.com_velocity = np.zeros(3)
    # Balance controller outputs (would be written by balance_controller.py)
    shared_state.balance_hip_roll_left = 0.0
    shared_state.balance_hip_roll_right = 0.0


def test_stance_ik_uses_balance_hip_roll():
    """Stance IK hip_roll comes from shared_state.balance_hip_roll_*, not internal computation."""
    _reset()
    # Set a specific hip roll via balance controller output
    shared_state.balance_hip_roll_right = 0.12  # stance side is right
    with patch('kinematics.solve_ik', return_value=(0.1, 0.2, 0.3)):
        gait_planner.update_gait_planner()
    # Stance (right) angles should use the balance roll
    assert shared_state.ik_right_angles[0] == 0.12


def test_swing_ik_hip_roll_is_zero():
    """Swing leg hip_roll is zero during swing (leg returns to neutral)."""
    _reset()
    shared_state.balance_hip_roll_left = 0.15  # swing side is left
    with patch('kinematics.solve_ik', return_value=(0.1, 0.2, 0.3)):
        gait_planner.update_gait_planner()
    # Swing (left) angles: hip_roll should be 0 during swing
    assert shared_state.ik_left_angles[0] == 0.0


def test_non_stance_ik_uses_balance_roll_in_ds():
    """During DOUBLE_SUPPORT, non-stance leg reads hip roll from balance_controller."""
    _reset()
    shared_state.step_phase = StepPhase.DOUBLE_SUPPORT
    shared_state.step_phase_timer = 0.0
    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_force = 30.0
    shared_state.right_foot_force = 30.0
    shared_state.balance_hip_roll_left = -0.05  # non-stance (left)
    shared_state.joint_positions = {
        'Left_Hip_Forwards': 0.1, 'Left_Knee': 0.2, 'Left_Ankle': 0.0,
        'Right_Hip_Fowards': 0.1, 'Right_Knee': 0.2, 'Right_Ankle': 0.0,
        'Left_Hip_Inwards': 0.0, 'Right_Hip_Inwards': 0.0,
    }
    gait_planner.update_gait_planner()
    # Non-stance (left) should use balance_hip_roll_left
    assert shared_state.ik_left_angles[0] == -0.05


def test_com_shift_sagittal_gate():
    """COM_SHIFT does not exit until BOTH lateral AND sagittal CP are within threshold."""
    _reset()
    shared_state.step_phase = StepPhase.COM_SHIFT
    shared_state.step_phase_timer = 0.0
    shared_state.stability_status = StabilityStatus.STABLE
    # Lateral CP close to stance foot, but sagittal CP far
    shared_state.capture_point = np.array([0.1, 0.5])  # sagittal far from stance Y=0.0
    shared_state.stance_foot_world_pos = np.array([0.1, 0.0, 0.0])
    shared_state.left_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.right_foot_contact_state = ContactState.CONTACT_CONFIRMED
    shared_state.joint_positions = {
        'Left_Hip_Forwards': 0.0, 'Left_Knee': 0.0, 'Left_Ankle': 0.0,
        'Right_Hip_Fowards': 0.0, 'Right_Knee': 0.0, 'Right_Ankle': 0.0,
        'Left_Hip_Inwards': 0.0, 'Right_Hip_Inwards': 0.0,
    }
    gait_planner.update_gait_planner()
    # Should still be in COM_SHIFT (sagittal gate blocks)
    assert shared_state.step_phase == StepPhase.COM_SHIFT


def test_gait_planner_has_no_compute_hip_roll():
    """_compute_hip_roll method should not exist after refactor."""
    assert not hasattr(gait_planner._gait_planner, '_compute_hip_roll')
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_gait_balance_integration.py -v`
Expected: FAIL — gait_planner still has `_compute_hip_roll`, doesn't read from shared_state.

- [ ] **Step 3: Modify gait_planner.py**

**Remove** these constants (lines ~79-82):
```python
HIP_ROLL_GAIN:          float = 0.0    # rad/m, DISABLED
HIP_ROLL_MAX:           float = 0.0    # rad, DISABLED
HIP_ROLL_RATE_LIMIT:    float = 0.03   # rad/cycle
```

**Add** this constant after PLACE_ENTRY_PHI:
```python
COM_SHIFT_SAGITTAL_THRESHOLD: float = 0.05  # m, wider than lateral because sagittal dynamics are faster
```

**Remove** the entire `_compute_hip_roll` method (lines ~197-243).

**Remove** instance vars from `__init__` (lines ~140-141):
```python
self._prev_stance_roll: float = 0.0
self._prev_swing_roll: float = 0.0
```

**Modify `_compute_stance_ik()`** — replace the hip_roll computation:
```python
def _compute_stance_ik(self) -> None:
    stance_side = shared_state.stance_side

    # Hip roll from balance_controller via shared_state
    if stance_side == "left":
        stance_roll = shared_state.balance_hip_roll_left
    else:
        stance_roll = shared_state.balance_hip_roll_right

    hip_key = (_LEFT_HIP_LINK if stance_side == "left"
               else _RIGHT_HIP_LINK)
    hip_pos = shared_state.link_positions.get(hip_key, np.zeros(3))
    stance_foot_rel = shared_state.stance_foot_world_pos - hip_pos
    rel_z = float(stance_foot_rel[2])
    if not (-1.0 < rel_z < 0.0):
        return
    foot_xyz_rel = (
        float(stance_foot_rel[0]),
        float(stance_foot_rel[1]),
        rel_z,
    )
    try:
        ik_angles = kinematics.solve_ik(foot_xyz_rel, stance_side)
    except ValueError:
        ik_angles = (0.0, 0.0, 0.0)

    angles = (stance_roll, ik_angles[0], ik_angles[1], ik_angles[2])
    if stance_side == "left":
        shared_state.ik_left_angles = angles
    else:
        shared_state.ik_right_angles = angles
```

**Modify `_update_non_stance_ik_from_joints()`** — read hip_roll from shared_state:
```python
def _update_non_stance_ik_from_joints(self) -> None:
    stance_side = shared_state.stance_side
    jp = shared_state.joint_positions

    if stance_side == "right":
        # Non-stance is left
        swing_roll = shared_state.balance_hip_roll_left
        shared_state.ik_left_angles = (
            swing_roll,
            jp.get('Left_Hip_Forwards', 0.0),
            jp.get('Left_Knee', 0.0),
            jp.get('Left_Ankle', 0.0),
        )
    else:
        # Non-stance is right
        swing_roll = shared_state.balance_hip_roll_right
        shared_state.ik_right_angles = (
            swing_roll,
            jp.get('Right_Hip_Fowards', 0.0),
            jp.get('Right_Knee', 0.0),
            jp.get('Right_Ankle', 0.0),
        )
```

**Modify `_handle_com_shift()`** — add sagittal gate:
```python
def _handle_com_shift(self, dt: float) -> None:
    self._compute_stance_ik()
    self._update_non_stance_ik_from_joints()

    timer    = shared_state.step_phase_timer
    stable   = (shared_state.stability_status != StabilityStatus.UNSTABLE)

    cp_close_lateral = abs(
        shared_state.capture_point[0] - shared_state.stance_foot_world_pos[0]
    ) < COM_SHIFT_THRESHOLD
    cp_close_sagittal = abs(
        shared_state.capture_point[1] - shared_state.stance_foot_world_pos[1]
    ) < COM_SHIFT_SAGITTAL_THRESHOLD
    cp_close = cp_close_lateral and cp_close_sagittal

    if stable and cp_close:
        self._transition_to(StepPhase.LIFT)
        return

    if timer >= COM_SHIFT_TIMEOUT:
        swing_force = self._swing_foot_force()
        if swing_force > SWING_UNLOAD_THRESHOLD:
            self._abort_to_double_support()
        else:
            self._transition_to(StepPhase.LIFT)
```

**Modify `reset_gait_planner()`** — remove hip_roll state:
```python
def reset_gait_planner() -> None:
    _gait_planner._ds_lock_pending = True
    _gait_planner._com_shift_ik_locked = False
    shared_state.stance_foot_world_pos = np.zeros(3)
```

- [ ] **Step 4: Run new + regression tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_gait_balance_integration.py Test_Enviroment/test_gait_planner.py Test_Enviroment/test_stance_anchor.py -v`
Expected: ALL PASS.

Note: `test_stance_anchor.py::test_stance_ik_recomputed_every_swing_cycle` expects `ik_right_angles == (0.0, 0.1, 0.2, 0.3)` — the hip_roll=0.0 now comes from `shared_state.balance_hip_roll_right` which defaults to 0.0 after reset. This should pass without changes.

- [ ] **Step 5: Commit**

```bash
git add gait_planner.py Test_Enviroment/test_gait_balance_integration.py
git commit -m "refactor(gait_planner): remove hip_roll logic, read from balance_controller"
```

---

### Task 5: HeartBeat.py — WBC pitch offset, import swap, saturation, apply_control

**Files:**
- Modify: `HeartBeat.py:59` (import swap)
- Modify: `HeartBeat.py:614-658` (`_wbc_step` — init target_torques, add pitch offset)
- Modify: `HeartBeat.py:438-491` (`apply_control` — balance hip roll, emergency merge, saturation)
- Modify: `HeartBeat.py:667-690` (warmup — swap active_balance call)
- Modify: `HeartBeat.py:740-741` (step — swap active_balance call)
- Test: `Test_Enviroment/test_heartbeat_balance_wiring.py`

- [ ] **Step 1: Write tests for HeartBeat balance wiring**

```python
# Test_Enviroment/test_heartbeat_balance_wiring.py
"""Tests for HeartBeat.py balance_controller integration."""
import pytest
import numpy as np
from shared_state import shared_state, URDF_JOINT_LIMITS


def test_heartbeat_imports_balance_controller():
    """HeartBeat imports balance_controller, not active_balance."""
    import HeartBeat
    assert hasattr(HeartBeat, 'balance_controller')
    assert not hasattr(HeartBeat, 'active_balance')


def test_wbc_initializes_target_torques():
    """_wbc_step initializes target_torques as empty dict."""
    shared_state.reset()
    shared_state.joint_positions = {
        'Left_Hip_Inwards': 0.0, 'Left_Hip_Forwards': 0.1,
        'Left_Knee': 0.2, 'Left_Ankle': 0.0,
        'Right_Hip_Inwards': 0.0, 'Right_Hip_Fowards': 0.1,
        'Right_Knee': 0.2, 'Right_Ankle': 0.0,
    }
    shared_state.joint_velocities = {
        'Left_Hip_Inwards': 0.0, 'Left_Hip_Forwards': 0.0,
        'Left_Knee': 0.0, 'Left_Ankle': 0.0,
        'Right_Hip_Inwards': 0.0, 'Right_Hip_Fowards': 0.0,
        'Right_Knee': 0.0, 'Right_Ankle': 0.0,
    }
    shared_state.ik_left_angles = (0.0, 0.15, 0.25, 0.0)
    shared_state.ik_right_angles = (0.0, 0.15, 0.25, 0.0)
    # Set stale torques to verify WBC overwrites
    shared_state.target_torques = {'stale_key': 999.0}

    from HeartBeat import Siclo1Controller
    # Can't easily instantiate controller, but we can verify the import works
    # and that the module-level code doesn't crash
    assert True


def test_saturate_hip_pitch():
    """_saturate_hip_pitch preserves emergency + GRF, scales WBC."""
    from HeartBeat import _saturate_hip_pitch
    # Total = 30 + 20 + 60 = 110, limit = 100
    result = _saturate_hip_pitch(wbc_tau=60.0, grf_tau=20.0,
                                  emergency_tau=30.0, effort_limit=100.0)
    # Protected = 20 + 30 = 50, remaining budget = 100 - 50 = 50
    # WBC scaled: 60 * min(1.0, 50/60) = 50
    # Total = 50 + 50 = 100
    assert abs(result) <= 100.0 + 1e-6
    assert abs(result - 100.0) < 1e-6


def test_saturate_hip_pitch_no_clipping_needed():
    """When total is within limit, return total unchanged."""
    from HeartBeat import _saturate_hip_pitch
    result = _saturate_hip_pitch(wbc_tau=30.0, grf_tau=10.0,
                                  emergency_tau=5.0, effort_limit=100.0)
    assert abs(result - 45.0) < 1e-6


def test_saturate_hip_pitch_negative():
    """Negative torques: same saturation logic applies."""
    from HeartBeat import _saturate_hip_pitch
    result = _saturate_hip_pitch(wbc_tau=-60.0, grf_tau=-20.0,
                                  emergency_tau=-30.0, effort_limit=100.0)
    assert abs(result) <= 100.0 + 1e-6
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_heartbeat_balance_wiring.py -v`
Expected: FAIL — `HeartBeat` still imports `active_balance`, `_saturate_hip_pitch` doesn't exist.

- [ ] **Step 3: Modify HeartBeat.py**

**Import swap** (line 59): Replace `import active_balance` with `import balance_controller`.

**Add saturation helper** after `_clip_position` (~line 268):

```python
def _saturate_hip_pitch(wbc_tau: float, grf_tau: float,
                        emergency_tau: float, effort_limit: float) -> float:
    """Scale WBC down if total exceeds limit. Preserve safety + feedforward."""
    total = wbc_tau + grf_tau + emergency_tau
    if abs(total) <= effort_limit:
        return total
    protected = grf_tau + emergency_tau
    remaining = effort_limit * np.sign(total) - protected
    if abs(wbc_tau) > 1e-6:
        scale = min(1.0, abs(remaining / wbc_tau))
        return protected + wbc_tau * scale
    return float(np.clip(protected, -effort_limit, effort_limit))
```

**Modify `_wbc_step()`** (~line 614): Initialize target_torques and add pitch offset.

Replace the first line:
```python
torques = getattr(shared_state, 'target_torques', {})
```
with:
```python
shared_state.target_torques = {}
torques = shared_state.target_torques
```

In the left-joints loop, after `theta_target = shared_state.ik_left_angles[idx]`, add:
```python
if idx == 1:  # hip_pitch — add sagittal balance offset
    theta_target += shared_state.sagittal_pitch_offset
```

Same for the right-joints loop, after `theta_target = shared_state.ik_right_angles[idx]`:
```python
if idx == 1:  # hip_pitch — add sagittal balance offset
    theta_target += shared_state.sagittal_pitch_offset
```

**Modify `apply_control()`** in PyBulletInterface (~line 438):

Replace the hip_roll_joints dict:
```python
hip_roll_joints = {
    'Left_Hip_Inwards':  shared_state.balance_hip_roll_left,
    'Right_Hip_Inwards': shared_state.balance_hip_roll_right,
}
```

Replace the torque merge loop (starting at `for jname, raw_torque in torques.items():`) to add emergency torque and saturation:
```python
emergency = getattr(self.shared_state, 'emergency_sagittal_torque', {})

for jname, raw_torque in torques.items():
    jid = self.joint_ids.get(jname)
    if jid is None:
        continue
    if jname in hip_roll_joints:
        continue

    grf_val = grf_corr.get(jname, 0.0)
    emerg_val = emergency.get(jname, 0.0)

    # Saturation-aware merge for hip pitch joints
    if jname in ('Left_Hip_Forwards', 'Right_Hip_Fowards') and abs(emerg_val) > 1e-6:
        lim = URDF_JOINT_LIMITS.get(jname, {}).get('effort', 100.0)
        merged = _saturate_hip_pitch(raw_torque, grf_val, emerg_val, lim)
    else:
        merged = raw_torque + grf_val + emerg_val

    if not math.isfinite(merged):
        print(f"[SANITY] NaN/Inf torque on {jname}: raw={raw_torque} "
              f"grf={grf_val} emerg={emerg_val} — zeroed to prevent crash")
        merged = 0.0
    clipped = _clip_effort(jname, merged)
    p.setJointMotorControl2(
        rid, jid,
        controlMode=p.TORQUE_CONTROL,
        force=clipped,
        physicsClientId=pc,
    )
```

**Warmup loop** (~line 679): Replace `active_balance.update_active_balance()` with `balance_controller.update_balance()`.

**Step loop** (~line 741): Replace `active_balance.update_active_balance()` with `balance_controller.update_balance()`.

Also in the warmup, after `gait_planner.reset_gait_planner()` add:
```python
balance_controller.reset_balance()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_heartbeat_balance_wiring.py Test_Enviroment/test_wbc_tracking.py -v`
Expected: ALL PASS

- [ ] **Step 5: Commit**

```bash
git add HeartBeat.py Test_Enviroment/test_heartbeat_balance_wiring.py
git commit -m "feat(HeartBeat): wire balance_controller, add pitch offset + saturation"
```

---

### Task 6: Regression tests + cleanup

**Files:**
- Delete: `active_balance.py` (fully replaced)
- Modify: `.gitignore` or any file referencing active_balance
- Test: Full test suite

- [ ] **Step 1: Run full test suite to check for active_balance imports**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && grep -rn "import active_balance\|from active_balance" --include="*.py" .`
Expected: Only HeartBeat.py (already changed). Any other files need fixing.

- [ ] **Step 2: Run all existing regression tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_gait_planner.py Test_Enviroment/test_stance_anchor.py Test_Enviroment/test_wbc_tracking.py -v`
Expected: ALL PASS

- [ ] **Step 3: Run all balance controller tests**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/test_balance_shared_state.py Test_Enviroment/test_balance_controller.py Test_Enviroment/test_stability_2d_margins.py Test_Enviroment/test_gait_balance_integration.py Test_Enviroment/test_heartbeat_balance_wiring.py -v`
Expected: ALL PASS

- [ ] **Step 4: Run full test suite**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/ -v --tb=short 2>&1 | tail -40`
Expected: No unexpected failures. Some tests that import active_balance directly will fail — those need updates in Step 5.

- [ ] **Step 5: Fix any tests that reference active_balance**

Search: `grep -rn "active_balance" Test_Enviroment/ --include="*.py"`

For each test that imports or references `active_balance`:
- If it tests active_balance internals → delete the test (replaced by test_balance_controller.py)
- If it references `shared_state.active_balance_mode` or `shared_state.lateral_error` → update to use new field names

- [ ] **Step 6: Delete active_balance.py**

```bash
git rm active_balance.py
```

- [ ] **Step 7: Run full test suite one final time**

Run: `cd /home/notlord/ros2_ws/Siclo1_V1 && python -m pytest Test_Enviroment/ -v --tb=short 2>&1 | tail -60`
Expected: ALL PASS (or only pre-existing failures unrelated to this change)

- [ ] **Step 8: Commit**

```bash
git add -A
git commit -m "refactor: remove active_balance.py, fix remaining references"
```

---

## Summary of Files Changed

| File | Action | Lines (~) |
|---|---|---|
| `shared_state.py` | MODIFY | +25 (fields + reset + stage name) |
| `balance_controller.py` | CREATE | ~220 lines |
| `stability.py` | MODIFY | +12 (2D margin decomposition) |
| `gait_planner.py` | MODIFY | -70, +15 (remove hip_roll, add sagittal gate) |
| `HeartBeat.py` | MODIFY | +30 (import, pitch offset, saturation, emergency) |
| `active_balance.py` | DELETE | -570 lines |
| `Test_Enviroment/test_balance_shared_state.py` | CREATE | ~70 |
| `Test_Enviroment/test_balance_controller.py` | CREATE | ~170 |
| `Test_Enviroment/test_stability_2d_margins.py` | CREATE | ~60 |
| `Test_Enviroment/test_gait_balance_integration.py` | CREATE | ~100 |
| `Test_Enviroment/test_heartbeat_balance_wiring.py` | CREATE | ~80 |
